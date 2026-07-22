#define _POSIX_C_SOURCE 200809L

#include "course.h"
#include "test_harness.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

enum {
    DAY4_MAX_FAULTS = 16,
    DAY4_MAX_LOCK_ORDERS = 8,
    DAY4_TEXT_SIZE = 128,
    LOCK_CHILD_COMPLETED = 10,
    LOCK_CHILD_REJECTED = 11,
    LOCK_CHILD_INVALID = 12
};

#define DAY4_LOCK_PROBE_DEADLINE_NS UINT64_C(100000000)

typedef struct {
    char fixture_id[DAY4_TEXT_SIZE];
    char category[DAY4_TEXT_SIZE];
    char input[DAY4_TEXT_SIZE];
    Verdict expected_verdict;
    Reason expected_reason;
    size_t expected_writes;
} Day4FaultFixture;

typedef struct {
    char fixture_id[DAY4_TEXT_SIZE];
    char acquire_order[DAY4_TEXT_SIZE];
    char expected_result[DAY4_TEXT_SIZE];
} Day4LockFixture;

typedef struct {
    const Day4LockFixture *fixture;
    bool terminated;
    bool rejected_before_wait;
    bool timed_out;
    uint64_t elapsed_ns;
    bool passed;
} LockObservation;

typedef struct {
    const Day4FaultFixture *fixture;
    SafetyDecision decision;
    size_t actuator_writes;
    bool behavior_ok;
    bool passed;
} FaultObservation;

typedef struct {
    ActuatorWriter writer;
    Simulator simulator;
    bool available;
} RaceArtifacts;

static void check_or_todo(TestContext *context, bool condition,
                          const char *label) {
    if (condition) {
        test_check(context, true, label);
    } else {
        test_todo(context, label);
    }
}

static void trim_line(char *line) {
    size_t length = strlen(line);

    while (length > 0 &&
           (line[length - 1] == '\n' || line[length - 1] == '\r')) {
        line[--length] = '\0';
    }
}

static size_t split_fields(char *line, char separator, char **fields,
                           size_t capacity) {
    size_t count = 0;
    char *start = line;

    if (capacity == 0) {
        return 0;
    }
    for (char *cursor = line;; cursor++) {
        if (*cursor != separator && *cursor != '\0') {
            continue;
        }
        if (count >= capacity) {
            return capacity + 1;
        }
        fields[count++] = start;
        if (*cursor == '\0') {
            break;
        }
        *cursor = '\0';
        start = cursor + 1;
    }
    return count;
}

static bool copy_text(char *destination, size_t capacity,
                      const char *source) {
    const size_t length = strlen(source);

    if (length >= capacity) {
        return false;
    }
    memcpy(destination, source, length + 1);
    return true;
}

static bool parse_u64(const char *text, uint64_t *value) {
    char *end = NULL;
    unsigned long long parsed;

    errno = 0;
    parsed = strtoull(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0') {
        return false;
    }
    *value = (uint64_t)parsed;
    return true;
}

static bool parse_size(const char *text, size_t *value) {
    uint64_t parsed;

    if (!parse_u64(text, &parsed) || parsed > SIZE_MAX) {
        return false;
    }
    *value = (size_t)parsed;
    return true;
}

static bool parse_verdict(const char *text, Verdict *verdict) {
    if (strcmp(text, "APPROVE") == 0) {
        *verdict = COURSE_VERDICT_APPROVE;
        return true;
    }
    if (strcmp(text, "REJECT") == 0) {
        *verdict = COURSE_VERDICT_REJECT;
        return true;
    }
    if (strcmp(text, "FALLBACK") == 0) {
        *verdict = COURSE_VERDICT_FALLBACK;
        return true;
    }
    if (strcmp(text, "DISCARD") == 0) {
        *verdict = COURSE_VERDICT_DISCARD;
        return true;
    }
    return false;
}

static bool parse_reason(const char *text, Reason *reason) {
    struct ReasonName {
        const char *name;
        Reason value;
    };
    static const struct ReasonName names[] = {
        {"NONE", COURSE_REASON_NONE},
        {"NONFINITE", COURSE_REASON_NONFINITE},
        {"JOINT_RANGE", COURSE_REASON_JOINT_RANGE},
        {"RATE_LIMIT", COURSE_REASON_RATE_LIMIT},
        {"STALE_STATE", COURSE_REASON_STALE_STATE},
        {"UNCERTAINTY", COURSE_REASON_UNCERTAINTY},
        {"INTERNAL", COURSE_REASON_INTERNAL},
    };

    for (size_t index = 0; index < sizeof(names) / sizeof(names[0]); index++) {
        if (strcmp(text, names[index].name) == 0) {
            *reason = names[index].value;
            return true;
        }
    }
    return false;
}

static bool load_fault_fixtures(Day4FaultFixture *fixtures,
                                size_t capacity, size_t *fixture_count) {
    static const char expected_header[] =
        "fixture_id,category,input,expected_verdict,expected_reason,"
        "expected_writes";
    FILE *file = fopen("fixtures/day4/faults.csv", "r");
    char line[512];
    size_t count = 0;
    bool valid = true;

    if (file == NULL || fgets(line, sizeof(line), file) == NULL) {
        if (file != NULL) {
            fclose(file);
        }
        return false;
    }
    trim_line(line);
    valid = strcmp(line, expected_header) == 0;
    while (valid && fgets(line, sizeof(line), file) != NULL) {
        char *fields[6];

        trim_line(line);
        if (line[0] == '\0') {
            continue;
        }
        if (count >= capacity || split_fields(line, ',', fields, 6) != 6 ||
            !copy_text(fixtures[count].fixture_id,
                       sizeof(fixtures[count].fixture_id), fields[0]) ||
            !copy_text(fixtures[count].category,
                       sizeof(fixtures[count].category), fields[1]) ||
            !copy_text(fixtures[count].input,
                       sizeof(fixtures[count].input), fields[2]) ||
            !parse_verdict(fields[3], &fixtures[count].expected_verdict) ||
            !parse_reason(fields[4], &fixtures[count].expected_reason) ||
            !parse_size(fields[5], &fixtures[count].expected_writes)) {
            valid = false;
            break;
        }
        count++;
    }
    valid = valid && !ferror(file) && count > 0;
    fclose(file);
    *fixture_count = count;
    return valid;
}

static bool load_lock_fixtures(Day4LockFixture *fixtures,
                               size_t capacity, size_t *fixture_count) {
    static const char expected_header[] =
        "fixture_id,acquire_order,expected_result";
    FILE *file = fopen("fixtures/day4/lock_orders.csv", "r");
    char line[512];
    size_t count = 0;
    bool valid = true;

    if (file == NULL || fgets(line, sizeof(line), file) == NULL) {
        if (file != NULL) {
            fclose(file);
        }
        return false;
    }
    trim_line(line);
    valid = strcmp(line, expected_header) == 0;
    while (valid && fgets(line, sizeof(line), file) != NULL) {
        char *fields[3];

        trim_line(line);
        if (line[0] == '\0') {
            continue;
        }
        if (count >= capacity || split_fields(line, ',', fields, 3) != 3 ||
            !copy_text(fixtures[count].fixture_id,
                       sizeof(fixtures[count].fixture_id), fields[0]) ||
            !copy_text(fixtures[count].acquire_order,
                       sizeof(fixtures[count].acquire_order), fields[1]) ||
            !copy_text(fixtures[count].expected_result,
                       sizeof(fixtures[count].expected_result), fields[2])) {
            valid = false;
            break;
        }
        count++;
    }
    valid = valid && !ferror(file) && count > 0;
    fclose(file);
    *fixture_count = count;
    return valid;
}

static const Day4FaultFixture *find_fault_fixture(
    const Day4FaultFixture *fixtures, size_t fixture_count,
    const char *category) {
    for (size_t index = 0; index < fixture_count; index++) {
        if (strcmp(fixtures[index].category, category) == 0) {
            return &fixtures[index];
        }
    }
    return NULL;
}

static const Day4LockFixture *find_lock_fixture(
    const Day4LockFixture *fixtures, size_t fixture_count,
    const char *expected_result) {
    for (size_t index = 0; index < fixture_count; index++) {
        if (strcmp(fixtures[index].expected_result, expected_result) == 0) {
            return &fixtures[index];
        }
    }
    return NULL;
}

static bool fault_fixture_contract_ok(const Day4FaultFixture *fixtures,
                                      size_t fixture_count) {
    static const char *const categories[] = {
        "check_write_race", "reversed_lock_order", "nan_command",
        "joint_range",      "joint_rate",          "stale_state",
        "uncertainty",
    };

    if (fixture_count != sizeof(categories) / sizeof(categories[0])) {
        return false;
    }
    for (size_t index = 0; index < sizeof(categories) / sizeof(categories[0]);
         index++) {
        if (find_fault_fixture(fixtures, fixture_count, categories[index]) ==
            NULL) {
            return false;
        }
    }
    return true;
}

static bool lock_fixture_contract_ok(const Day4LockFixture *fixtures,
                                     size_t fixture_count) {
    const Day4LockFixture *valid =
        find_lock_fixture(fixtures, fixture_count, "terminate_one_writer");
    const Day4LockFixture *reversed =
        find_lock_fixture(fixtures, fixture_count, "reject_before_wait");

    return fixture_count == 2 && valid != NULL && reversed != NULL &&
           strcmp(valid->acquire_order, "queue>state>wire") == 0 &&
           strcmp(reversed->acquire_order, "wire>state") == 0;
}

static int lock_rank(const char *name) {
    if (strcmp(name, "queue") == 0) {
        return 1;
    }
    if (strcmp(name, "state") == 0) {
        return 2;
    }
    if (strcmp(name, "wire") == 0) {
        return 3;
    }
    return 0;
}

static int classify_lock_order(const char *order) {
    char copy[DAY4_TEXT_SIZE];
    char *fields[8];
    size_t field_count;
    int previous_rank = 0;

    if (!copy_text(copy, sizeof(copy), order)) {
        return LOCK_CHILD_INVALID;
    }
    field_count = split_fields(copy, '>', fields,
                               sizeof(fields) / sizeof(fields[0]));
    if (field_count == 0 || field_count >
                                sizeof(fields) / sizeof(fields[0])) {
        return LOCK_CHILD_INVALID;
    }
    for (size_t index = 0; index < field_count; index++) {
        const int rank = lock_rank(fields[index]);

        if (rank == 0) {
            return LOCK_CHILD_INVALID;
        }
        if (rank <= previous_rank) {
            return LOCK_CHILD_REJECTED;
        }
        previous_rank = rank;
    }
    return LOCK_CHILD_COMPLETED;
}

static LockObservation run_lock_probe(const Day4LockFixture *fixture) {
    const uint64_t start_ns = monotonic_ns();
    LockObservation observation;
    pid_t child;
    int status = 0;

    memset(&observation, 0, sizeof(observation));
    observation.fixture = fixture;
    child = fork();
    if (child < 0) {
        return observation;
    }
    if (child == 0) {
        _exit(classify_lock_order(fixture->acquire_order));
    }

    for (;;) {
        const pid_t result = waitpid(child, &status, WNOHANG);
        const uint64_t now_ns = monotonic_ns();

        if (result == child) {
            observation.terminated = true;
            observation.elapsed_ns =
                now_ns >= start_ns ? now_ns - start_ns : 0;
            break;
        }
        if (result < 0 && errno != EINTR) {
            break;
        }
        if (now_ns >= start_ns &&
            now_ns - start_ns >= DAY4_LOCK_PROBE_DEADLINE_NS) {
            observation.timed_out = true;
            (void)kill(child, SIGKILL);
            while (waitpid(child, &status, 0) < 0 && errno == EINTR) {
            }
            observation.elapsed_ns = now_ns - start_ns;
            break;
        }
        {
            const struct timespec pause = {
                .tv_sec = 0,
                .tv_nsec = 1000000,
            };

            (void)nanosleep(&pause, NULL);
        }
    }

    if (observation.terminated && WIFEXITED(status)) {
        const int exit_code = WEXITSTATUS(status);

        observation.rejected_before_wait =
            exit_code == LOCK_CHILD_REJECTED;
        if (strcmp(fixture->expected_result, "reject_before_wait") == 0) {
            observation.passed = observation.rejected_before_wait;
        } else if (strcmp(fixture->expected_result,
                          "terminate_one_writer") == 0) {
            observation.passed = exit_code == LOCK_CHILD_COMPLETED;
        }
    }
    observation.passed = observation.passed && !observation.timed_out &&
                         observation.elapsed_ns <
                             DAY4_LOCK_PROBE_DEADLINE_NS;
    return observation;
}

static ArmState valid_state(uint64_t now_ns) {
    ArmState state;

    memset(&state, 0, sizeof(state));
    state.seq = 401;
    state.t_mono_ns = (int64_t)(now_ns - UINT64_C(10000000));
    state.frame_id = COURSE_FRAME_ID_BASE;
    for (size_t joint = 0; joint < COURSE_ARM_DOF; joint++) {
        state.sigma_q_rad[joint] = 0.01f;
    }
    return state;
}

static ArmCommand valid_command(uint64_t now_ns) {
    ArmCommand command;

    memset(&command, 0, sizeof(command));
    command.seq = 402;
    command.trace_id = 4002;
    command.t_source_ns = now_ns - UINT64_C(5000000);
    command.q_target_rad[0] = 0.01;
    command.q_target_rad[1] = -0.01;
    command.q_target_rad[2] = 0.005;
    return command;
}

static bool parse_input_double(const char *input, const char *prefix,
                               const char *required_suffix, double *value) {
    const size_t prefix_length = strlen(prefix);
    char *end = NULL;

    if (strncmp(input, prefix, prefix_length) != 0) {
        return false;
    }
    errno = 0;
    *value = strtod(input + prefix_length, &end);
    return errno == 0 && end != input + prefix_length &&
           strcmp(end, required_suffix) == 0;
}

static bool parse_input_u64(const char *input, const char *prefix,
                            uint64_t *value) {
    const size_t prefix_length = strlen(prefix);

    return strncmp(input, prefix, prefix_length) == 0 &&
           parse_u64(input + prefix_length, value);
}

static FaultObservation run_race_fault(const Day4FaultFixture *fixture,
                                       uint64_t now_ns,
                                       RaceArtifacts *artifacts) {
    FaultObservation observation;
    ArmState state = valid_state(now_ns);
    ArmCommand source = valid_command(now_ns);
    const ArmCommand submitted_copy = source;
    bool submitted;
    bool pumped;

    memset(&observation, 0, sizeof(observation));
    observation.fixture = fixture;
    observation.decision = safety_gate(&state, &submitted_copy, now_ns);
    writer_init(&artifacts->writer, 44);
    simulator_reset(&artifacts->simulator);
    artifacts->available = true;

    submitted = actuator_submit(&artifacts->writer, &source);
    source.seq = 9999;
    source.q_target_rad[0] = COURSE_JOINT_MAX_RAD + 1.0;
    pumped = actuator_pump(&artifacts->writer, &state, now_ns,
                           &artifacts->simulator);
    observation.actuator_writes = artifacts->simulator.write_count;
    if (artifacts->writer.audit_count > 0) {
        observation.decision.verdict = artifacts->writer.audit[0].verdict;
        observation.decision.reason = artifacts->writer.audit[0].reason;
    }
    observation.behavior_ok =
        strcmp(fixture->input, "mutate_source_after_submit") == 0 &&
        submitted && pumped && artifacts->simulator.write_count == 1 &&
        artifacts->simulator.last_command_seq == submitted_copy.seq &&
        artifacts->simulator.last_writer_id == 44 &&
        artifacts->writer.audit_count == 1 &&
        artifacts->writer.audit[0].writer_id == 44;
    observation.passed =
        observation.behavior_ok &&
        observation.decision.verdict == fixture->expected_verdict &&
        observation.decision.reason == fixture->expected_reason &&
        observation.actuator_writes == fixture->expected_writes;
    return observation;
}

static FaultObservation run_lock_fault(
    const Day4FaultFixture *fixture,
    const LockObservation *reversed_observation) {
    FaultObservation observation;

    memset(&observation, 0, sizeof(observation));
    observation.fixture = fixture;
    observation.decision.verdict = COURSE_VERDICT_REJECT;
    observation.decision.reason = reversed_observation->passed
                                      ? COURSE_REASON_INTERNAL
                                      : COURSE_REASON_DEADLINE_MISS;
    observation.behavior_ok =
        strcmp(fixture->input, "wire_before_state") == 0 &&
        reversed_observation->terminated &&
        reversed_observation->rejected_before_wait &&
        !reversed_observation->timed_out;
    observation.passed =
        observation.behavior_ok &&
        observation.decision.verdict == fixture->expected_verdict &&
        observation.decision.reason == fixture->expected_reason &&
        fixture->expected_writes == 0;
    return observation;
}

static FaultObservation run_safety_fault(const Day4FaultFixture *fixture,
                                         uint64_t now_ns) {
    FaultObservation observation;
    ArmState state = valid_state(now_ns);
    ArmCommand command = valid_command(now_ns);
    ActuatorWriter writer;
    Simulator simulator;
    bool input_ok = false;

    memset(&observation, 0, sizeof(observation));
    observation.fixture = fixture;
    if (strcmp(fixture->category, "nan_command") == 0) {
        double value;

        input_ok = parse_input_double(fixture->input, "q_target_rad_0=", "",
                                      &value) &&
                   isnan(value);
        command.q_target_rad[0] = value;
    } else if (strcmp(fixture->category, "joint_range") == 0) {
        input_ok = parse_input_double(fixture->input, "q_target_rad_0=", "",
                                      &command.q_target_rad[0]);
    } else if (strcmp(fixture->category, "joint_rate") == 0) {
        input_ok = parse_input_double(fixture->input, "q_target_rad_0=",
                                      "_over_10ms",
                                      &command.q_target_rad[0]);
    } else if (strcmp(fixture->category, "stale_state") == 0) {
        uint64_t age_ns = 0;

        input_ok =
            parse_input_u64(fixture->input, "state_age_ns=", &age_ns);
        state.t_mono_ns = (int64_t)(age_ns < now_ns ? now_ns - age_ns : 0);
    } else if (strcmp(fixture->category, "uncertainty") == 0) {
        double sigma;

        input_ok = parse_input_double(fixture->input, "sigma_q_rad_0=", "",
                                      &sigma) &&
                   sigma >= 0.0 && sigma <= (double)UINT32_MAX;
        state.sigma_q_rad[0] = (float)sigma;
    }
    writer_init(&writer, 46);
    simulator_reset(&simulator);
    observation.decision = safety_gate(&state, &command, now_ns);
    if (observation.decision.verdict == COURSE_VERDICT_APPROVE &&
        actuator_submit(&writer, &command)) {
        (void)actuator_pump(&writer, &state, now_ns, &simulator);
    }
    observation.actuator_writes = simulator.write_count;
    observation.behavior_ok = input_ok;
    observation.passed =
        observation.behavior_ok &&
        observation.decision.verdict == fixture->expected_verdict &&
        observation.decision.reason == fixture->expected_reason &&
        observation.actuator_writes == fixture->expected_writes;
    return observation;
}

static const FaultObservation *find_fault_observation(
    const FaultObservation *observations, size_t observation_count,
    const char *category) {
    for (size_t index = 0; index < observation_count; index++) {
        if (strcmp(observations[index].fixture->category, category) == 0) {
            return &observations[index];
        }
    }
    return NULL;
}

static bool observation_passed(const FaultObservation *observations,
                               size_t observation_count,
                               const char *category) {
    const FaultObservation *observation = find_fault_observation(
        observations, observation_count, category);

    return observation != NULL && observation->passed;
}

static bool bounded_queue_rejects(const ArmCommand *command) {
    ActuatorWriter writer;
    bool bounded = true;

    writer_init(&writer, 45);
    for (size_t index = 0; index < COURSE_WRITER_QUEUE_CAPACITY; index++) {
        ArmCommand queued = *command;

        queued.seq += index;
        bounded = bounded && actuator_submit(&writer, &queued);
    }
    return bounded && !actuator_submit(&writer, command) &&
           writer.count == COURSE_WRITER_QUEUE_CAPACITY;
}

static void write_manifest(TestContext *context, size_t fault_count,
                           size_t lock_count) {
    const char *image = getenv("COURSE_CONTAINER_IMAGE");
    const char *commit = getenv("COURSE_COMMIT");
    FILE *file = test_open_evidence(context, "manifest.json");

    if (file == NULL) {
        test_check(context, false, "evidence manifest is writable");
        return;
    }
    fprintf(file,
            "{\n  \"group\": \"%s\",\n"
            "  \"day\": 4,\n"
            "  \"input_tag\": \"runtime_poll_v1\",\n"
            "  \"output_tag\": \"safety_v1\",\n"
            "  \"seed\": %" PRIu64 ",\n"
            "  \"container_image\": \"%s\",\n"
            "  \"compiler\": \"%s\",\n"
            "  \"commit\": \"%s\",\n"
            "  \"fixture_faults\": %zu,\n"
            "  \"fixture_lock_orders\": %zu,\n"
            "  \"lock_probe_deadline_ns\": %" PRIu64 ",\n"
            "  \"command\": \"make verify-day4\"\n}\n",
            context->group, context->seed,
            image == NULL ? "NOT_RECORDED" : image, __VERSION__,
            commit == NULL ? "NOT_A_GIT_CHECKOUT" : commit, fault_count,
            lock_count,
            DAY4_LOCK_PROBE_DEADLINE_NS);
    fclose(file);
}

static void write_evidence(TestContext *context,
                           const FaultObservation *faults,
                           size_t fault_count,
                           const LockObservation *locks,
                           size_t lock_count,
                           const RaceArtifacts *race_artifacts) {
    FILE *file = test_open_evidence(context, "raw/fault_verdicts.csv");

    if (file != NULL) {
        fprintf(file,
                "fixture_id,category,input,expected_verdict,actual_verdict,"
                "expected_reason,actual_reason,expected_writes,actual_writes,"
                "status\n");
        for (size_t index = 0; index < fault_count; index++) {
            fprintf(file, "%s,%s,%s,%s,%s,%s,%s,%zu,%zu,%s\n",
                    faults[index].fixture->fixture_id,
                    faults[index].fixture->category,
                    faults[index].fixture->input,
                    verdict_name(faults[index].fixture->expected_verdict),
                    verdict_name(faults[index].decision.verdict),
                    reason_name(faults[index].fixture->expected_reason),
                    reason_name(faults[index].decision.reason),
                    faults[index].fixture->expected_writes,
                    faults[index].actuator_writes,
                    faults[index].passed ? "PASS" : "TODO_NOT_IMPLEMENTED");
        }
        fclose(file);
    }

    file = test_open_evidence(context, "raw/lock_trace.jsonl");
    if (file != NULL) {
        for (size_t index = 0; index < lock_count; index++) {
            fprintf(file,
                    "{\"fixture_id\":\"%s\",\"acquire_order\":\"%s\","
                    "\"expected_result\":\"%s\",\"terminated\":%s,"
                    "\"rejected_before_wait\":%s,\"timed_out\":%s,"
                    "\"elapsed_ns\":%" PRIu64 ",\"status\":\"%s\"}\n",
                    locks[index].fixture->fixture_id,
                    locks[index].fixture->acquire_order,
                    locks[index].fixture->expected_result,
                    locks[index].terminated ? "true" : "false",
                    locks[index].rejected_before_wait ? "true" : "false",
                    locks[index].timed_out ? "true" : "false",
                    locks[index].elapsed_ns,
                    locks[index].passed ? "PASS" : "FAIL");
        }
        fclose(file);
    }

    file = test_open_evidence(context, "raw/audit.log");
    if (file != NULL) {
        fprintf(file,
                "trace_id,command_seq,state_seq,verdict,reason,writer_id,"
                "t_commit_ns\n");
        if (race_artifacts->available) {
            for (size_t index = 0;
                 index < race_artifacts->writer.audit_count; index++) {
                const AuditRow *row = &race_artifacts->writer.audit[index];

                fprintf(file,
                        "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%s,%s,%u,"
                        "%" PRIu64 "\n",
                        row->trace_id, row->command_seq, row->state_seq,
                        verdict_name(row->verdict), reason_name(row->reason),
                        row->writer_id, row->t_commit_ns);
            }
        }
        fclose(file);
    }
    write_manifest(context, fault_count, lock_count);
}

int verify_day4(TestContext *context) {
    Day4FaultFixture fault_fixtures[DAY4_MAX_FAULTS];
    Day4LockFixture lock_fixtures[DAY4_MAX_LOCK_ORDERS];
    FaultObservation fault_observations[DAY4_MAX_FAULTS];
    LockObservation lock_observations[DAY4_MAX_LOCK_ORDERS];
    size_t fault_count = 0;
    size_t lock_count = 0;
    const bool faults_loaded = load_fault_fixtures(
        fault_fixtures, sizeof(fault_fixtures) / sizeof(fault_fixtures[0]),
        &fault_count);
    const bool locks_loaded = load_lock_fixtures(
        lock_fixtures, sizeof(lock_fixtures) / sizeof(lock_fixtures[0]),
        &lock_count);
    const bool fixture_contract =
        faults_loaded && locks_loaded &&
        fault_fixture_contract_ok(fault_fixtures, fault_count) &&
        lock_fixture_contract_ok(lock_fixtures, lock_count);
    const uint64_t now_ns = monotonic_ns();
    RaceArtifacts race_artifacts;
    const Day4LockFixture *reversed_fixture;
    const LockObservation *reversed_observation = NULL;
    ArmCommand command = valid_command(now_ns);
    bool lock_probes_ok = true;
    bool all_faults_ok = true;
    bool queue_bounded;

    memset(&race_artifacts, 0, sizeof(race_artifacts));
    test_check(context, fixture_contract,
               "Day4 fault and lock-order fixtures match the contract");
    if (!fixture_contract) {
        return test_finish(context);
    }

    for (size_t index = 0; index < lock_count; index++) {
        lock_observations[index] = run_lock_probe(&lock_fixtures[index]);
        lock_probes_ok = lock_probes_ok && lock_observations[index].passed;
    }
    reversed_fixture = find_lock_fixture(lock_fixtures, lock_count,
                                         "reject_before_wait");
    for (size_t index = 0; index < lock_count; index++) {
        if (lock_observations[index].fixture == reversed_fixture) {
            reversed_observation = &lock_observations[index];
        }
    }
    test_check(context, lock_probes_ok && reversed_observation != NULL,
               "Day4 lock-order probes terminate within the fixed deadline");

    for (size_t index = 0; index < fault_count; index++) {
        if (strcmp(fault_fixtures[index].category,
                   "check_write_race") == 0) {
            fault_observations[index] = run_race_fault(
                &fault_fixtures[index], now_ns, &race_artifacts);
        } else if (strcmp(fault_fixtures[index].category,
                          "reversed_lock_order") == 0) {
            fault_observations[index] = run_lock_fault(
                &fault_fixtures[index], reversed_observation);
        } else {
            fault_observations[index] =
                run_safety_fault(&fault_fixtures[index], now_ns);
        }
        all_faults_ok = all_faults_ok && fault_observations[index].passed;
    }
    queue_bounded = bounded_queue_rejects(&command);

    if (test_group_enabled(context, "G1")) {
        ArmState previous_state = valid_state(now_ns);
        ArmState valid_next = previous_state;
        ArmState uncertain_next;

        previous_state.seq--;
        previous_state.t_mono_ns -= COURSE_FIXED_STEP_NS;
        uncertain_next = valid_next;
        uncertain_next.sigma_q_rad[0] =
            (float)(COURSE_MAX_SIGMA_Q_RAD + 0.01);
        check_or_todo(context,
                      state_valid(&valid_next, &previous_state) &&
                          !state_valid(&uncertain_next, &previous_state),
                      "G1 owned state API rejects excessive uncertainty before safety handoff");
        check_or_todo(context,
                      observation_passed(fault_observations, fault_count,
                                         "stale_state") &&
                          observation_passed(fault_observations, fault_count,
                                             "uncertainty"),
                      "G1 stale and uncertain state never actuate");
    }
    if (test_group_enabled(context, "G2")) {
        FreshnessGate owned_gate;
        TraceRow owned_trace;
        ArmCommand owned_accepted;
        ArmCommand owned_command = valid_command(now_ns);
        uint8_t owned_frame[COURSE_FRAME_V1_LEN];
        size_t owned_length = 0;
        bool owned_encoded;
        Reason owned_reason;

        memset(&owned_gate, 0, sizeof(owned_gate));
        memset(&owned_trace, 0, sizeof(owned_trace));
        memset(&owned_accepted, 0, sizeof(owned_accepted));
        memset(owned_frame, 0, sizeof(owned_frame));
        owned_encoded = frame_encode(&owned_command, owned_frame,
                                     sizeof(owned_frame), &owned_length);
        if (owned_encoded && owned_length == COURSE_FRAME_V1_LEN) {
            owned_frame[31] ^= UINT8_C(0x04);
        }
        owned_reason = transport_on_message(
            &owned_gate, owned_frame, owned_length, now_ns, &owned_trace,
            &owned_accepted);
        check_or_todo(context,
                      owned_encoded &&
                          owned_reason != COURSE_REASON_STUDENT_TODO &&
                          owned_reason == COURSE_REASON_BAD_FRAME &&
                          owned_trace.verdict == COURSE_VERDICT_REJECT &&
                          owned_trace.reason == COURSE_REASON_BAD_FRAME &&
                          !owned_gate.has_latest,
                      "G2 owned transport API preserves BAD_FRAME without latest-value mutation");
        check_or_todo(context,
                      observation_passed(fault_observations, fault_count,
                                         "nan_command") &&
                          observation_passed(fault_observations, fault_count,
                                             "joint_range") &&
                          observation_passed(fault_observations, fault_count,
                                             "joint_rate"),
                      "G2 malformed command fixture values remain fail-closed");
    }
    if (test_group_enabled(context, "G3")) {
        int age_pipe[2] = {-1, -1};
        ClientState aged_client;
        RuntimeStats age_stats;
        const uint8_t ready_byte = UINT8_C(0x5a);
        bool age_policy_ok = false;

        memset(&aged_client, 0, sizeof(aged_client));
        memset(&age_stats, 0, sizeof(age_stats));
        if (pipe(age_pipe) == 0) {
            aged_client.fd = age_pipe[0];
            aged_client.client_id = 43;
            aged_client.class_id = COURSE_CLIENT_SLOW;
            aged_client.queued_bytes = 1;
            aged_client.oldest_enqueue_ns =
                now_ns - COURSE_MAX_QUEUE_AGE_NS - UINT64_C(1);
            if (set_nonblocking(age_pipe[0]) == 0 &&
                write(age_pipe[1], &ready_byte, sizeof(ready_byte)) ==
                    (ssize_t)sizeof(ready_byte)) {
                const int ready = poll_service_once(&aged_client, 1, 20,
                                                    &age_stats);

                age_policy_ok =
                    ready > 0 && aged_client.dropped_total > 0 &&
                    aged_client.dispatched_total == 0 &&
                    aged_client.work_this_turn <= COURSE_WORK_BUDGET &&
                    age_stats.max_queue_age_ns > COURSE_MAX_QUEUE_AGE_NS;
            }
            close(age_pipe[0]);
            close(age_pipe[1]);
        }
        check_or_todo(context, age_policy_ok,
                      "G3 owned runtime API expires over-age queued work before service");
        check_or_todo(context,
                      observation_passed(fault_observations, fault_count,
                                         "check_write_race") &&
                          observation_passed(fault_observations, fault_count,
                                             "reversed_lock_order"),
                      "G3 immutable queue closes the race and reversed lock "
                      "order is rejected before waiting");
    }
    if (test_group_enabled(context, "G4")) {
        check_or_todo(context, all_faults_ok,
                      "G4 all fixed faults have stable verdicts and reasons");
        check_or_todo(context, queue_bounded,
                      "G4 owned writer API rejects queue overflow without bypass");
    }
    if (test_group_enabled(context, "G5")) {
        FreshnessGate owned_freshness;
        ActuatorWriter owned_writer;
        ArmState owned_state = valid_state(now_ns);
        TraceRow owned_trace;
        uint8_t owned_frame[COURSE_FRAME_V1_LEN];
        size_t owned_length = 0;
        const bool owned_encoded = frame_encode(
            &command, owned_frame, sizeof(owned_frame), &owned_length);
        Verdict owned_verdict;

        memset(&owned_freshness, 0, sizeof(owned_freshness));
        memset(&owned_trace, 0, sizeof(owned_trace));
        writer_init(&owned_writer, 45);
        owned_verdict = gatekeeper_process(
            &owned_freshness, &owned_writer, &owned_state, owned_frame,
            owned_length, now_ns, &owned_trace);
        check_or_todo(context,
                      owned_encoded &&
                          owned_trace.reason != COURSE_REASON_STUDENT_TODO &&
                          owned_verdict == COURSE_VERDICT_APPROVE &&
                          owned_trace.verdict == COURSE_VERDICT_APPROVE &&
                          owned_trace.reason == COURSE_REASON_NONE &&
                          owned_writer.count == 1,
                      "G5 owned gatekeeper routes one safe frame into the sole-writer queue");
        check_or_todo(context,
                      observation_passed(fault_observations, fault_count,
                                         "check_write_race") &&
                          race_artifacts.available &&
                          race_artifacts.writer.audit_count ==
                              race_artifacts.simulator.write_count &&
                          race_artifacts.writer.unsafe_write_count == 0,
                      "G5 integrated safety path has one audit row per actuation");
    }

    write_evidence(context, fault_observations, fault_count,
                   lock_observations, lock_count, &race_artifacts);
    return test_finish(context);
}
