#include "course.h"
#include <math.h>
#include <string.h>

SafetyDecision safety_gate(const ArmState *state,
                                  const ArmCommand *command,
                                  uint64_t now_ns) {
    
    /* DAY1_G4_TODO_A: freeze fail-closed verdict and reason semantics. */
    SafetyDecision fail_closed = {
        .verdict = COURSE_VERDICT_REJECT,
        .reason = COURSE_REASON_STUDENT_TODO,
    };
    /* DAY2_G4_TODO_A: keep an absent state or command fail-closed with an
       explicit internal-error reason. */
    if (state == NULL || command == NULL) {
        fail_closed.reason = COURSE_REASON_INTERNAL;
        return fail_closed;
    }
    /* DAY3_G4_TODO_A: make repeated fail-closed dispatch decisions stable so
       the isolated path can be measured under load. */
    
    /* DAY4_G4_TODO: implement the pure finite/range/rate/state safety checks. */

    /* 1) 有限性检查 */
    for (int i = 0; i < COURSE_ARM_DOF; i++) {
        if (!isfinite(state->q_rad[i]) ||
            !isfinite(state->dq_rad_s[i]) ||
            !isfinite(command->q_target_rad[i])) {
            fail_closed.reason = COURSE_REASON_NONFINITE;
            return fail_closed;
        }
    }

    /* 2) 状态过期检查（新增） */
    if (now_ns < (uint64_t)state->t_mono_ns) {
        fail_closed.reason = COURSE_REASON_CLOCK_ERROR;
        return fail_closed;
    }
    uint64_t state_age_ns = now_ns - (uint64_t)state->t_mono_ns;
    if (state_age_ns > COURSE_MAX_STATE_AGE_NS) {
        fail_closed.reason = COURSE_REASON_STALE_STATE;
        return fail_closed;
    }

    /* 3) 关节范围检查 */
    for (int i = 0; i < COURSE_ARM_DOF; i++) {
        if (command->q_target_rad[i] < COURSE_JOINT_MIN_RAD ||
            command->q_target_rad[i] > COURSE_JOINT_MAX_RAD) {
            fail_closed.reason = COURSE_REASON_JOINT_RANGE;
            return fail_closed;
        }
    }

    /* 4) 速率限制检查 */
    double dt_sec = (double)state_age_ns / 1e9;  // 复用已计算的有效年龄
    if (dt_sec <= 0.0) {
        fail_closed.reason = COURSE_REASON_CLOCK_ERROR;
        return fail_closed;
    }
    for (int i = 0; i < COURSE_ARM_DOF; i++) {
        double diff = command->q_target_rad[i] - state->q_rad[i];
        double rate = diff / dt_sec;
        if (fabs(rate) > COURSE_MAX_JOINT_RATE_RAD_S) {
            fail_closed.reason = COURSE_REASON_RATE_LIMIT;
            return fail_closed;
        }
    }

    /* 5) 不确定性检查 */
    for (int i = 0; i < COURSE_ARM_DOF; i++) {
        if (state->sigma_q_rad[i] > COURSE_MAX_SIGMA_Q_RAD) {
            fail_closed.reason = COURSE_REASON_UNCERTAINTY;
            return fail_closed;
        }
    }

    SafetyDecision approved = {
        .verdict = COURSE_VERDICT_APPROVE,
        .reason = COURSE_REASON_NONE,
    };
    return approved;
}

void writer_init(ActuatorWriter *writer, uint32_t writer_id) {
    if (writer == NULL) {
        return;
    }
    memset(writer, 0, sizeof(*writer));
    writer->writer_id = writer_id;
}

bool actuator_submit(ActuatorWriter *writer,
                         const ArmCommand *command) {
    /* DAY4_G4_TODO_B: move an immutable command into the bounded queue. */
    /* DAY5_G4_TODO_A: reject an unowned writer_id == 0 before queue admission
       while preserving admission for an initialized sole writer. */
    if (writer == NULL || command == NULL) {
        return false;
    }
    if (writer->writer_id == 0) {
        return false;
    }
    if (writer->count >= COURSE_WRITER_QUEUE_CAPACITY) {
        return false;
    }
    writer->queue[writer->tail] = *command;
    writer->tail = (writer->tail + 1) % COURSE_WRITER_QUEUE_CAPACITY;
    writer->count++;
    return true;
}

bool actuator_pump(ActuatorWriter *writer, const ArmState *state,
                       uint64_t now_ns, Simulator *simulator) {
    /* DAY4_G4_TODO_C: gate, append audit, increment writer count, then call
       simulator_commit_from_writer; no other module may commit. */
    if (writer == NULL || state == NULL || simulator == NULL) {
        return false;
    }
    if (writer->count == 0) {
        return false;
    }

    ArmCommand cmd = writer->queue[writer->head];
    writer->head = (writer->head + 1) % COURSE_WRITER_QUEUE_CAPACITY;
    writer->count--;

    SafetyDecision decision = safety_gate(state, &cmd, now_ns);

    if (writer->audit_count < COURSE_AUDIT_CAPACITY) {
        AuditRow *row = &writer->audit[writer->audit_count];
        row->trace_id = cmd.trace_id;
        row->command_seq = cmd.seq;
        row->state_seq = state->seq;
        row->verdict = decision.verdict;
        row->reason = decision.reason;
        row->writer_id = writer->writer_id;
        row->t_commit_ns = now_ns;
        writer->audit_count++;
    }

    if (decision.verdict == COURSE_VERDICT_APPROVE) {
        writer->write_count++;
        simulator_commit_from_writer(simulator, writer, &cmd, decision);
    } else {
        writer->unsafe_write_count++;
    }

    return true;
}