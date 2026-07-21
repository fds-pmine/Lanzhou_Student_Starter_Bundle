#include "course.h"

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
    (void)state;
    (void)command;
    (void)now_ns;
    return fail_closed;
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
    (void)writer;
    (void)command;
    return false;
}

bool actuator_pump(ActuatorWriter *writer, const ArmState *state,
                       uint64_t now_ns, Simulator *simulator) {
    /* DAY4_G4_TODO_C: gate, append audit, increment writer count, then call
       simulator_commit_from_writer; no other module may commit. */
    (void)writer;
    (void)state;
    (void)now_ns;
    (void)simulator;
    return false;
}
