#include "course.h"

#include <string.h>

ControllerAction controller_sanitize(
    const ControllerResult *result, uint64_t active_generation,
    uint64_t now_ns, const ArmState *state, ArmCommand *command_out,
    Reason *reason_out) {
    if (command_out != NULL) {
        memset(command_out, 0, sizeof(*command_out));
    }
    if (reason_out != NULL) {
        *reason_out = COURSE_REASON_STUDENT_TODO;
    }

    /* DAY5_G5_TODO: contain timeout/NaN/range/confidence faults with the
       frozen fallback or discard policy. */
    (void)result;
    (void)active_generation;
    (void)now_ns;
    (void)state;
    return COURSE_CONTROLLER_DISCARD;
}

Verdict gatekeeper_process(FreshnessGate *freshness,
                                  ActuatorWriter *writer,
                                  const ArmState *state,
                                  const uint8_t *frame, size_t frame_length,
                                  uint64_t now_ns, TraceRow *trace) {
    if (trace != NULL) {
        memset(trace, 0, sizeof(*trace));
        trace->t_gate_ns = now_ns;
        trace->verdict = COURSE_VERDICT_REJECT;
        trace->reason = COURSE_REASON_STUDENT_TODO;
    }

    /* DAY1_G5_TODO_A: reject a missing frame with an explicit BAD_FRAME trace
       before touching the writer queue. */
    if (frame == NULL) {
        if (trace != NULL) {
            trace->reason = COURSE_REASON_BAD_FRAME;
        }
        return COURSE_VERDICT_REJECT;
    }

    /* DAY2_G5_TODO_A: connect frame, freshness, and four-stamp trace. */
    /* DAY3_G5_TODO_A: keep repeated malformed-frame integration calls
       deterministic and non-writing under runtime load. */
    /* DAY4_G5_TODO_A: route integration through the sole safe writer. */
    /* DAY5_G5_TODO_B: connect decode, freshness, safety, queue, and trace. */
    (void)freshness;
    (void)writer;
    (void)state;
    (void)frame;
    (void)frame_length;
    return COURSE_VERDICT_REJECT;
}

bool trace_replay_matches(const TraceRow *recorded,
                              const TraceRow *replayed) {
    /* DAY5_G5_TODO: compare the frozen replay fields exactly. */
    (void)recorded;
    (void)replayed;
    return false;
}
