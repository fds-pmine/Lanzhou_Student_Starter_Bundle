#include "course.h"

#include <string.h>

ControllerAction controller_sanitize(
    const ControllerResult *result,
    uint64_t active_generation,
    uint64_t now_ns,
    const ArmState *state,
    ArmCommand *command_out,
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

Verdict gatekeeper_process(
    FreshnessGate *freshness,
    ActuatorWriter *writer,
    const ArmState *state,
    const uint8_t *frame,
    size_t frame_length,
    uint64_t now_ns,
    TraceRow *trace) {
    ArmCommand command;
    uint64_t last_seq;
    RxVerdict rx;
    Reason reason;

    if (trace != NULL) {
        memset(trace, 0, sizeof(*trace));
        trace->t_gate_ns = now_ns;
        trace->verdict = COURSE_VERDICT_REJECT;
        trace->reason = COURSE_REASON_STUDENT_TODO;
    }

    /*
     * Day 1 and Day 3:
     * Reject missing and malformed frames before decoding, changing
     * freshness state, or touching the writer queue.
     */
    if (frame == NULL || frame_length != COURSE_FRAME_V1_LEN) {
        if (trace != NULL) {
            trace->t_ack_ns = now_ns;
            trace->verdict = COURSE_VERDICT_REJECT;
            trace->reason = COURSE_REASON_BAD_FRAME;
        }

        return COURSE_VERDICT_REJECT;
    }

    memset(&command, 0, sizeof(command));

    last_seq =
        freshness != NULL && freshness->has_last
            ? freshness->last_seq
            : 0;

    rx = frame_decode(
        frame,
        frame_length,
        last_seq,
        &command);

    if (rx == COURSE_RX_ACCEPT) {
        reason =
            freshness != NULL
                ? freshness_accept(freshness, &command, now_ns)
                : COURSE_REASON_INTERNAL;
    } else if (rx == COURSE_RX_NACK_SEQUENCE) {
        reason = COURSE_REASON_NOT_NEW;
    } else {
        reason = COURSE_REASON_BAD_FRAME;
    }

    if (trace != NULL) {
        if (rx == COURSE_RX_ACCEPT) {
            trace->trace_id = command.trace_id;
            trace->seq = command.seq;
            trace->t_pub_ns = command.t_source_ns;
            trace->t_rx_ns = now_ns;
        }

        trace->t_ack_ns = now_ns;
        trace->verdict =
            reason == COURSE_REASON_NONE
                ? COURSE_VERDICT_APPROVE
                : COURSE_VERDICT_REJECT;
        trace->reason = reason;
    }

    /*
     * Day 3:
     * The writer is deliberately untouched. Repeated malformed inputs
     * therefore produce the same REJECT/BAD_FRAME result and zero writes.
     */

    /* DAY4_G5_TODO_A: route integration through the sole safe writer. */
    /* DAY5_G5_TODO_B: connect decode, freshness, safety, queue, and trace. */
    (void)writer;
    (void)state;

    return reason == COURSE_REASON_NONE
               ? COURSE_VERDICT_APPROVE
               : COURSE_VERDICT_REJECT;
}

bool trace_replay_matches(
    const TraceRow *recorded,
    const TraceRow *replayed) {
    /* DAY5_G5_TODO: compare the frozen replay fields exactly. */
    (void)recorded;
    (void)replayed;

    return false;
}