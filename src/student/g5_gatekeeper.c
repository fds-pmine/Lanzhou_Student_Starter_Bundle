#include "course.h"

#include <math.h>
#include <string.h>

static ControllerAction controller_discard(Reason reason,
                                           Reason *reason_out) {
    if (reason_out != NULL) {
        *reason_out = reason;
    }
    return COURSE_CONTROLLER_DISCARD;
}

ControllerAction controller_sanitize(
    const ControllerResult *result, uint64_t active_generation,
    uint64_t now_ns, const ArmState *state, ArmCommand *command_out,
    Reason *reason_out) {
    size_t joint;

    if (command_out != NULL) {
        memset(command_out, 0, sizeof(*command_out));
    }
    if (reason_out != NULL) {
        *reason_out = COURSE_REASON_INTERNAL;
    }

    /*
     * The Day 5 policy uses fail-closed discard for every controller fault.
     * A command is copied out only after every frozen check has passed.
     */
    if (result == NULL || state == NULL || command_out == NULL) {
        return controller_discard(COURSE_REASON_INTERNAL, reason_out);
    }

    if (result->generation != active_generation ||
        result->command.generation != active_generation) {
        return controller_discard(COURSE_REASON_CANCELLED, reason_out);
    }

    if (result->timed_out || result->produced_ns > now_ns ||
        now_ns - result->produced_ns > COURSE_CONTROLLER_TIMEOUT_NS) {
        return controller_discard(COURSE_REASON_CONTROLLER_TIMEOUT,
                                  reason_out);
    }

    if (!isfinite(result->confidence)) {
        return controller_discard(COURSE_REASON_CONTROLLER_INVALID,
                                  reason_out);
    }

    for (joint = 0; joint < COURSE_ARM_DOF; ++joint) {
        const double target = result->command.q_target_rad[joint];

        if (!isfinite(target) || target < COURSE_JOINT_MIN_RAD ||
            target > COURSE_JOINT_MAX_RAD) {
            return controller_discard(COURSE_REASON_CONTROLLER_INVALID,
                                      reason_out);
        }
    }

    if (result->confidence < COURSE_MIN_CONTROLLER_CONFIDENCE) {
        return controller_discard(COURSE_REASON_LOW_CONFIDENCE, reason_out);
    }

    *command_out = result->command;
    command_out->generation = active_generation;
    if (reason_out != NULL) {
        *reason_out = COURSE_REASON_NONE;
    }
    return COURSE_CONTROLLER_USE;
}

Verdict gatekeeper_process(
    FreshnessGate *freshness, ActuatorWriter *writer,
    const ArmState *state, const uint8_t *frame, size_t frame_length,
    uint64_t now_ns, TraceRow *trace) {
    ArmCommand command;
    SafetyDecision safety;
    uint64_t last_seq;
    RxVerdict rx;
    Reason reason;

    if (trace != NULL) {
        memset(trace, 0, sizeof(*trace));
        trace->t_rx_ns = now_ns;
        trace->t_gate_ns = now_ns;
        trace->t_ack_ns = now_ns;
        trace->verdict = COURSE_VERDICT_REJECT;
        trace->reason = COURSE_REASON_INTERNAL;
    }

    /* Reject before decoding, changing freshness state, or touching writer. */
    if (frame == NULL || frame_length != COURSE_FRAME_V1_LEN) {
        if (trace != NULL) {
            trace->reason = COURSE_REASON_BAD_FRAME;
        }
        return COURSE_VERDICT_REJECT;
    }

    if (freshness == NULL || writer == NULL || state == NULL) {
        return COURSE_VERDICT_REJECT;
    }

    memset(&command, 0, sizeof(command));
    last_seq = freshness->has_last ? freshness->last_seq : 0;
    rx = frame_decode(frame, frame_length, last_seq, &command);

    if (rx != COURSE_RX_ACCEPT) {
        reason = rx == COURSE_RX_NACK_SEQUENCE
                     ? COURSE_REASON_NOT_NEW
                     : COURSE_REASON_BAD_FRAME;
        if (trace != NULL) {
            trace->reason = reason;
        }
        return COURSE_VERDICT_REJECT;
    }

    if (trace != NULL) {
        trace->trace_id = command.trace_id;
        trace->seq = command.seq;
        trace->t_pub_ns = command.t_source_ns;
    }

    reason = freshness_accept(freshness, &command, now_ns);
    if (reason != COURSE_REASON_NONE) {
        if (trace != NULL) {
            trace->reason = reason;
        }
        return COURSE_VERDICT_REJECT;
    }

    safety = safety_gate(state, &command, now_ns);
    if (safety.verdict != COURSE_VERDICT_APPROVE) {
        if (trace != NULL) {
            trace->reason = safety.reason;
        }
        return COURSE_VERDICT_REJECT;
    }

    if (!actuator_submit(writer, &command)) {
        if (trace != NULL) {
            trace->reason = COURSE_REASON_INTERNAL;
        }
        return COURSE_VERDICT_REJECT;
    }

    if (trace != NULL) {
        trace->verdict = COURSE_VERDICT_APPROVE;
        trace->reason = COURSE_REASON_NONE;
    }
    return COURSE_VERDICT_APPROVE;
}

bool trace_replay_matches(const TraceRow *recorded,
                          const TraceRow *replayed) {
    if (recorded == NULL || replayed == NULL) {
        return false;
    }

    return recorded->trace_id == replayed->trace_id &&
           recorded->seq == replayed->seq &&
           recorded->t_pub_ns == replayed->t_pub_ns &&
           recorded->t_rx_ns == replayed->t_rx_ns &&
           recorded->t_gate_ns == replayed->t_gate_ns &&
           recorded->t_ack_ns == replayed->t_ack_ns &&
           recorded->verdict == replayed->verdict &&
           recorded->reason == replayed->reason;
}