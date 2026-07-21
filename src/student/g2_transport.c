#include "course.h"

#include <string.h>

/*
 * FrameV1 frozen wire layout:
 *
 * byte 0 : sync 0 = 0xA5
 * byte 1 : sync 1 = 0x5A
 * byte 2 : frame version
 * byte 3 : frame type
 * byte 4..5 : payload length, little-endian
 * byte 6..13 : sequence number, little-endian
 * byte 14..21 : source timestamp, little-endian
 * byte 22..29 : trace ID, little-endian
 * byte 30..53 : q_target_rad[3], IEEE-754 double, little-endian
 * byte 54..55 : CRC-16/CCITT-FALSE, little-endian
 *
 * CRC coverage:
 * byte 2 through byte 53, namely version through payload.
 */

enum {
    FRAME_OFFSET_SYNC_0 = 0,
    FRAME_OFFSET_SYNC_1 = 1,
    FRAME_OFFSET_VERSION = 2,
    FRAME_OFFSET_TYPE = 3,
    FRAME_OFFSET_PAYLOAD_LENGTH = 4,
    FRAME_OFFSET_SEQUENCE = 6,
    FRAME_OFFSET_SOURCE_TIME = 14,
    FRAME_OFFSET_TRACE_ID = 22,
    FRAME_OFFSET_JOINT_TARGET = 30
};


/* Write a 16-bit integer in little-endian order. */
static void put_u16_le(uint8_t *destination, uint16_t value) {
    destination[0] = (uint8_t)(value & UINT16_C(0x00FF));
    destination[1] =
        (uint8_t)((value >> 8) & UINT16_C(0x00FF));
}


/* Read a 16-bit little-endian integer. */
static uint16_t get_u16_le(const uint8_t *source) {
    return (uint16_t)(
        (uint16_t)source[0] |
        ((uint16_t)source[1] << 8)
    );
}


/* Write a 64-bit integer in little-endian order. */
static void put_u64_le(uint8_t *destination, uint64_t value) {
    size_t index;

    for (index = 0; index < 8; ++index) {
        destination[index] =
            (uint8_t)((value >> (8U * index)) & UINT64_C(0xFF));
    }
}


/* Read a 64-bit little-endian integer. */
static uint64_t get_u64_le(const uint8_t *source) {
    uint64_t value = UINT64_C(0);
    size_t index;

    for (index = 0; index < 8; ++index) {
        value |= ((uint64_t)source[index]) << (8U * index);
    }

    return value;
}


/*
 * Encode a double by preserving its IEEE-754 binary representation,
 * then writing the 64-bit pattern in little-endian order.
 */
static void put_f64_le(uint8_t *destination, double value) {
    uint64_t bits = UINT64_C(0);

    memcpy(&bits, &value, sizeof(bits));
    put_u64_le(destination, bits);
}


/* Read a little-endian IEEE-754 double. */
static double get_f64_le(const uint8_t *source) {
    uint64_t bits = get_u64_le(source);
    double value = 0.0;

    memcpy(&value, &bits, sizeof(value));
    return value;
}


/* Return the parser to synchronization-searching mode. */
static void parser_reset(FrameParser *parser) {
    parser->used = 0;
    parser->syncing = true;
}


/*
 * After a rejected candidate frame, search its buffered bytes for a later
 * synchronization prefix. This permits recovery when a truncated frame is
 * immediately followed by a valid frame.
 */
static void parser_resync(FrameParser *parser) {
    size_t index;

    /*
     * Start at index 1 because index 0 is the rejected frame's original
     * synchronization prefix.
     */
    for (index = 1; index + 1 < parser->used; ++index) {
        if (parser->bytes[index] == COURSE_FRAME_SYNC_0 &&
            parser->bytes[index + 1] == COURSE_FRAME_SYNC_1) {
            size_t remaining = parser->used - index;

            memmove(
                parser->bytes,
                parser->bytes + index,
                remaining
            );

            parser->used = remaining;
            parser->syncing = false;
            return;
        }
    }

    /*
     * Preserve a final 0xA5 because it may be the first byte of the next
     * synchronization word.
     */
    if (parser->used > 0 &&
        parser->bytes[parser->used - 1] == COURSE_FRAME_SYNC_0) {
        parser->bytes[0] = COURSE_FRAME_SYNC_0;
        parser->used = 1;
        parser->syncing = true;
        return;
    }

    parser_reset(parser);
}


uint16_t crc16_ccitt_false(const uint8_t *bytes, size_t length) {
    uint16_t crc = UINT16_C(0xFFFF);
    size_t index;

    /*
     * CRC-16/CCITT-FALSE:
     *
     * polynomial = 0x1021
     * initial = 0xFFFF
     * refin = false
     * refout = false
     * xorout = 0x0000
     */
    if (bytes == NULL) {
        /*
         * CRC of an empty input is the initial value.
         * A NULL pointer with a nonzero length is invalid.
         */
        return length == 0
                   ? UINT16_C(0xFFFF)
                   : UINT16_C(0);
    }

    for (index = 0; index < length; ++index) {
        unsigned int bit;

        crc ^= (uint16_t)((uint16_t)bytes[index] << 8);

        for (bit = 0; bit < 8; ++bit) {
            if ((crc & UINT16_C(0x8000)) != 0) {
                crc = (uint16_t)(
                    (uint16_t)(crc << 1) ^ UINT16_C(0x1021)
                );
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}


bool frame_encode(const ArmCommand *command, uint8_t *out,
                  size_t capacity, size_t *written) {
    uint16_t crc;
    size_t joint;

    if (written != NULL) {
        *written = 0;
    }

    if (command == NULL ||
        out == NULL ||
        capacity < COURSE_FRAME_V1_LEN) {
        return false;
    }

    /*
     * Clear all bytes before encoding so no uninitialized or stale bytes
     * enter the frozen wire representation.
     */
    memset(out, 0, COURSE_FRAME_V1_LEN);

    /* Frame header. */
    out[FRAME_OFFSET_SYNC_0] = COURSE_FRAME_SYNC_0;
    out[FRAME_OFFSET_SYNC_1] = COURSE_FRAME_SYNC_1;
    out[FRAME_OFFSET_VERSION] = COURSE_FRAME_VERSION;

    if (command->emergency) {
        out[FRAME_OFFSET_TYPE] = COURSE_FRAME_TYPE_ESTOP;
    } else {
        out[FRAME_OFFSET_TYPE] = COURSE_FRAME_TYPE_COMMAND;
    }

    put_u16_le(
        out + FRAME_OFFSET_PAYLOAD_LENGTH,
        (uint16_t)COURSE_COMMAND_PAYLOAD_LEN
    );

    /* Fixed command metadata. */
    put_u64_le(
        out + FRAME_OFFSET_SEQUENCE,
        command->seq
    );

    put_u64_le(
        out + FRAME_OFFSET_SOURCE_TIME,
        command->t_source_ns
    );

    put_u64_le(
        out + FRAME_OFFSET_TRACE_ID,
        command->trace_id
    );

    /*
     * Frozen command payload:
     *
     * 3 doubles × 8 bytes = 24 bytes.
     */
    for (joint = 0; joint < COURSE_ARM_DOF; ++joint) {
        put_f64_le(
            out + FRAME_OFFSET_JOINT_TARGET +
                joint * sizeof(double),
            command->q_target_rad[joint]
        );
    }

    /*
     * CRC covers byte 2 through byte 53:
     *
     * version, type, payload length, command metadata and payload.
     *
     * The synchronization bytes at offsets 0 and 1 are not included.
     * The CRC storage bytes at offsets 54 and 55 are not included.
     */
    crc = crc16_ccitt_false(
        out + FRAME_OFFSET_VERSION,
        COURSE_FRAME_CRC_OFFSET - FRAME_OFFSET_VERSION
    );

    put_u16_le(
        out + COURSE_FRAME_CRC_OFFSET,
        crc
    );

    if (written != NULL) {
        *written = COURSE_FRAME_V1_LEN;
    }

    return true;
}


RxVerdict frame_decode(const uint8_t *bytes, size_t length,
                       uint64_t last_seq, ArmCommand *out) {
    uint8_t frame_type;
    uint16_t payload_length;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint64_t sequence;
    size_t joint;

    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }

    if (bytes == NULL || out == NULL) {
        return COURSE_RX_REJECT;
    }

    /*
     * Required validation order:
     *
     * length
     * -> version/type
     * -> CRC
     * -> sequence
     * -> payload
     */

    /* 1. Verify the complete frozen wire length. */
    if (length != COURSE_FRAME_V1_LEN) {
        return COURSE_RX_NACK_LENGTH;
    }

    /*
     * Verify the synchronization word.
     * There is no separate NACK_SYNC verdict, so generic rejection is used.
     */
    if (bytes[FRAME_OFFSET_SYNC_0] != COURSE_FRAME_SYNC_0 ||
        bytes[FRAME_OFFSET_SYNC_1] != COURSE_FRAME_SYNC_1) {
        return COURSE_RX_REJECT;
    }

    /* Verify the frozen 24-byte payload declaration. */
    payload_length = get_u16_le(
        bytes + FRAME_OFFSET_PAYLOAD_LENGTH
    );

    if (payload_length != COURSE_COMMAND_PAYLOAD_LEN) {
        return COURSE_RX_NACK_LENGTH;
    }

    /* 2. Verify the frame version. */
    if (bytes[FRAME_OFFSET_VERSION] != COURSE_FRAME_VERSION) {
        return COURSE_RX_NACK_VERSION;
    }

    /* Verify the frame type. */
    frame_type = bytes[FRAME_OFFSET_TYPE];

    if (frame_type != COURSE_FRAME_TYPE_COMMAND &&
        frame_type != COURSE_FRAME_TYPE_ESTOP) {
        return COURSE_RX_REJECT;
    }

    /*
     * 3. Verify CRC before reading sequence metadata or payload values.
     *
     * CRC covers byte 2 through byte 53.
     */
    received_crc = get_u16_le(
        bytes + COURSE_FRAME_CRC_OFFSET
    );

    calculated_crc = crc16_ccitt_false(
        bytes + FRAME_OFFSET_VERSION,
        COURSE_FRAME_CRC_OFFSET - FRAME_OFFSET_VERSION
    );

    if (received_crc != calculated_crc) {
        return COURSE_RX_NACK_CRC;
    }

    /* 4. Reject duplicate or out-of-order sequence numbers. */
    sequence = get_u64_le(
        bytes + FRAME_OFFSET_SEQUENCE
    );

    if (sequence <= last_seq) {
        return COURSE_RX_NACK_SEQUENCE;
    }

    /*
     * 5. Populate the output only after all integrity and ordering checks
     * have passed.
     */
    out->seq = sequence;

    out->t_source_ns = get_u64_le(
        bytes + FRAME_OFFSET_SOURCE_TIME
    );

    out->trace_id = get_u64_le(
        bytes + FRAME_OFFSET_TRACE_ID
    );

    for (joint = 0; joint < COURSE_ARM_DOF; ++joint) {
        out->q_target_rad[joint] = get_f64_le(
            bytes + FRAME_OFFSET_JOINT_TARGET +
                joint * sizeof(double)
        );
    }

    /*
     * generation is local runtime metadata and is not transmitted in
     * FrameV1.
     */
    out->generation = UINT64_C(0);

    /*
     * emergency is represented by the frame type byte.
     */
    out->emergency =
        frame_type == COURSE_FRAME_TYPE_ESTOP;

    return COURSE_RX_ACCEPT;
}


void frame_parser_init(FrameParser *parser) {
    if (parser != NULL) {
        memset(parser, 0, sizeof(*parser));
        parser->syncing = true;
    }
}


RxVerdict frame_feed(FrameParser *parser, uint8_t byte,
                     uint64_t last_seq, ArmCommand *out) {
    RxVerdict verdict;

    if (parser == NULL) {
        return COURSE_RX_REJECT;
    }

    /*
     * Synchronization stage:
     *
     * used == 0: no synchronization byte has been found.
     * used == 1: 0xA5 was found and the parser is waiting for 0x5A.
     */
    if (parser->syncing) {
        if (parser->used == 0) {
            if (byte == COURSE_FRAME_SYNC_0) {
                parser->bytes[0] = byte;
                parser->used = 1;
            }

            return COURSE_RX_NEED_MORE;
        }

        /*
         * Protect against an invalid parser state.
         */
        if (parser->used != 1 ||
            parser->bytes[0] != COURSE_FRAME_SYNC_0) {
            parser_reset(parser);

            if (byte == COURSE_FRAME_SYNC_0) {
                parser->bytes[0] = byte;
                parser->used = 1;
            }

            return COURSE_RX_NEED_MORE;
        }

        if (byte == COURSE_FRAME_SYNC_1) {
            /*
             * Complete synchronization word A5 5A found.
             */
            parser->bytes[1] = byte;
            parser->used = 2;
            parser->syncing = false;
        } else if (byte == COURSE_FRAME_SYNC_0) {
            /*
             * Handle overlapping synchronization input:
             *
             * A5 A5 5A
             *
             * The second A5 becomes the start of a new synchronization
             * candidate.
             */
            parser->bytes[0] = byte;
            parser->used = 1;
        } else {
            /*
             * The second synchronization byte did not match 0x5A.
             */
            parser->used = 0;
        }

        return COURSE_RX_NEED_MORE;
    }

    /*
     * Receive the remaining bytes of the synchronized frame.
     */
    if (parser->used >= COURSE_FRAME_V1_LEN) {
        parser_reset(parser);
        return COURSE_RX_REJECT;
    }

    parser->bytes[parser->used] = byte;
    parser->used++;

    /*
     * The complete FrameV1 candidate has not yet arrived.
     */
    if (parser->used < COURSE_FRAME_V1_LEN) {
        return COURSE_RX_NEED_MORE;
    }

    /*
     * A complete candidate frame is available.
     */
    verdict = frame_decode(
        parser->bytes,
        parser->used,
        last_seq,
        out
    );

    if (verdict == COURSE_RX_ACCEPT) {
        /*
         * The accepted frame has been consumed. Begin searching for the next
         * frame.
         */
        parser_reset(parser);
    } else {
        /*
         * Retain a possible later synchronization word. This supports
         * recovery from a corrupted frame and from a truncated frame followed
         * immediately by another frame.
         */
        parser_resync(parser);
    }

    return verdict;
}


// Reason freshness_accept(FreshnessGate *gate,
//                         const ArmCommand *command,
//                         uint64_t now_ns) {
//     (void)gate;
//     (void)command;
//     (void)now_ns;

//     /* DAY2_G2_TODO_A: sequence, clock, age, then latest-value update. */
//     return COURSE_REASON_STUDENT_TODO;
// }

Reason freshness_accept(FreshnessGate *gate,
                        const ArmCommand *command,
                        uint64_t now_ns) {
    /* 基本参数检查 */
    if (gate == NULL || command == NULL) {
        return COURSE_REASON_BAD_FRAME;
    }

    /* 1. 序列号递增检查 */
    if (gate->has_last && command->seq <= gate->last_seq) {
        return COURSE_REASON_NOT_NEW;
    }

    /* 2. 时钟合理性：源时间戳不能晚于当前时间（避免未来数据） */
    if (now_ns < command->t_source_ns) {
        return COURSE_REASON_CLOCK_ERROR;
    }

    /* 3. 命令年龄检查（不得陈旧） */
    if (now_ns - command->t_source_ns > COURSE_MAX_CMD_AGE_NS) {
        return COURSE_REASON_STALE_COMMAND;
    }

    /* 所有检查通过 → 更新门状态 */
    gate->last_seq = command->seq;
    gate->has_last = true;
    gate->latest = *command;   /* 结构体整体赋值，包含所有字段 */
    gate->has_latest = true;

    return COURSE_REASON_NONE;
}

Reason transport_on_message(FreshnessGate *gate,
                            const uint8_t *bytes, size_t length,
                            uint64_t now_ns, TraceRow *trace,
                            ArmCommand *accepted) {
    ArmCommand cmd;
    RxVerdict rx;
    Reason reason = COURSE_REASON_STUDENT_TODO;

    /* 初始化输出参数（若有） */
    if (trace != NULL) {
        memset(trace, 0, sizeof(*trace));
        trace->verdict = COURSE_VERDICT_REJECT;
        trace->reason = COURSE_REASON_STUDENT_TODO;
    }
    if (accepted != NULL) {
        memset(accepted, 0, sizeof(*accepted));
    }

    /*
     * 1. 尝试解码帧，但不对序列号做检查（last_seq = 0），
     *    因为序列号检查由 freshness_accept 负责。
     */
    rx = frame_decode(bytes, length, 0, &cmd);

    /* 解码失败 → 返回 BAD_FRAME，并填充 trace */
    if (rx != COURSE_RX_ACCEPT) {
        reason = COURSE_REASON_BAD_FRAME;
        if (trace != NULL) {
            trace->trace_id = 0;          /* 无法可靠获取，置零 */
            trace->seq = 0;
            trace->t_pub_ns = 0;
            trace->t_rx_ns = now_ns;
            trace->t_gate_ns = now_ns;
            trace->t_ack_ns = now_ns;
            trace->verdict = COURSE_VERDICT_REJECT;
            trace->reason = reason;
        }
        return reason;
    }

    /*
     * 2. 解码成功，调用 freshness_accept 做序列号、年龄、时钟检查。
     *    该函数内部会更新 gate 的 last_seq 和 latest（仅当接受时）。
     */
    reason = freshness_accept(gate, &cmd, now_ns);

    /*
     * 3. 填充 trace 行（四个时间戳 + 判决）
     *    t_rx_ns  = 接收时间（即 now_ns）
     *    t_gate_ns = 门检查时间（可视为 now_ns，因为检查同步完成）
     *    t_ack_ns  = 确认时间（同样设为 now_ns，满足顺序要求）
     */
    if (trace != NULL) {
        trace->trace_id = cmd.trace_id;
        trace->seq = cmd.seq;
        trace->t_pub_ns = cmd.t_source_ns;
        trace->t_rx_ns = now_ns;
        trace->t_gate_ns = now_ns;   /* 门检查瞬间完成 */
        trace->t_ack_ns = now_ns;    /* 确认瞬间完成（实际可稍后，但相等也合法） */
        trace->verdict = (reason == COURSE_REASON_NONE)
                             ? COURSE_VERDICT_APPROVE
                             : COURSE_VERDICT_REJECT;
        trace->reason = reason;
    }

    /*
     * 4. 若接受，将解码后的命令复制给 accepted（若指针非空）。
     *    freshness_accept 已更新 gate 内部状态，此处无需重复操作。
     */
    if (accepted != NULL && reason == COURSE_REASON_NONE) {
        *accepted = cmd;
    }

    return reason;
}

// Reason transport_on_message(FreshnessGate *gate,
//                             const uint8_t *bytes, size_t length,
//                             uint64_t now_ns, TraceRow *trace,
//                             ArmCommand *accepted) {
//     (void)gate;
//     (void)bytes;
//     (void)length;
//     (void)now_ns;

//     if (trace != NULL) {
//         memset(trace, 0, sizeof(*trace));
//         trace->verdict = COURSE_VERDICT_REJECT;
//         trace->reason = COURSE_REASON_STUDENT_TODO;
//     }

//     if (accepted != NULL) {
//         memset(accepted, 0, sizeof(*accepted));
//     }

//     /* DAY2_G2_TODO_B: decode, stamp t_rx, gate, latest, trace and ACK. */

//     /*
//      * DAY3_G2_TODO_A: emit a complete four-stamp trace row for runtime
//      * handoff, including rejected messages.
//      */

//     /* DAY4_G2_TODO_A: preserve corrupt and stale rejection reasons. */

//     /* DAY5_G2_TODO_A: preserve emergency source age for priority dispatch. */

//     return COURSE_REASON_STUDENT_TODO;
// }