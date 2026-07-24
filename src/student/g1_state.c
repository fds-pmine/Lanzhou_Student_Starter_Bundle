#include "course.h"

#include <stdbool.h>
#include <stddef.h>
#include <math.h>

bool state_valid(const ArmState *state, const ArmState *previous) {
    if (state == NULL) {
        return false;
    }

    /* DAY1_G1_TODO_A: finite values, coordinate frame, and joint limits. */
    // 1. 坐标系 (coordinate frame) 校验
    if (state->frame_id != COURSE_FRAME_ID_BASE) {
        return false;
    }

    // 2. 遍历所有关节，检查有限值、关节极限和不确定度
    for (int i = 0; i < COURSE_ARM_DOF; ++i) {
        // 数值必须有限 (finite values)
        if (!isfinite(state->q_rad[i]) || 
            !isfinite(state->dq_rad_s[i]) || 
            !isfinite(state->sigma_q_rad[i])) {
            return false;
        }

        // 关节位置约束 (joint limits)
        if (state->q_rad[i] < COURSE_JOINT_MIN_RAD || 
            state->q_rad[i] > COURSE_JOINT_MAX_RAD) {
            return false;
        }

        /* DAY4_G1_TODO_A: reject excessive state uncertainty before handoff to the safety gate. */
        // 状态的不确定度 (uncertainty) 必须在安全范围内
        if (state->sigma_q_rad[i] > COURSE_MAX_SIGMA_Q_RAD) {
            return false;
        }
    }

    if (previous != NULL) {
        /* DAY2_G1_TODO_A: reject non-increasing sequence or monotonic time. */
        /* DAY3_G1_TODO_A: accept a valid source progression even when a slow
           consumer observes a larger sequence and time gap. */
        /* DAY5_G1_TODO_A: reject sequence or time regression during replay across
           the controller-host boundary. */
           
        if (state->seq == 0) {
            // 控制器发生了重启 (Fresh State)，序列号归 0。
            // 此时只需要确保这不是一次连续发了两个 seq=0 的重放即可。
            if (state->seq == previous->seq) {
                return false; 
            }
        } else {
            // 常规数据流：必须严格递增。拒绝任何序列号或时间的停滞与倒退。
            if (state->seq <= previous->seq) {
                return false;
            }
            if (state->t_mono_ns <= previous->t_mono_ns) {
                return false;
            }
        }
    }

    return true; 
}

void twin_step(ArmState *state, const ArmCommand *command, int64_t step_ns) {
    if (state == NULL || command == NULL || step_ns <= 0) {
        return;
    }

    /* DAY1_G1_TODO_B: deterministic fixed-step simulator update. */
    double dt = (double)step_ns / 1e9;

    for (int i = 0; i < COURSE_ARM_DOF; ++i) {
        double error = command->q_target_rad[i] - state->q_rad[i];
        
        double max_delta = COURSE_MAX_JOINT_RATE_RAD_S * dt;
        double delta = error;

        if (delta > max_delta) {
            delta = max_delta;
        } else if (delta < -max_delta) {
            delta = -max_delta;
        }

        state->q_rad[i] += delta;
        state->dq_rad_s[i] = delta / dt; 
    }

    state->t_mono_ns += step_ns;
    state->seq += 1;
}

uint64_t state_schema_hash(void) {
    /* DAY1_G1_TODO_C: hash the frozen field names, units and dimensions. */
    const char *schema = "seq:u64,t_mono_ns:i64,frame_id:u8,q_rad:f64[3],dq_rad_s:f64[3],sigma_q_rad:f32[3]";
    
    uint64_t hash = 0xcbf29ce484222325ULL;
    for (const char *p = schema; *p != '\0'; p++) {
        hash ^= (uint64_t)(*p);
        hash *= 0x100000001b3ULL;
    }
    
    return hash;
}