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
        // 数值必须有限 (finite values)，拒绝 NaN 或 Infinity
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
           
        // 时间和序列号必须严格单调递增
        // 只要是严格大于 (>), 即可兼容慢消费者的 jump (gap), 
        // 并且严格拒绝了任何时间倒退、相同时间戳或序列号回放 (regression/replay)
        if (state->seq <= previous->seq) {
            return false;
        }
        if (state->t_mono_ns <= previous->t_mono_ns) {
            return false;
        }
    }

    return true; 
}

void twin_step(ArmState *state, const ArmCommand *command, int64_t step_ns) {
    if (state == NULL || command == NULL || step_ns <= 0) {
        return;
    }

    /* DAY1_G1_TODO_B: deterministic fixed-step simulator update. */
    
    // 1. 将步长从纳秒 (ns) 转换为秒 (s)
    double dt = (double)step_ns / 1e9;

    // 2. 遍历所有关节，实现确定性地向指令目标位置逼近
    for (int i = 0; i < COURSE_ARM_DOF; ++i) {
        // 计算目标位置和当前位置的误差
        double error = command->q_target_rad[i] - state->q_rad[i];
        
        // 计算在这个时间步长内，按照机器最大速率允许的最大步进量
        double max_delta = COURSE_MAX_JOINT_RATE_RAD_S * dt;
        double delta = error;

        // 对移动量进行限幅钳位 (Clamping)
        if (delta > max_delta) {
            delta = max_delta;
        } else if (delta < -max_delta) {
            delta = -max_delta;
        }

        // 确定性地更新状态位置
        state->q_rad[i] += delta;
        // 确定性地推算并更新当前速度
        state->dq_rad_s[i] = delta / dt; 
    }

    // 3. 更新状态时间与序列号
    state->t_mono_ns += step_ns;
    state->seq += 1;
}

uint64_t state_schema_hash(void) {
    /* DAY1_G1_TODO_C: hash the frozen field names, units and dimensions. */
    
    // 在工业协议与进程间通信中，通常使用哈希来验证两端使用了相同的数据结构定义
    // 这里使用 FNV-1a 算法对字段名、单位和维度特征字符串进行哈希。
    const char *schema = "seq:u64,t_mono_ns:i64,frame_id:u8,q_rad:f64[3],dq_rad_s:f64[3],sigma_q_rad:f32[3]";
    
    uint64_t hash = 0xcbf29ce484222325ULL;
    for (const char *p = schema; *p != '\0'; p++) {
        hash ^= (uint64_t)(*p);
        hash *= 0x100000001b3ULL;
    }
    
    return hash;
}