#include "course.h"
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

// ==================== 消费者相关定义 ====================
#define MAX_CONSUMERS       10
#define CONSUMER_BUF_SIZE   16

typedef struct {
    int         id;
    bool        is_slow;
    ArmState    state_buf[CONSUMER_BUF_SIZE];
    int64_t     recv_ns_buf[CONSUMER_BUF_SIZE];   // 每条状态被广播时的单调时间
    int         head;
    int         tail;
    int         count;
    int         dropped;          // 因缓冲区满丢弃的条目数
} Consumer;

// 全局消费者（测试会直接访问）
Consumer consumers[MAX_CONSUMERS];
int num_consumers = 0;

// ==================== 第一天 & 第二天 ====================
bool state_valid(const ArmState *state, const ArmState *previous) {
    if (state == NULL) return false;

    if (state->frame_id != COURSE_FRAME_ID_BASE)
        return false;

    for (int i = 0; i < COURSE_ARM_DOF; ++i) {
        if (!isfinite(state->q_rad[i]) ||
            !isfinite(state->dq_rad_s[i]) ||
            !isfinite(state->sigma_q_rad[i]))
            return false;

        if (state->q_rad[i] < COURSE_JOINT_MIN_RAD ||
            state->q_rad[i] > COURSE_JOINT_MAX_RAD)
            return false;

        if (state->sigma_q_rad[i] > COURSE_MAX_SIGMA_Q_RAD)
            return false;
    }

    if (previous != NULL) {
        if (state->seq <= previous->seq)
            return false;
        if (state->t_mono_ns < previous->t_mono_ns)
            return false;
    }
    return true;
}

void twin_step(ArmState *state, const ArmCommand *command, int64_t step_ns) {
    if (state == NULL || command == NULL || step_ns <= 0)
        return;

    double dt = (double)step_ns / 1e9;

    for (int i = 0; i < COURSE_ARM_DOF; ++i) {
        double error = command->q_target_rad[i] - state->q_rad[i];
        double max_delta = COURSE_MAX_JOINT_RATE_RAD_S * dt;
        double delta = error;
        if (delta > max_delta) delta = max_delta;
        else if (delta < -max_delta) delta = -max_delta;

        state->q_rad[i] += delta;
        state->dq_rad_s[i] = delta / dt;
    }

    state->t_mono_ns += step_ns;
    state->seq += 1;
}

uint64_t state_schema_hash(void) {
    const char *schema = "seq:u64,t_mono_ns:i64,frame_id:u8,q_rad:f64[3],dq_rad_s:f64[3],sigma_q_rad:f32[3]";
    uint64_t hash = 0xcbf29ce484222325ULL;
    for (const char *p = schema; *p != '\0'; p++) {
        hash ^= (uint64_t)(*p);
        hash *= 0x100000001b3ULL;
    }
    return hash;
}

// ==================== 第三天新增 ====================
void init_consumer(int id, bool is_slow) {
    if (num_consumers >= MAX_CONSUMERS) return;
    consumers[num_consumers].id = id;
    consumers[num_consumers].is_slow = is_slow;
    consumers[num_consumers].head = 0;
    consumers[num_consumers].tail = 0;
    consumers[num_consumers].count = 0;
    consumers[num_consumers].dropped = 0;
    num_consumers++;
}

// 非阻塞广播：向所有消费者推送状态，满则覆盖最旧
void broadcast_state(const ArmState *state) {
    if (state == NULL) return;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t now_ns = (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;

    for (int i = 0; i < num_consumers; i++) {
        Consumer *c = &consumers[i];

        c->state_buf[c->tail] = *state;
        c->recv_ns_buf[c->tail] = now_ns;

        c->tail = (c->tail + 1) % CONSUMER_BUF_SIZE;
        if (c->count == CONSUMER_BUF_SIZE) {
            c->head = (c->head + 1) % CONSUMER_BUF_SIZE;
            c->dropped++;
        } else {
            c->count++;
        }
    }
}

// 消费者读取：取出最旧状态，成功返回 true，缓冲区空返回 false
bool consume_state(int consumer_id, ArmState *out_state, int64_t *out_recv_ns) {
    if (consumer_id < 0 || consumer_id >= num_consumers) return false;
    Consumer *c = &consumers[consumer_id];
    if (c->count == 0) return false;
    *out_state = c->state_buf[c->head];
    if (out_recv_ns) *out_recv_ns = c->recv_ns_buf[c->head];
    c->head = (c->head + 1) % CONSUMER_BUF_SIZE;
    c->count--;
    return true;
}

// 导出 CSV（供测试分析）
void export_freshness_csv(const char *filename) {
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        perror("无法创建 freshness-clients.csv");
        return;
    }

    fprintf(fp, "consumer_id,is_slow,seq,t_mono_ns,recv_ns,dropped\n");

    for (int i = 0; i < num_consumers; i++) {
        Consumer *c = &consumers[i];
        int idx = c->head;
        for (int j = 0; j < c->count; j++) {
            ArmState *st = &c->state_buf[idx];
            fprintf(fp, "%d,%d,%llu,%lld,%lld,%d\n",
                    c->id,
                    c->is_slow ? 1 : 0,
                    (unsigned long long)st->seq,
                    (long long)st->t_mono_ns,
                    (long long)c->recv_ns_buf[idx],
                    c->dropped);
            idx = (idx + 1) % CONSUMER_BUF_SIZE;
        }
    }
    fclose(fp);
}