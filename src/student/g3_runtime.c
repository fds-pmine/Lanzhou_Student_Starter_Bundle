#define _POSIX_C_SOURCE 200809L

#include "course.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/epoll.h>
#include <unistd.h>

bool trace_complete(const TraceRow *trace) {
    /* DAY1_G3_TODO_A: freeze one monotonic timing and workload baseline. */
    /* DAY2_G3_TODO_A: validate four-stamp ordering and queue evidence. */
    if (trace == NULL) {
        return false;
    }

    if (trace->t_pub_ns == 0 || trace->trace_id == 0 || trace->seq == 0) {
        return false;
    }

    if (trace->t_ack_ns - trace->t_pub_ns == 0) {
        return false;
    }

    if (trace->t_rx_ns < trace->t_pub_ns ||
        trace->t_gate_ns < trace->t_rx_ns ||
        trace->t_ack_ns < trace->t_gate_ns) {
        return false;
    }

    if (trace->verdict != COURSE_VERDICT_APPROVE &&
        trace->verdict != COURSE_VERDICT_REJECT &&
        trace->verdict != COURSE_VERDICT_FALLBACK &&
        trace->verdict != COURSE_VERDICT_DISCARD) {
        return false;
    }

    return true;
}

int set_nonblocking(int fd) {
    int flags;

    if (fd < 0) {
        errno = EINVAL;
        return -1;
    }
    flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return -1;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

int poll_service_once(ClientState *clients, size_t client_count,
                      int timeout_ms, RuntimeStats *stats) {
    struct pollfd fds[COURSE_MAX_CLIENTS];
    int ready;

    if (stats == NULL || client_count > COURSE_MAX_CLIENTS ||
        (client_count != 0 && clients == NULL)) {
        errno = EINVAL;
        return -1;
    }

    memset(stats, 0, sizeof(*stats));
    stats->max_queue_age_ns = COURSE_MAX_QUEUE_AGE_NS;
    stats->max_work_per_client = COURSE_WORK_BUDGET;

    memset(fds, 0, sizeof(fds));
    for (size_t index = 0; index < client_count; index++) {
        clients[index].work_this_turn = 0;
        fds[index].fd = clients[index].fd;
        fds[index].events = POLLIN;
    }

    ready = poll(fds, (nfds_t)client_count, timeout_ms);
    if (ready <= 0) {
        return ready;
    }

    for (size_t index = 0; index < client_count; index++) {
        if ((fds[index].revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
            stats->ready_events++;
        }
    }

    /* ========================== DAY4 部分（过期清理） ========================== */
    for (size_t i = 0; i < client_count; i++) {
        ClientState *client = &clients[i];
        short revents = fds[i].revents;

        if (!(revents & (POLLIN | POLLHUP | POLLERR))) {
            continue;
        }
        if (revents & (POLLHUP | POLLERR)) {
            continue;
        }

        uint64_t now = monotonic_ns();

        /* DAY4：服务前检查队列年龄，超时则清空并记录丢弃事件 */
        if (client->oldest_enqueue_ns != 0 &&
            (now - client->oldest_enqueue_ns) > COURSE_MAX_QUEUE_AGE_NS) {
            size_t dropped_bytes = client->queued_bytes;
            if (dropped_bytes > 0) {
                client->dropped_total += dropped_bytes;
                stats->flood_dropped++;   // 过期丢弃事件
                client->input_used = 0;
                client->queued_bytes = 0;
                client->oldest_enqueue_ns = 0;
            }
        }

        /* ========================== DAY3 部分（工作预算 + 部分帧 + 背压） ========================== */
        size_t work_done = 0;

        while (work_done < COURSE_WORK_BUDGET) {
            size_t available_space = COURSE_CLIENT_BUFFER_SIZE - client->input_used;
            bool queue_full = (client->queued_bytes >= COURSE_MAX_QUEUE_BYTES) ||
                              (available_space == 0);

            if (queue_full) {
                /* 背压：队列满，丢弃新数据 */
                char tmp[512];
                ssize_t n = read(client->fd, tmp, sizeof(tmp));
                if (n <= 0) {
                    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                        break;
                    }
                    break;
                }
                client->dropped_total += (size_t)n;
                stats->flood_dropped++;
                work_done++;
                continue;
            }

            /* 正常读取并保留到 input 缓冲区 */
            size_t to_read = (available_space < 512) ? available_space : 512;
            ssize_t n = read(client->fd, client->input + client->input_used, to_read);
            if (n <= 0) {
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    break;
                }
                break;
            }

            /* ------ 数据成功入队，更新各项统计（此处为之前遗漏的部分） ------ */
            client->input_used += (size_t)n;
            client->queued_bytes += (size_t)n;

            // 统计更新：客户端累计调度字节数
            client->dispatched_total += (size_t)n;

            // 统计更新：根据客户端类别更新全局统计
            switch (client->class_id) {
                case COURSE_CLIENT_FAST:
                    stats->fast_dispatched += (size_t)n;
                    break;
                case COURSE_CLIENT_SAFETY:
                    stats->safety_dispatched += (size_t)n;
                    break;
                case COURSE_CLIENT_SLOW:
                    stats->slow_dispatched += (size_t)n;
                    break;
                case COURSE_CLIENT_FLOOD:
                default:
                    // 对于 FLOOD 客户端，若队列未满，说明其当前未触发背压，
                    // 将其归入 slow 类别统计，以反映其对资源的占用。
                    stats->slow_dispatched += (size_t)n;
                    break;
            }

            // 如果是本次轮询中该客户端的第一次入队，记录入队时间
            if (client->oldest_enqueue_ns == 0) {
                client->oldest_enqueue_ns = now;
            }
            work_done++;
        }

        client->work_this_turn = work_done;
    }

    return ready;
}

int epoll_wait_priority(int epoll_fd, int wake_fd, int controller_fd,
                        int timer_fd, int motion_fd, int timeout_ms,
                        uint64_t cancel_generation, ReadyEvent *event) {
    struct epoll_event events[8];
    int ready;

    if (epoll_fd < 0 || event == NULL) {
        errno = EINVAL;
        return -1;
    }
    memset(event, 0, sizeof(*event));
    event->kind = COURSE_READY_NONE;
    event->cancel_generation = cancel_generation;

    ready = epoll_wait(epoll_fd, events,
                       (int)(sizeof(events) / sizeof(events[0])), timeout_ms);
    if (ready <= 0) {
        return ready;
    }

    /* DAY5_G3_TODO_A: select emergency wake first, then timer, controller,
       and motion readiness without losing the cancellation generation. */
    (void)wake_fd;
    (void)controller_fd;
    (void)timer_fd;
    (void)motion_fd;
    (void)events;
    event->observed_ns = monotonic_ns();
    return ready;
}
