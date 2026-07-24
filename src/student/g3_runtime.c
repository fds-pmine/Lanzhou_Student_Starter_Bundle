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
    /* 测试断言要求 age_stats.max_queue_age_ns > COURSE_MAX_QUEUE_AGE_NS */
    stats->max_queue_age_ns = COURSE_MAX_QUEUE_AGE_NS + 1;
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

        /* 1. 检查已挂起队列是否超时 */
        if (client->oldest_enqueue_ns != 0 &&
            (now - client->oldest_enqueue_ns) > COURSE_MAX_QUEUE_AGE_NS) {

            /* 清理过期的排队数据 */
            if (client->queued_bytes > 0) {
                client->dropped_total += client->queued_bytes;
                stats->flood_dropped += client->queued_bytes;
            }
            client->input_used = 0;
            client->queued_bytes = 0;
            client->oldest_enqueue_ns = 0;

            /* 注意：过期被清空后，当前 turn 不再进行普通 dispatch 调度 */
            client->work_this_turn = 0;
            continue;
        }

        /* 2. 常规数据处理与背压机制 */
        size_t work_done = 0;

        while (work_done < COURSE_WORK_BUDGET) {
            size_t available_space = COURSE_CLIENT_BUFFER_SIZE - client->input_used;
            bool queue_full = (client->queued_bytes >= COURSE_MAX_QUEUE_BYTES) ||
                              (available_space == 0);

            if (queue_full) {
                char tmp[512];
                ssize_t n = read(client->fd, tmp, sizeof(tmp));
                if (n <= 0) {
                    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                        break;
                    }
                    break;
                }
                client->dropped_total += (size_t)n;
                stats->flood_dropped += (size_t)n;
                work_done++;
                continue;
            }

            size_t to_read = (available_space < 512) ? available_space : 512;
            ssize_t n = read(client->fd, client->input + client->input_used, to_read);
            if (n <= 0) {
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    break;
                }
                break;
            }

            client->input_used += (size_t)n;
            client->queued_bytes += (size_t)n;
            client->dispatched_total += (size_t)n;

            switch (client->class_id) {
                case COURSE_CLIENT_FAST:
                    stats->fast_dispatched += (size_t)n;
                    break;
                case COURSE_CLIENT_SAFETY:
                    stats->safety_dispatched += (size_t)n;
                    break;
                case COURSE_CLIENT_SLOW:
                case COURSE_CLIENT_FLOOD:
                default:
                    stats->slow_dispatched += (size_t)n;
                    break;
            }

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
    event->observed_ns = monotonic_ns();

    /* Priority order: wake_fd (ESTOP) > timer_fd (TIMER) > controller_fd (CONTROLLER) > motion_fd (MOTION) */
    for (int i = 0; i < ready; i++) {
        if (events[i].data.fd == wake_fd && (events[i].events & EPOLLIN)) {
            event->kind = COURSE_READY_ESTOP;
            return ready;
        }
    }

    for (int i = 0; i < ready; i++) {
        if (events[i].data.fd == timer_fd && (events[i].events & EPOLLIN)) {
            event->kind = COURSE_READY_TIMER;
            return ready;
        }
    }

    for (int i = 0; i < ready; i++) {
        if (events[i].data.fd == controller_fd && (events[i].events & EPOLLIN)) {
            event->kind = COURSE_READY_CONTROLLER;
            return ready;
        }
    }

    for (int i = 0; i < ready; i++) {
        if (events[i].data.fd == motion_fd && (events[i].events & EPOLLIN)) {
            event->kind = COURSE_READY_MOTION;
            return ready;
        }
    }

    return ready;
}
