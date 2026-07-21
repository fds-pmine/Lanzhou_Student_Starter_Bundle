#define _POSIX_C_SOURCE 200809L

#include "course.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/epoll.h>

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

    /* The supplied scaffold performs the readiness wait without blocking on a
       client. Students own the bounded service policy after poll returns. */
    ready = poll(fds, (nfds_t)client_count, timeout_ms);
    if (ready <= 0) {
        return ready;
    }

    for (size_t index = 0; index < client_count; index++) {
        if ((fds[index].revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
            stats->ready_events++;
        }
    }

    /* DAY3_G3_TODO: retain partial frames, enforce COURSE_WORK_BUDGET, and
       apply byte backpressure while servicing each ready descriptor. */
    /* DAY4_G3_TODO_A: expire over-age queued work before service and expose
       queue age/drop evidence without blocking. */
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
