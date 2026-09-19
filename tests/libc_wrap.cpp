// Fault injection for libc calls made by wivrn_sockets.cpp and pico_sched.h.
// Same countdown scheme as openssl_wrap.cpp: test_fault_in counts down over
// wrapped calls and the call that reaches 0 fails. test_fault_ret picks the
// value that "failure" returns for integer-returning calls, so a test can make
// e.g. sendmsg() return 0 (peer gone) or -1 (hard error), and sched calls
// report success or failure on demand.
#include <dirent.h>
#include <errno.h>
#include <poll.h>
#include <sched.h>
#include <string.h>
#include <stdio.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

extern "C" int test_fault_in;
extern "C" long test_fault_ret = -1;
extern "C" const char * test_fault_only = nullptr;
// When nonzero, __wrap_poll stamps this mask into every fd's revents and
// reports all fds ready: lets tests drive poll branches (POLLERR/POLLHUP
// without POLLIN) that real sockets can't produce deterministically.
extern "C" int test_poll_revents = 0;

// Fails when the countdown hits 0, or on every call while test_fault_only names
// this function (persistent mode for order-independent injection).
#define WRAPPED_FAILS(name) \
	((test_fault_only && strcmp(test_fault_only, name) == 0) || \
	 (test_fault_in > 0 && --test_fault_in == 0))

extern "C" {

int __real_socket(int, int, int);
int __wrap_socket(int domain, int type, int protocol)
{
	return WRAPPED_FAILS("socket") ? -1 : __real_socket(domain, type, protocol);
}

int __real_bind(int, const struct sockaddr *, socklen_t);
int __wrap_bind(int fd, const struct sockaddr *addr, socklen_t len)
{
	return WRAPPED_FAILS("bind") ? -1 : __real_bind(fd, addr, len);
}

int __real_connect(int, const struct sockaddr *, socklen_t);
int __wrap_connect(int fd, const struct sockaddr *addr, socklen_t len)
{
	return WRAPPED_FAILS("connect") ? -1 : __real_connect(fd, addr, len);
}

int __real_listen(int, int);
int __wrap_listen(int fd, int n)
{
	return WRAPPED_FAILS("listen") ? -1 : __real_listen(fd, n);
}

int __real_accept(int, struct sockaddr *, socklen_t *);
int __wrap_accept(int fd, struct sockaddr *addr, socklen_t *len)
{
	return WRAPPED_FAILS("accept") ? -1 : __real_accept(fd, addr, len);
}

int __real_setsockopt(int, int, int, const void *, socklen_t);
int __wrap_setsockopt(int fd, int level, int name, const void *val, socklen_t len)
{
	return WRAPPED_FAILS("setsockopt") ? -1 : __real_setsockopt(fd, level, name, val, len);
}

ssize_t __real_recvfrom(int, void *, size_t, int, struct sockaddr *, socklen_t *);
ssize_t __wrap_recvfrom(int fd, void *buf, size_t n, int flags, struct sockaddr *addr, socklen_t *len)
{
	return WRAPPED_FAILS("recvfrom") ? (ssize_t)test_fault_ret : __real_recvfrom(fd, buf, n, flags, addr, len);
}

ssize_t __real_recv(int, void *, size_t, int);
ssize_t __wrap_recv(int fd, void *buf, size_t n, int flags)
{
	return WRAPPED_FAILS("recv") ? (ssize_t)test_fault_ret : __real_recv(fd, buf, n, flags);
}

int __real_recvmmsg(int, struct mmsghdr *, unsigned int, int, struct timespec *);
int __wrap_recvmmsg(int fd, struct mmsghdr *vm, unsigned int n, int flags, struct timespec *t)
{
	return WRAPPED_FAILS("recvmmsg") ? (int)test_fault_ret : __real_recvmmsg(fd, vm, n, flags, t);
}

ssize_t __real_sendmsg(int, const struct msghdr *, int);
ssize_t __wrap_sendmsg(int fd, const struct msghdr *msg, int flags)
{
	return WRAPPED_FAILS("sendmsg") ? (ssize_t)test_fault_ret : __real_sendmsg(fd, msg, flags);
}

int __real_sendmmsg(int, struct mmsghdr *, unsigned int, int);
int __wrap_sendmmsg(int fd, struct mmsghdr *vm, unsigned int n, int flags)
{
	return WRAPPED_FAILS("sendmmsg") ? (int)test_fault_ret : __real_sendmmsg(fd, vm, n, flags);
}

ssize_t __real_writev(int, const struct iovec *, int);
ssize_t __wrap_writev(int fd, const struct iovec *iov, int n)
{
	return WRAPPED_FAILS("writev") ? (ssize_t)test_fault_ret : __real_writev(fd, iov, n);
}

DIR *__real_opendir(const char *);
DIR *__wrap_opendir(const char *name)
{
	return WRAPPED_FAILS("opendir") ? nullptr : __real_opendir(name);
}

FILE *__real_fopen(const char *, const char *);
FILE *__wrap_fopen(const char *name, const char *mode)
{
	return WRAPPED_FAILS("fopen") ? nullptr : __real_fopen(name, mode);
}

int __real_sched_setscheduler(pid_t, int, const struct sched_param *);
int __wrap_sched_setscheduler(pid_t pid, int policy, const struct sched_param *p)
{
	return WRAPPED_FAILS("sched_setscheduler") ? (int)test_fault_ret : __real_sched_setscheduler(pid, policy, p);
}

int __real_sched_setaffinity(pid_t, size_t, const cpu_set_t *);
int __wrap_sched_setaffinity(pid_t pid, size_t sz, const cpu_set_t *mask)
{
	return WRAPPED_FAILS("sched_setaffinity") ? (int)test_fault_ret : __real_sched_setaffinity(pid, sz, mask);
}

int __real_sched_getaffinity(pid_t, size_t, cpu_set_t *);
int __wrap_sched_getaffinity(pid_t pid, size_t sz, cpu_set_t *mask)
{
	return WRAPPED_FAILS("sched_getaffinity") ? (int)test_fault_ret : __real_sched_getaffinity(pid, sz, mask);
}

int __real_setpriority(int, int, int);
int __wrap_setpriority(int which, int who, int prio)
{
	return WRAPPED_FAILS("setpriority") ? (int)test_fault_ret : __real_setpriority(which, who, prio);
}

long __real_sysconf(int);
long __wrap_sysconf(int name)
{
	return WRAPPED_FAILS("sysconf") ? test_fault_ret : __real_sysconf(name);
}

int __real_poll(struct pollfd *, nfds_t, int);
int __wrap_poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
	if (test_poll_revents)
	{
		for (nfds_t i = 0; i < nfds; ++i)
			fds[i].revents = test_poll_revents;
		return nfds;
	}
	if (WRAPPED_FAILS("poll"))
	{
		errno = EINTR;
		return -1;
	}
	return __real_poll(fds, nfds, timeout);
}
}
