#pragma once
#define POLLIN 1
#define POLLOUT 4
#define POLLERR 8
#define POLLHUP 16
struct pollfd { int fd; short events; short revents; };
extern "C" int poll(pollfd *, unsigned long, int);
