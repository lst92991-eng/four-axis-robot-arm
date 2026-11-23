#pragma once
#include <cstddef>
using ssize_t = std::ptrdiff_t;
#define AF_UNIX 1
#define SOCK_STREAM 1
#define MSG_NOSIGNAL 0x4000
#define MSG_DONTWAIT 0x40
#define SOL_SOCKET 1
#define SO_RCVTIMEO 20
#define SO_SNDTIMEO 21
extern "C" {
int socketpair(int, int, int, int[2]);
int setsockopt(int, int, int, const void *, unsigned);
ssize_t send(int, const void *, size_t, int);
ssize_t recv(int, void *, size_t, int);
}
