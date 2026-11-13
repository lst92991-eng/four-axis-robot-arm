#pragma once
#include <cstddef>
using ssize_t = std::ptrdiff_t;
#define STDIN_FILENO 0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2
#ifndef SIGKILL
#define SIGKILL 9
#endif
#define _exit scara_test_exit
extern "C" {
int fork();
int close(int);
int chdir(const char *);
int dup2(int, int);
int execl(const char *, const char *, ...);
int kill(int, int);
[[noreturn]] void scara_test_exit(int);
}
