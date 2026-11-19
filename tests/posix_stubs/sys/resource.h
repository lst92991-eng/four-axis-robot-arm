#pragma once
#define RLIMIT_AS 9
struct rlimit { unsigned long rlim_cur; unsigned long rlim_max; };
extern "C" int setrlimit(int, const rlimit *);
