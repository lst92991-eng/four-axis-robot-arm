#pragma once
#define F_GETFL 3
#define F_SETFL 4
#define O_NONBLOCK 0x800
#define O_WRONLY 1
extern "C" int fcntl(int, int, ...);
extern "C" int open(const char *, int, ...);
