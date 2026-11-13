#pragma once
#define WNOHANG 1
extern "C" int waitpid(int, int *, int);
