#pragma once

#if defined(_WIN32) || defined(WIN32)        /*Windows*/
#define WINDOWS_IMPL
#include <windows.h>
#include <time.h>                 //time(), clock()
#include <Mmsystem.h>             //timeGetTime()
#pragma comment(lib, "Winmm.lib") //timeGetTime()
#elif defined(__linux__) || defined(__APPLE__) || defined(__FreeBSD__) || defined(BSD)   /*Linux*/
#define LINUX_IMPL
#include <sys/time.h>            //gettimeofday()
#endif
#include <stdio.h>


class timer {
public:
	timer();
	~timer();
	double getClock();

private:
	#ifdef WINDOWS_IMPL
		__int64 start;
	#elif defined(LINUX_IMPL)
		struct timeval start;
	#endif
};
