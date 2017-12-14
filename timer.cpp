#include "timer.h"


timer::timer() {
   #ifdef WINDOWS_IMPL
	LARGE_INTEGER  large_interger;
	QueryPerformanceCounter(&large_interger);
	start = large_interger.QuadPart;
   #elif defined(LINUX_IMPL)
	gettimeofday(&start, NULL);
   #endif
}
timer::~timer() {

}

double timer::getClock() {
	#ifdef WINDOWS_IMPL
		LARGE_INTEGER  large_interger;
		QueryPerformanceFrequency(&large_interger);
		double dff = large_interger.QuadPart;  //高精度计时器频率
		QueryPerformanceCounter(&large_interger);
		return  (large_interger.QuadPart - start)*1000 / dff; //ms
	#elif defined(LINUX_IMPL)
		struct timeval end;
		gettimeofday(&end, NULL);
		return 1000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)*1.0 / 1000; //ms
	#endif	
}