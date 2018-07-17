#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#define LOOP_COUNT 100

using namespace std;
double compute_timediff(struct timeval a,struct timeval b);
int run_loopsimple()
{

	int a[LOOP_COUNT][LOOP_COUNT];
	int b[LOOP_COUNT][LOOP_COUNT];
	int c[LOOP_COUNT][LOOP_COUNT];

	for(int i = 0; i < LOOP_COUNT; i++)
	{

		for(int j = 0; j < LOOP_COUNT;j++)
		{

			c[i][j] = 0;
			a[i][j] = i;
			b[i][j] = j;
			for(int k = 0; k < LOOP_COUNT;k++)
			{
				c[i][j] += a[i][k] + b[k][j];
			}

		}

	}
	int sum = 0;
	for(int i = 0; i < LOOP_COUNT; i++)
	{
		for(int j = 0; j < LOOP_COUNT; j++)
		{
			sum += c[i][j];
		}
	}
	return sum;
}
int run_looppointer()
{
	int a[LOOP_COUNT][LOOP_COUNT];
		int b[LOOP_COUNT][LOOP_COUNT];
		int c[LOOP_COUNT][LOOP_COUNT];
		for(int i = 0; i < LOOP_COUNT;i++)
		{
			int *cptr = c[i];
			int *bptr = b[0];
			for(int j = 0; j < LOOP_COUNT; j++)
			{
				c[i][j] = 0;
				a[i][j] = i;
				b[i][j] = j;
				int *aptr = a[i];
				*cptr = (*aptr++) * (*bptr++);
				for(int k = 0; k < LOOP_COUNT;k++)
				{
					*cptr += (*aptr++) * b[k][j];
				}
				cptr++;
			}
		}
		int sum = 0;
			for(int i = 0; i < LOOP_COUNT; i++)
			{
				for(int j = 0; j < LOOP_COUNT; j++)
				{
					sum += c[i][j];
				}
			}
		return sum;
}
int main (void)
{
	struct timeval start;
	struct timeval now;
	double time_loopsimple,time_looppointer;
	usleep(100000);
	cout << "Running Simple Loop Iterator." << endl;
	gettimeofday(&start,NULL);
	for(int i = 0; i < LOOP_COUNT*10; i++)
	{
		run_loopsimple();
	}

	gettimeofday(&now,NULL);
	time_loopsimple = compute_timediff(start,now);
	cout << "Time delta: " << time_loopsimple << " sec." << std::endl;
	cout << "Running Pointer Loop Iterator." << endl;
	gettimeofday(&start,NULL);
	for(int i = 0; i < LOOP_COUNT*10; i++)
	{
		run_looppointer();
	}
	gettimeofday(&now,NULL);
	time_looppointer = compute_timediff(start,now);
	cout << "Time delta: " << time_looppointer << " sec." << std::endl;
	cout << "Time Improvement: " << 100.0*(time_loopsimple-time_looppointer)/(time_loopsimple) << "%" << std::endl;
	return 0;
}
double compute_timediff(struct timeval a,struct timeval b)
{
	double t1 = (double)a.tv_sec + (double)a.tv_usec/1000000.0;
	double t2 = (double)b.tv_sec + (double)b.tv_usec/1000000.0;
	return t2-t1;
}
