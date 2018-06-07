#ifndef _CTHREADBASE_H_
#define _CTHREADBASE_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
using namespace std;

class CThreadBase
{
public:
	CThreadBase();
	~CThreadBase();
	bool Create();
	int Cancel();
	virtual void Run();
private:
	static void* doPthread(void *pSelf);
	pthread_t tid;
};


#endif