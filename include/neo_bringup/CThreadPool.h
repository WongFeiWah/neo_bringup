#ifndef _CTHREADPOOL_H_
#define _CTHREADPOOL_H_

#include <stdio.h>
#include <stdlib.h>
#include <list>
#include <unistd.h>
#include <pthread.h>
using namespace std;

class CTask
{
public:
	CTask();
	virtual ~CTask();
	virtual void doAction() = 0;
private:

};

class CThreadPool
{
public:
	CThreadPool(int maxCount = 10);
	~CThreadPool();
	void addTask(CTask *Task);
	static void *runtine(void *pSelf);
protected:
	pthread_t tid;
	int m_maxCount;
	int m_nowCount;
	int m_waitCount;
	list<CTask*> threadlist;
	pthread_mutex_t m_mutex;
	pthread_cond_t m_cond;
	pthread_cond_t end_cond;
	bool isQuit;
};
#endif