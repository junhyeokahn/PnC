#ifndef DARTPTHREAD
#define DARTPTHREAD

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
 
class DartpThread{
protected:
    pthread_t sejong_thread;
    bool firstLoopFlag;
    bool isRunning;


    void terminate();
    bool isFirstLoop();

public:
    DartpThread();
    virtual ~DartpThread(void);
    virtual void run(void) = 0;

    void start();
};
void *runData(void * arg);
void sigint(int signo);

#endif
