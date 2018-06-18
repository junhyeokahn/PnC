#include "DartpThread.hpp"
#include <iostream>

DartpThread::DartpThread() :
    sejong_thread(),
    firstLoopFlag(false),
    isRunning(false)
{}

DartpThread::~DartpThread()
{
    pthread_cancel(sejong_thread);
    pthread_join(sejong_thread, NULL);
}
void DartpThread::terminate()
{
    printf("terminating thread\n");
    isRunning = false;
}
bool DartpThread::isFirstLoop()
{
    return firstLoopFlag;
}

void *runThread(void * arg)
{
    ((DartpThread*) arg)->run();
    return NULL;
}

void sigint(int signo)
{
    (void) signo;
}
void DartpThread::start()
{
    if(!isRunning){
        sigset_t sigset, oldset;
        sigemptyset(&sigset);
        sigaddset(&sigset, SIGINT);
        pthread_sigmask(SIG_BLOCK, &sigset, &oldset);
        pthread_create(&sejong_thread, NULL, runThread, this);
        struct sigaction s;
        s.sa_handler = sigint;
        sigemptyset(&s.sa_mask);
        s.sa_flags = 0;
        sigaction(SIGINT, &s, NULL);
        pthread_sigmask(SIG_SETMASK, &oldset, NULL);

        isRunning = true;
    }
}

