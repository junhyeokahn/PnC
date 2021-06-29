#include "Utils/IO/Pthread.hpp"
#include <iostream>

Pthread::Pthread() : sejong_thread(), firstLoopFlag(false), isRunning(false) {}

Pthread::~Pthread() {
  pthread_cancel(sejong_thread);
  pthread_join(sejong_thread, NULL);
}
void Pthread::terminate() {
  printf("terminating thread\n");
  isRunning = false;
}
bool Pthread::isFirstLoop() { return firstLoopFlag; }

void *runThread(void *arg) {
  ((Pthread *)arg)->run();
  return NULL;
}

void sigint(int signo) { (void)signo; }
void Pthread::start() {
  if (!isRunning) {
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
