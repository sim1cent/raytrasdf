#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include "raytrasdf.h"

#define THREADS_N (4)
#define TASKS_N (50)
#define SHOW_PROGRESS

static uint count;
static pthread_mutex_t lock;
static uint s;
static V3 *matrix;

static void *thread_work(void (*render_worker)(uint,uint,V3*)){
    while(1){
        pthread_mutex_lock(&lock);
        uint i=count;
        ++count;
#ifdef SHOW_PROGRESS
        if(i<TASKS_N) fprintf(stderr,"\r%02u%%",100*i/TASKS_N);
#endif
        pthread_mutex_unlock(&lock);
        if(i>=TASKS_N)
            break;
        render_worker(i*s/TASKS_N,(i+1)*s/TASKS_N,matrix);
    }
    return NULL;
}

void start_work(void (*render_worker)(uint,uint,V3*), uint size, V3 *mat){
    pthread_t threads[THREADS_N];
    pthread_mutex_init(&lock,NULL);
    count=0;
    s=size;
    matrix=mat;
    for(uint i=0;i<THREADS_N;++i)
        pthread_create(threads+i,NULL,(void*(*)(void*))thread_work,render_worker);
    for(uint i=0;i<THREADS_N;++i)
        pthread_join(threads[i],NULL);
    pthread_mutex_destroy(&lock);
#ifdef SHOW_PROGRESS
    fputs("\r\033[KDone!\n",stderr);
#endif
}
