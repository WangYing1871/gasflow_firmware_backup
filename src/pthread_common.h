#ifndef pthread_common_H
#define pthread_common_H

//#include <zephyr/logging/log.h>
//LOG_MODULE_REGISTER(GFC,LOG_LEVEL_INF);
//#include "posix_internal.h"
//#include <pthread.h>
//
//#define TASK_NUM 4
//
//#define TASK_ID_SUBMIT_DATA 0
//static bool is_submit_data = false;
//typedef pthread_mutex_t* mutex_t;
//static pthread_mutex_t mutexs[TASK_NUM];
//static pthread_t threads[TASK_NUM];
//
//inline void mutex_init(mutex_t m){
//  int ret;
//  ret = pthread_mutex_init(m,NULL);
//  if (IS_ENABLED(CONFIG_SAMPLE_ERROR_CHECKING) && ret != 0){
//    errno = ret;
//    //perror("pthread_mutex_init");
//    __ASSERT(false,"failed to initialize fork");
//  }
//}
//
//void init_mutexs(void){
//  ARRAY_FOR_EACH(mutexs,i){
//    //LOG_INF("initialization mutex_t %zu",i); 
//    mutex_init(&mutexs[i]);
//  }
//}
//
//inline void take(mutex_t m){
//  int ret;
//  ret = pthread_mutex_lock(m);
//  if (IS_ENABLED(CONFIG_SAMPLE_ERROR_CHECKING) && ret != 0){
//    errno = ret;
//    //perror("pthread_mutex_lock");
//    __ASSERT(false,"failed to lock mutex");
//  }
//}
//inline void drop(mutex_t m){
//  int ret;
//  ret = pthread_mutex_unlock(m);
//  if (IS_ENABLED(CONFIG_SAMPLE_ERROR_CHECKING) && ret != 0){
//    errno = ret;
//    //perror("pthread_mutex_unlock");
//    __ASSERT(false, "failed to unlock mutex");}
//}


    
#endif
