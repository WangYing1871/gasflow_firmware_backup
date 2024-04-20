#include "period_task.h"
void period_task_adaptor(struct k_timer* pt){
  struct period_task* task = 
    CONTAINER_OF(pt,struct period_task,m_timer);
  if (task->work_queue==NULL) k_work_submit(&task->work);
  else k_work_submit_to_queue(task->work_queue,&task->work); }

void l_init_period_task(struct period_task* pt){
  k_work_init(&pt->work,pt->task);
  k_timer_init(&pt->m_timer,period_task_adaptor,NULL); }

void l_start_period_task(struct period_task* pt){
  k_timer_start(&pt->m_timer
      ,K_MSEC(pt->m_interval_td),K_MSEC(pt->m_interval_tp)); }

void l_stop_period_task(struct period_task* pt){
  k_timer_stop(&pt->m_timer);
}
