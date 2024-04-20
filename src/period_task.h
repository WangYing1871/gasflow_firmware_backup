#ifndef period_task_H
#define period_task_H 1 
#include <zephyr/kernel.h>
struct period_task{
  uint32_t m_interval_td;
  uint32_t m_interval_tp;
  struct k_timer m_timer;
  struct k_work work;
  k_work_handler_t task;
  struct k_work_q* work_queue;
};

void period_task_adaptor(struct k_timer*);
void l_init_period_task(struct period_task*);
void l_start_period_task(struct period_task*);
void l_stop_period_task(struct period_task*);

#endif
