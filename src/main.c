#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <stdio.h>

#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(GFC,LOG_LEVEL_INF);
#include "posix_internal.h"
#include <pthread.h>

#define TASK_NUM 4

#define TASK_ID_SUBMIT_DATA 0
static bool is_submit_data = false;
typedef pthread_mutex_t* mutex_t;
static pthread_mutex_t mutexs[TASK_NUM];
static pthread_t threads[TASK_NUM];

static void mutex_init(mutex_t m){
  int ret;
  ret = pthread_mutex_init(m,NULL);
  if (IS_ENABLED(CONFIG_SAMPLE_ERROR_CHECKING) && ret != 0){
    errno = ret;
    perror("pthread_mutex_init");
    __ASSERT(false,"failed to initialize fork");
  }
}

static void init_mutexs(void){
  ARRAY_FOR_EACH(mutexs,i){
    LOG_INF("initialization mutex_t %zu",i); 
    mutex_init(&mutexs[i]);
  }
}

static inline void take(mutex_t m){
  int ret;
  ret = pthread_mutex_lock(m);
  if (IS_ENABLED(CONFIG_SAMPLE_ERROR_CHECKING) && ret != 0){
    errno = ret;
    perror("pthread_mutex_lock");
    __ASSERT(false,"failed to lock mutex");
  }
}
static inline void drop(mutex_t m){
  int ret;
  ret = pthread_mutex_unlock(m);
  if (IS_ENABLED(CONFIG_SAMPLE_ERROR_CHECKING) && ret != 0){
    errno = ret;
    perror("pthread_mutex_unlock");
    __ASSERT(false, "failed to unlock mutex");}
}


static uint16_t modbus_registers[64];
static uint8_t __attribute__((unused)) modbus_coils;

#define MY_STACK_SIZE 512
#define MY_PRIORITY 5
K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);
static struct k_work_q my_work_q;



struct period_task{
  struct k_timer timer;
  struct k_work work;
  k_work_handler_t task;
  struct k_work_q* work_queue;
};

extern void period_task_adaptor(struct k_timer* T){
  struct period_task* task = CONTAINER_OF(T,struct period_task,timer);
  if (task->work_queue==NULL)
    k_work_submit(&task->work);
  else
    k_work_submit(&task->work);
    //k_work_submit_to_queue(task->work_queue,&task->work);
}

void init_pt(struct period_task* T,k_work_handler_t handler,struct k_work_q* wq){
  k_timer_init(&T->timer,period_task_adaptor,NULL);
  T->task = handler;
  //T->work_queue = wq;
  k_work_init(&T->work,T->task);
}

void start_pt(struct period_task* T,k_timeout_t duration,k_timeout_t period){
  k_timer_start(&T->timer,duration,period);
}


struct k_timer timer_updata;
struct k_timer timer_period;


#define PWM_HALF_USEC(x) 500*(x)

#define MFC_MODBUS_RTU_ID 1
#define SET_LED 0x0000
#define MODBUS_SEVER_FC_TEMP  0x0001
#define MODBUS_SEVER_FC_PRESS 0x0005

#define READ_FLOWMETER_ADDR 0x0009

#define DEVIES_STATE_BITS 0x000E
#define WRITE_REGISTER_SetFlow_LO 0x000F
#define WRITE_REGISTER_SetFlow_HI 0x0010
#define SET_FLOWMETER_ADDR 0x0011

#define MODBUS_SEVER_FC_SET_VSWITCH 0x0012
#define MODBUS_SEVER_FC_SET_SM 0x0013
#define SET_PERIOD_DUMP 0x0014
#define SET_PERIOD_RECYCLE 0x0015
#define SET_MFC_PV_LO 0x0016
#define SET_MFC_PV_HI 0x0017

#define READ_CURRENT_FLOW 0x0000
#define READ_TOTAL_FLOW 0x0001
#define SET_FLOW 0x0000
#define RESET_TOTAL_FLOW 0x0001
#define GPIO8_HIGH 0x0001
#define GPIO8_LOW  0x0000

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);



#if !DT_NODE_EXISTS(DT_NODELABEL(valve_switch))
# error "compiler: device node not define"
#endif
#if !DT_NODE_EXISTS(DT_ALIAS(servo0))
# error "compiler: define node <servo0> not define"
#else
# define SEVER0_NODE DT_ALIAS(servo0)
#endif
static const struct pwm_dt_spec servo_st = PWM_DT_SPEC_GET(SEVER0_NODE);
static const uint32_t __attribute__((unused)) min_pulse = DT_PROP(SEVER0_NODE,min_pulse);
static const uint32_t __attribute__((unused)) max_pulse = DT_PROP(SEVER0_NODE,max_pulse);

static const struct gpio_dt_spec valve_switch = 
  GPIO_DT_SPEC_GET_OR(DT_NODELABEL(valve_switch),gpios,{0});

static const struct device* bmp280;
static const struct device* aht20;
static int client_iface;

struct sensors_msg{
  uint16_t data[32];
};

static struct sensors_msg monitor = {0};
#include <zephyr/zbus/zbus.h>
ZBUS_CHAN_DECLARE(sensors_data_update);
void* peripheral_thread(void* arg){
  int my_id = POINTER_TO_INT(arg);
  struct sensors_msg sm = {0};
  take(&mutexs[my_id]);
  while(is_submit_data){
    struct sensor_value value;
    sensor_sample_fetch(bmp280);
    sensor_channel_get(bmp280,SENSOR_CHAN_AMBIENT_TEMP,&value);
    monitor.data[0] = value.val1;
    monitor.data[1] = (uint16_t)(value.val2>>16);
    monitor.data[2] = (uint16_t)(value.val2 & 0xFFFF);
    k_msleep(10);
    sensor_channel_get(bmp280,SENSOR_CHAN_PRESS,&value);
    monitor.data[3] = value.val1;
    monitor.data[4] = (uint16_t)(value.val2>>16);
    monitor.data[5] = (uint16_t)(value.val2 & 0xFFFF);
    k_msleep(30);
    modbus_read_holding_regs(client_iface,1,0x0010,monitor.data+11,2);

    //LOG_INF("my_thread");
    //LOG_INF("my thread. bmp: %d",POINTER_TO_INT(bmp280));


    //for (int i=0; i<3; ++i) sm.temprature[i]++;
    //for (int i=0; i<3; ++i) sm.pressure[i]++;
    //for (int i=0; i<3; ++i) sm.humidity[i]++;
    //zbus_chan_pub(&sensors_data_update,&sm,K_MSEC(200));
    k_msleep(300);
  }
  drop(&mutexs[my_id]);
  return NULL;
}
//K_THREAD_DEFINE(peripheral_thread_id,4096,peripheral_thread,NULL,NULL,NULL,5,0,0);



struct sensor_wq_info{
  struct k_work work;
  const struct zbus_channel* chan;
  uint8_t handle;
};
static struct sensor_wq_info wq_handler1 = {.handle = 1};
static struct sensor_wq_info wq_handler2 = {.handle = 2};
static struct sensor_wq_info wq_handler3 = {.handle = 3};
static struct sensor_wq_info wq_handler4 = {.handle = 4};

static void wq_dh_cb1(struct k_work* item){
  struct sensors_msg msg;
  struct sensor_wq_info* sens = 
    CONTAINER_OF(item,struct sensor_wq_info,work);
  zbus_chan_read(sens->chan,&msg,K_MSEC(200));
  LOG_INF("sen handle(T) %d %d %d %d",sens->handle
      ,msg.data[0] ,msg.data[1] ,msg.data[2]); }
static void wq_dh_cb2(struct k_work* item){
  struct sensors_msg msg;
  struct sensor_wq_info* sens = 
    CONTAINER_OF(item,struct sensor_wq_info,work);
  zbus_chan_read(sens->chan,&msg,K_MSEC(200));
  LOG_INF("sen handle(P) %d %d %d %d",sens->handle
      ,msg.data[3] ,msg.data[4] ,msg.data[5]); }
static void wq_dh_cb3(struct k_work* item){
  struct sensors_msg msg;
  struct sensor_wq_info* sens = 
    CONTAINER_OF(item,struct sensor_wq_info,work);
  zbus_chan_read(sens->chan,&msg,K_MSEC(200));
  LOG_INF("sen handle(MFC-PV) %d %d %d",sens->handle
      ,msg.data[11],msg.data[12]); }

static void wq_dh_cb4(struct k_work* item){
  struct sensors_msg msg;
  struct sensor_wq_info* sens = 
    CONTAINER_OF(item,struct sensor_wq_info,work);
  zbus_chan_read(sens->chan,&msg,K_MSEC(200));
  for (int i=0; i<ARRAY_SIZE(monitor.data); ++i)
    monitor.data[i] = msg.data[i];

}

static void dh1_cb(const struct zbus_channel* chan){
  wq_handler1.chan = chan;
  k_work_submit(&wq_handler1.work); }
static void dh2_cb(const struct zbus_channel* chan){
  wq_handler2.chan = chan;
  k_work_submit(&wq_handler2.work); }
static void dh3_cb(const struct zbus_channel* chan){
  wq_handler3.chan = chan;
  k_work_submit(&wq_handler3.work); }
static void dh4_cb(const struct zbus_channel* chan){
  wq_handler4.chan = chan;
  k_work_submit(&wq_handler4.work); }

ZBUS_LISTENER_DEFINE(delay_handler1_lis,dh1_cb);
ZBUS_LISTENER_DEFINE(delay_handler2_lis,dh2_cb);
ZBUS_LISTENER_DEFINE(delay_handler3_lis,dh3_cb);
ZBUS_LISTENER_DEFINE(delay_handler4_lis,dh4_cb);
ZBUS_CHAN_DEFINE(sensors_data_update
    ,struct sensors_msg
    ,NULL
    ,NULL
    ,ZBUS_OBSERVERS(
      delay_handler1_lis
      ,delay_handler2_lis
      ,delay_handler3_lis
      ,delay_handler4_lis
      )
    ,ZBUS_MSG_INIT(0));



const static struct modbus_iface_param client_accu_iface = {
  .mode = MODBUS_MODE_RTU,
  .rx_timeout = 500000,
  .serial = {
    .baud = 115200,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits_client = UART_CFG_STOP_BITS_1
  }
};

#define MODBUS_CLIENT_NODE DT_ALIAS(modbus_client)
static int init_modbus_client(void){
  char const iface_name[] = {DEVICE_DT_NAME(MODBUS_CLIENT_NODE)};
  LOG_INF("init_modbus_client: iface_name %s",iface_name);
  client_iface = modbus_iface_get_by_name(iface_name);
  LOG_INF("init_modbus_client: client_iface: %d",client_iface);

  return modbus_init_client(client_iface,client_accu_iface);
}

enum run_state{
  k_debug
  ,k_run
};

static enum run_state rs = k_debug;

static int coil_rd(uint16_t addr, bool* state){
  LOG_INF("todo");
  return 0;
}

static int coil_wr(uint16_t addr, bool state){
  LOG_INF("todo");

  return 0;
}


static int registers_read(uint16_t addr, uint16_t* reg){
  if (rs==k_debug)
    *reg = modbus_registers[addr];
  else if(rs==k_run)
    *reg = monitor.data[addr];
  //LOG_INF("registers_read, addr %u",addr);
  return 0;
}

static int registers_write(uint16_t addr, uint16_t value);

static struct modbus_user_callbacks mbs_cbs = {
  .coil_rd = coil_rd,
  .coil_wr = coil_wr,
  .holding_reg_rd = registers_read,
  .holding_reg_wr = registers_write
};
const static struct modbus_iface_param server_param = {
	.mode = MODBUS_MODE_RTU,
	.server = {
		.user_cb = &mbs_cbs,
		.unit_id = MFC_MODBUS_RTU_ID,
	},
	.serial = {
		.baud = 19200,
		.parity = UART_CFG_PARITY_NONE,
    .stop_bits_client = UART_CFG_STOP_BITS_2
	}
};

#define MODBUS_SERVER_NODE DT_ALIAS(modbus_server)
static int init_modbus_server(void){
  const char iface_name[] = {DEVICE_DT_NAME(MODBUS_SERVER_NODE)};
  LOG_INF("init_modbus_server: iface_name %s",iface_name);
  int iface = modbus_iface_get_by_name(iface_name);
  if (iface<0){
    LOG_ERR("failed to get iface index for %s",iface_name);
    return iface; }
  return modbus_init_server(iface,server_param); }

static const struct device* get_bmp280_device(void){
#if !DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
# pragma message "i2c1 node has been unable, sensor unable too"
  LOG_INF("i2c1 unable");
  return NULL;
#else
  const struct device* const dev = DEVICE_DT_GET_ANY(bosch_bme280);
  if (dev==NULL){
    LOG_ERR("sensor get failed");
    return NULL; }
  if (!device_is_ready(dev)){
    LOG_ERR("device %s is not ready",dev->name);
    return NULL; }
  return dev;
#endif
}
static const struct device* get_aht20_device(void){
#if !DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1),okay)
# pragma message "i2c1 node has been unable, sensor unable too"
  LOG_INF("i2c1 unable");
  return NULL;
#else
  const struct device* const dev = DEVICE_DT_GET_ANY(aosong_aht20);
  //if (dev==NULL){
  //  LOG_ERR("sensor get failed");
  //  return NULL; }
  //if (!device_is_ready(dev)){
  //  LOG_ERR("device %s is not ready",dev->name);
  //  return NULL; }
  return dev;
#endif
}



//---------------------------------------------------------------------
static void* trans_to_recycle_mode(void* arg){
  int ec = gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_ACTIVE);
  if (ec<0){
    k_msleep(30);
    if (gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_ACTIVE)<0) return INT_TO_POINTER(-1);
  }
  k_msleep(30);
  ec = pwm_set_pulse_dt(&servo_st,PWM_HALF_USEC(14));
  if (ec<0){
    k_msleep(30);
    if (pwm_set_pulse_dt(&servo_st,PWM_HALF_USEC(14))<0) return INT_TO_POINTER(-2)
  }
  k_msleep(200);
  uint16_t float0[2] = {0,0};
  ec = modbus_write_holding_regs(client_iface,1,0x006a,float0,2);
  if (ec != 0){
    k_msleep(200);
    ec = modbus_read_holding_regs(client_iface,1,0x006a,float0,2);
    if (ec !=0) return INT_TO_POINTER(-3); }
  return NULL; }

static void* trans_to_dump_mode(void* arg){
  int ec = pwm_set_pulse_dt(&servo_st,0);
  if (ec<0){
    k_msleep(30);
    if (pwm_set_pulse_dt(&servo_st,0)<0) return INT_TO_POINTER(-1);
  }
  k_msleep(30);

  uint16_t float_toset[2] = {
    modbus_registers[SET_MFC_PV_LO]
    ,modbus_registers[SET_MFC_PV_HI]
  }
  ec = modbus_write_holding_regs(client_iface,1,0x006a,float_toset,2);
  if (ec != 0){
    k_msleep(200);
    ec = modbus_write_holding_regs(client_iface,1,0x006a,float_toset,2);
    if (ec!=0){
      gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_INACTIVE);
      return INT_TO_POINTER(-2);
    }
  }

  k_msleep(200);
  ec = gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_INACTIVE);
  if (ec<0){
    k_msleep(30);
    if(gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_INACTIVE)<0) return INT_TO_POINTER(-3);
  }
  return NULL;
}

//TODO
//static self_cycle_mode(){
//
//  LOG_INF("self cycle mode!");
//  rs = k_run;
//
//  size_t i = 1;
//  while(1){
//    int ec;
//    uint16_t dt = modbus_registers[SET_PERIOD_DUMP];
//    uint16_t rt = modbus_registers[SET_PERIOD_RECYCLE];
//    LOG_INF("period %d start, dump: %d, recycle: %d" ,i++,dt,rt);
//
//    if (dt==0){
//    }else{
//      ec = trans_to_dump_mode();
//      if (ec != 0){
//        LOG_WRN("%d loop, Failed in 'trans_to_dump_mode'", i);
//        continue;
//      }
//      k_msleep(dt*1000);
//    }
//    if(rt==0){
//    }else{
//      ec = trans_to_recycle_mode();
//      if (ec != 0){
//        LOG_WRN("%d loop, Failed in 'trans_to_recycle_mode'", i);
//        continue;
//      }
//      k_msleep(rt*1000);
//    }
//  }
//  return 0;
//}

//TODO //XXX
typedef void*(*task_t)(void*);
struct poll_task{
  k_timepoint_t m_interval_t0;
  k_timepoint_t m_interval_t1;
  struct k_timer m_timer0;
  struct k_timer m_timer1;
  task_t m_task0;
  task_t m_task1;
};
#define FIRST_TASK_ID 1
#define SECOND_TASK_ID 2

void l_poll_task_adapter0(struct k_timer* t){
  struct poll_task* pt = CONTAINER_OF(t,struct poll_task,m_timer0);
  task_t task_0 = pt->m_task0;
  pthread_create(&threads[FIRST_TASK_ID],NULL
      ,task_0,INT_TO_POINTER(FIRST_TASK_ID));
  k_timer_stop(&t);
  struct k_timer* ot = pt->m_timer1;
  k_timer_start(pt->m_timer1,K_NO_WAIT,K_MSEC(pt->m_interval_t1));
}

void l_poll_task_adapter1(struct k_timer* t){
  struct poll_task* pt = CONTAINER_OF(t,struct poll_task,m_timer1);
  task_t task_1 = pt->m_task1;
  pthread_create(&threads[SECOND_TASK_ID],NULL
      ,task_1,INT_TO_POINTER(SECOND_TASK_ID));
  k_timer_stop(&t);
  struct k_timer* ot = pt->m_timer0;
  k_timer_start(pt->m_timer0,K_NO_WAIT,K_MSEC(pt->m_interval_t0));
}


//extern void period_task_adaptor(struct k_timer* T){
//  struct period_task* task = CONTAINER_OF(T,struct period_task,timer);
//  if (task->work_queue==NULL)
//    k_work_submit(&task->work);
//  else
//    k_work_submit(&task->work);
//    //k_work_submit_to_queue(task->work_queue,&task->work);
//}


int l_poll_task_init(struct poll_task* pt
    ,task_t ta
    ,task_t tb
    ,k_timeout_t a
    ,k_timeout_t b){

  k_timer_init(&pt->m_timer0,l_poll_task_adapter0,NULL)
  k_timer_init(&pt->m_timer1,l_poll_task_adapter1,NULL)
  return 0;
}

int l_poll_task_start(struct poll_task* pt, int inlet){
  switch(inlet){
    case(0):{
      k_timer_start(pt->m_timer0,K_NO_WAIT,pt->m_interval_t0);
      break;
      return 0;
    }
    case(1):{
      k_timer_start(pt->m_timer1,K_NO_WAIT,pt->m_interval_t1);
      break;
      return 0;
    }
    case(1):{
      LOG_WRN("INTEL id %d not now",inter);
      return -1;
    }
  }
  return 0;
}
static struct poll_task self_cycle_mode = {
  .m_task0 = trans_to_dump_mode
  ,.m_task1 = trans_to_recycle_mode
  ,.m_interval_t0 = 30000
  ,.m_interval_t1 = 30000
};

//---------------------------------------------------------------------
void AAA_WK(struct k_work* item){
  struct timeval tv;
  gettimeofday(&tv,NULL);
  LOG_INF("AAA_WK %d %d %d, thread: %d"
      ,(unsigned int)(tv.tv_sec>>32),(unsigned int)tv.tv_sec,(unsigned int)tv.tv_usec
      ,pthread_self()); }
void BBB_WK(struct k_work* item){
  struct timeval tv;
  gettimeofday(&tv,NULL);
  LOG_INF("BBB_WK %d %d %d, thread: %d"
      ,(unsigned int)(tv.tv_sec>>32),(unsigned int)tv.tv_sec,(unsigned int)tv.tv_usec
      ,pthread_self()); }
void CCC_WK(struct k_work* item){
  struct timeval tv;
  gettimeofday(&tv,NULL);
  LOG_INF("CCC_WK %d %d %d, thread: %d"
      ,(unsigned int)(tv.tv_sec>>32),(unsigned int)tv.tv_sec,(unsigned int)tv.tv_usec
      ,pthread_self()); }
//---------------------------------------------------------------------
enum run_mode{
  k_dump
  ,k_cycle
  ,k_poll
  ,k_unknow
};

static enum run_mode e_rm = run_mode::k_dump;


static void init_registers(){
  for (size_t i=0; i<32; i++) modbus_registers[i] = 0x00;
  modbus_registers[10] = modbus_registers[11] = 0xFFFF;
  modbus_registers[12] = modbus_registers[13] = 0xFFFF;
  modbus_registers[SET_MFC_PV_LO] = 0;
  modbus_registers[SET_MFC_PV_HI] = 16880;
  modbus_registers[SET_PERIOD_DUMP] = 30;
  modbus_registers[SET_PERIOD_RECYCLE] = 30; }

static void link_devices(){
  modbus_registers[DEVIES_STATE_BITS] = 0;
  if (gpio_is_ready_dt(&led))
    modbus_registers[DEVIES_STATE_BITS] |= 0x0001
  if (gpio_is_ready_dt(&valve_switch))
    modbus_registers[DEVIES_STATE_BITS] |= 0x0002
  if (pwm_is_ready_dt(&servo_st)){
    pwm_set_pulse_dt(&servo_st,0);
    modbus_registers[DEVIES_STATE_BITS] |= 0x0004; }
  bmp280 = get_bmp280_device();
  if (bmp280 != NULL)
    modbus_registers[DEVIES_STATE_BITS] |= 0x0008;
  aht20 = get_aht20_device();
  if (aht20 !=NULL)
    modbus_registers[DEVIES_STATE_BITS] |= 0x0010;
  if (!init_modbus_client()){
    modbus_registers[DEVIES_STATE_BITS] |= 0x0020;
    k_msleep(2000);
    uint16_t digit_mode_command[2] = {0x0000,0x41D0};
    if (modbus_write_holding_regs(client_iface,1,0x0074,digit_mode_command,2)!=0){
      k_msleep(2000);
      if (modbus_write_holding_regs(client_iface,1,0x0074,digit_mode_command,2)==0)
        modbus_register[DEVIES_STATE_BITS] |= 0x0040;
    }else modbus_register[DEVIES_STATE_BITS] |= 0x0040;
  }
}
static init_wq(){
  k_work_init(&wq_handler1.work,wq_dh_cb1);
  k_work_init(&wq_handler2.work,wq_dh_cb2);
  k_work_init(&wq_handler3.work,wq_dh_cb3);
  k_work_init(&wq_handler4.work,wq_dh_cb4);
}

static void exit_(){

}

static void restart(){
  init_registers();
  init_mutexs();
  init_wq();
  link_devices();
  if(modbus_register[DEVIES_STATE_BITS]==0x007F){
    LOG_INF("all is ok, blink the LED");
    gpio_pin_configure_dt(&led,GPIO_OUTPUT_ACTIVE); }
  trans_to_dump_mode(NULL);
  e_rm = run_mode::k_dump;
}
 

static int registers_write(uint16_t addr, uint16_t value){
  switch(addr){
    case(SET_LED):
      if (value==0){
        exit_();
        gpio_pin_configure_dt(&led,GPIO_OUTPUT_INACTIVE);
      }
      else if (value==1){
        gpio_pin_configure_dt(&led,GPIO_OUTwUT_INACTIVE);
        restart();
      }
      else if (value==2) rs==k_debug;
      else if (value==3) rs==k_run;
      else if (value==4){
        take(&mutexs[1]);
        is_submit_data = true;
        pthread_create(&threads[TASK_ID_SUBMIT_DATA],NULL
            ,peripheral_thread,INT_TO_POINTER(TASK_ID_SUBMIT_DATA));
        drop(&mutexs[1]);
      }else if(value==5){
        take(&mutexs[1]);
        is_submit_data = false;
        drop(&mutexs[1]);
      }else if(value==6){
        trans_to_dump_mode(NULL);
        e_rm = run_mode::k_dump;
      }else if(value==7){
        trans_to_dump_mode(NULL);
      }
      modbus_registers[0] = value;
      break;
    case(MODBUS_SEVER_FC_TEMP):
      struct sensor_value value2;
      sensor_sample_fetch(bmp280);
      sensor_channel_get(bmp280,SENSOR_CHAN_AMBIENT_TEMP,&value2);
      modbus_registers[2] = value2.val1;
      modbus_registers[3] = (uint16_t)(value2.val2>>16);
      modbus_registers[4] = (uint16_t)(value2.val2 & 0xFFFF);
      LOG_INF("senso: temp: %d.%d",value2.val1,value2.val2);
      break;
    case(MODBUS_SEVER_FC_PRESS):
      struct sensor_value value1;
      sensor_sample_fetch(bmp280);
      sensor_channel_get(bmp280,SENSOR_CHAN_PRESS,&value1);
      modbus_registers[6] = value1.val1;
      modbus_registers[7] = (uint16_t)(value1.val2>>16);
      modbus_registers[8] = (uint16_t)(value1.val2 & 0xFFFF);
      LOG_INF("sensor press: %d.%d",value1.val1,value1.val2);
      break;
    case(READ_FLOWMETER_ADDR):
      if (value==READ_CURRENT_FLOW){
        int ecode = 
          modbus_read_holding_regs(client_iface,1,0x0010,modbus_registers+10,2);
        //ecode = 
        //  modbus_read_holding_regs(client_iface,1,0x0010,modbus_registers+10,2);
        if (ecode!=0){ 
          LOG_ERR("current read failed");
          return -1;
        }
        LOG_INF("read current flow ok");
        LOG_HEXDUMP_INF(modbus_registers+10,2,"moment ...");}
      else if (value==READ_TOTAL_FLOW){
        int ecode = 
          modbus_read_holding_regs(client_iface,1,0x001c,modbus_registers+12,2);
        if (ecode!=0){
          LOG_ERR("total read failed");
          return -1;
        }
        LOG_INF("read total flow ok");
        LOG_HEXDUMP_INF(modbus_registers+12,2,"sustain ...");
      }
      break;
    
    case (SET_FLOWMETER_ADDR):
      if (value==RESET_TOTAL_FLOW){
        uint16_t zero_float[2] = {0,0};
        int ec = modbus_write_holding_regs( client_iface,1,0x001c,zero_float,2);
        if (ec!=0){ 
          LOG_ERR("reset total flow failed");
          return -1;
        }
        LOG_INF("reset total flow ok");
      }else if(value==SET_FLOW){
        //LOG_INF("inter: SET_FLOW !!");

        int ec = modbus_write_holding_regs(client_iface,1,0x006a,modbus_registers+15,2);

        if (ec != 0){
          LOG_ERR("set flow failed");
          return -1;
        }
        LOG_INF("set flow ok"); 
      }
      break;
    case(WRITE_REGISTER_SetFlow_LO):
      LOG_INF("write_register_setflow_lo: %d",value);
      modbus_registers[15] = value;
      break;
    case(WRITE_REGISTER_SetFlow_HI):
      LOG_INF("write_register_setflow_hi: %d",value);
      modbus_registers[16] = value;
      break;
    case(MODBUS_SEVER_FC_SET_VSWITCH):
      if (value==GPIO8_HIGH){
        if (gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_INACTIVE)<0){
          LOG_ERR("valve_switch set to 'INACTIVE' failed");
          return -1;
        }
        LOG_INF("valve_switch switch to 'INACTIVE' ok");
        //modbus_registers[18] = value;
      }
      else if(value==GPIO8_LOW){
        if (gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_ACTIVE)<0){
          LOG_ERR("valve_switch set to 'ACTIVE' failed");
          return -1;
        }
        LOG_INF("valve_switch switch to 'ACTIVE' ok");
        //modbus_registers[18] = value;
      }
      break;
    case(MODBUS_SEVER_FC_SET_SM):
      LOG_INF("MODBUS_SEVER_FC_SET_SM, value: %d",value);
      pwm_set_pulse_dt(&servo_st,PWM_USEC(value/2));
      //modbus_registers[19] = value;
      //if (pwm_set_pulse_dt(&servo_st,value/2)<0){
      //  LOG_ERR("servo_motor's pluse be set to %ld failed",PWM_USEC(value/2));
      //  return -1;
      //}
      //pwm_set_pulse_dt(&servo_st,50);
      LOG_INF("servo_motor's pluse be set to %ld ok",PWM_USEC(value/2));
      break;
    case(SET_PERIOD_DUMP):
    case(SET_PERIOD_RECYCLE):
      modbus_registers[addr] = value;
      break;
    default:
      LOG_INF("unknow command %d",addr);
      break;
  }
  return 0;
}

int main(void){
  int ec = init_modbus_server();
  if (ec != 0){
    LOG_ERR("Fatal ERROR, modbus-rtu-server init failed");
    return 0; }
  LOG_INF("modbus-rtu-server init ok");
  restart();
  return 0;
}

