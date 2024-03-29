#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>

#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <zephyr/sys/slist.h>

struct k_timer timer_updata;
struct k_timer timer_period;
static sys_slist_t data_block = SYS_SLIST_STATIC_INIT(&data_block);

LOG_MODULE_REGISTER(demo_version,LOG_LEVEL_INF);
static uint16_t modbus_registers[64];
static uint8_t __attribute__((unused)) modbus_coils;

#define PWM_HALF_USEC(x) 500*PWM_NSEC(x)

#define PERIOD_TIME 50000
#define DUMP_TIME PERIOD_TIME/5
#define UPDATE_TIME 5000

#define SET_LED 0x0000
#define MODBUS_SEVER_FC_TEMP  0x0001
#define MODBUS_SEVER_FC_PRESS 0x0005

#define READ_FLOWMETER_ADDR 0x0009

#define WRITE_REGISTER_SetFlow_LO 0x000F
#define WRITE_REGISTER_SetFlow_HI 0x0010
#define SET_FLOWMETER_ADDR 0x0011

#define MODBUS_SEVER_FC_SET_VSWITCH 0x0012
#define MODBUS_SEVER_FC_SET_SM 0x0013

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
static int client_iface;

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

static int coil_rd(uint16_t addr, bool* state){
  LOG_INF("todo");
  return 0;
}

static int coil_wr(uint16_t addr, bool state){
  LOG_INF("todo");

  return 0;
}


static int registers_read(uint16_t addr, uint16_t* reg){
  *reg = modbus_registers[addr];
  LOG_INF("registers_read, addr %u",addr);
  return 0;
}

static int trans_to_dump_mode(){
  //unable the pump
  int ec = pwm_set_pulse_dt(&servo_st,0);
  if (ec<0){
    k_msleep(30);
    if (pwm_set_pulse_dt(&servo_st,0)<0) return -1;
  }
  k_msleep(30);

  uint16_t float40[2] = {0,16928};
  ec = modbus_write_holding_regs(client_iface,1,0x006a,float40,2);
  if (ec != 0){
    k_msleep(200);
    ec = modbus_write_holding_regs(client_iface,1,0x006a,float40,2);
    if (ec!=0){
      gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_INACTIVE);
      return -2;
    }
  }

  k_msleep(200);
  ec = gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_INACTIVE);
  if (ec<0){
    k_msleep(30);
    if(gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_INACTIVE)<0) return -3;
  }
  return 0;
}

static int trans_to_recycle_mode(){
  int ec = gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_ACTIVE);
  if (ec<0){
    k_msleep(30);
    if (gpio_pin_configure_dt(&valve_switch,GPIO_OUTPUT_ACTIVE)<0) return -1;
  }
  k_msleep(30);

  ec = pwm_set_pulse_dt(&servo_st,PWM_HALF_USEC(16));
  if (ec<0){
    k_msleep(30);
    if (pwm_set_pulse_dt(&servo_st,PWM_HALF_USEC(13))<0) return -3;
  }

  k_msleep(1000);

  uint16_t float0[2] = {0,0};
  ec = modbus_write_holding_regs(client_iface,1,0x006a,float0,2);
  if (ec != 0){
    k_msleep(200);
    ec = modbus_read_holding_regs(client_iface,1,0x006a,float0,2);
    if (ec !=0) return -2; }
  return 0;

}

static void upload_data(struct k_timer* timer){
  LOG_INF("update data");
  struct sensor_value value;
  sensor_sample_fetch(bmp280);
  sensor_channel_get(bmp280,SENSOR_CHAN_AMBIENT_TEMP,&value);
  modbus_registers[2] = value.val1;
  modbus_registers[3] = (uint16_t)(value.val2>>16);
  modbus_registers[4] = (uint16_t)(value.val2 & 0xFFFF);
  k_msleep(25);
  sensor_channel_get(bmp280,SENSOR_CHAN_PRESS,&value);
  modbus_registers[6] = value.val1;
  modbus_registers[7] = (uint16_t)(value.val2>>16);
  modbus_registers[8] = (uint16_t)(value.val2 & 0xFFFF);
  k_msleep(25);
  modbus_read_holding_regs(client_iface,1,0x0010,modbus_registers+10,2);
  k_msleep(250);
  modbus_read_holding_regs(client_iface,1,0x001c,modbus_registers+12,2);
}

static void self_cycle(struct k_timer* timer){
  trans_to_dump_mode();
  k_msleep(DUMP_TIME);
  trans_to_recycle_mode();
}

static int self_cycle_mode(){

  LOG_INF("Poll Mode!");
  int ec = trans_to_dump_mode();
  k_msleep(2000);

  //for (int i=0; i<1000; ++i){
  //k_timer_start(&timer_period,K_MSEC(PERIOD_TIME),K_MSEC(PERIOD_TIME));
  //k_timer_start(&timer_updata,K_MSEC(UPDATE_TIME),K_MSEC(UPDATE_TIME));

  size_t i = 1;
  struct timeval tv;
  while(1){
    gettimeofday(&tv,NULL);
    LOG_INF("[%d %d %d] Period %d Start. slist size %d"
        ,(unsigned int)(tv.tv_sec>>32),(unsigned int)tv.tv_sec,(unsigned int)tv.tv_usec
        ,i++,sys_slist_len(&data_block));

    ec = trans_to_dump_mode();
    if (ec != 0){
      LOG_WRN("%d Cycle, Failed in 'trans_to_dump_mode'", i);
      break;
    }else LOG_INF("==> Dump Mode");

    for (int j=0; j<20; ++j){
      //sys_snode_t* node;
      //sys_slist_append(&data_block,node);

      //if (j==4){ 
      //  if (trans_to_recycle_mode()!=0){
      //    LOG_INF("%d Cycle, failed in 'trans_to_recycle_mode'",i);
      //    break;
      //  }else{
      //    LOG_INF("===> Recycle Mode");
      //    continue;
      //  }
      //}

      k_msleep(4500);
      upload_data(NULL);
      k_msleep(200);
      //ec = modbus_read_holding_regs(client_iface,1,0x0010,modbus_registers+10,2);
      //if (ec!=0){
      //  k_msleep(200);
      //  if (modbus_read_holding_regs(client_iface,1,0x0010,modbus_registers+10,2)!=0)
      //    modbus_registers[10] = modbus_registers[11] = 0xFFFF;
      //  continue;
      //}
      //k_msleep(500);
      //ec = modbus_read_holding_regs(client_iface,1,0x001c,modbus_registers+12,2);
      //if (ec!=0){
      //  k_msleep(200);
      //  if(modbus_read_holding_regs(client_iface,1,0x001c,modbus_registers+12,2)!=0)
      //    modbus_registers[12] = modbus_registers[13] = 0xFFFF;
      //  continue;
      //}
    }
  }
  return 0;
}
//---------------------------------------------------------------------
static int registers_write(uint16_t addr, uint16_t value){
  switch(addr){
    case(SET_LED):
      if (value==0) gpio_pin_configure_dt(&led,GPIO_OUTPUT_INACTIVE);
      else if (value==1) gpio_pin_configure_dt(&led,GPIO_OUTPUT_ACTIVE);
      else if (value==2){
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
      //pwm_set_pulse_dt(&servo_st,PWM_USEC(value/2));
      pwm_set_pulse_dt(&servo_st,PWM_HALF_USEC(value));
      k_msleep(50);
      //modbus_registers[19] = value;
      //if (pwm_set_pulse_dt(&servo_st,value/2)<0){
      //  LOG_ERR("servo_motor's pluse be set to %ld failed",PWM_USEC(value/2));
      //  return -1;
      //}
      //pwm_set_pulse_dt(&servo_st,50);
      LOG_INF("servo_motor's pluse be set to %d ok",PWM_HALF_USEC(value));
    break;
    default:
      LOG_INF("unknow command %d",addr);
      break;
  }
  return 0;
}

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
		.unit_id = 1,
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
  LOG_INF("found device %s [OK]",dev->name);
  return dev;
#endif
}
int main(void){

  for (size_t i=0; i<32; i++) modbus_registers[i] = 0x00;
  modbus_registers[10] = modbus_registers[11] = 0xFFFF;
  modbus_registers[12] = modbus_registers[13] = 0xFFFF;
  if (!gpio_is_ready_dt(&led)){
    LOG_ERR("gpio8 output led ready failed");
    return -1;
  }
  //LOG_INF("gpio8 output led ready ok");
  modbus_registers[14] |= 0b00000001;

  if (!gpio_is_ready_dt(&valve_switch)){
    LOG_ERR("gpio8 output valve_switch ready failed");
    return -1;
  }
  modbus_registers[14] |= 0b00000010;
  //LOG_INF("gpio8 output valve_switch ready ok");
  if (!pwm_is_ready_dt(&servo_st)){
    LOG_ERR("pwm device not ready: %s",servo_st.dev->name);
    return -1;
  }
  pwm_set_pulse_dt(&servo_st,0);
  //LOG_INF("pwm device ready: %s",servo_st.dev->name);
  modbus_registers[14] |= 0b00000100;

  bmp280 = get_bmp280_device();
  if (bmp280 == NULL){
    LOG_ERR("sensor initialization failed");
  }else{
    LOG_INF("sensor bmp280 initialization ok");
    modbus_registers[14] |= 0b00001000;
  }

  if (init_modbus_client()){
    LOG_ERR("modbus rtu client initialization failed"); 
    return -1;
  }
  modbus_registers[14] |= 0b00100000;
  //LOG_INF("modbus rtu client initialization ok");
  if (init_modbus_server()){ 
    LOG_ERR("modbus rtu server initialization failed");
    return -1;
  }
  //LOG_INF("modbus rtu server initialization ok");
  modbus_registers[14] |= 0b01000000;

  k_msleep(200);
  uint16_t digit_mode_command[2] = {0x0000,0x41D0};
  if (modbus_write_holding_regs(client_iface,1,0x0074,digit_mode_command,2)!=0){
    LOG_ERR("set flowmeter mode as 'digit"); 
    return -1;
  }
  modbus_registers[14] |= 0b10000000;
  LOG_INF("set flowmeter mode as 'digit'");


  k_timer_init(&timer_updata,&upload_data,NULL);
  k_timer_init(&timer_period,&self_cycle,NULL);
  LOG_INF("all is ok, blink the LED");
  gpio_pin_configure_dt(&led,GPIO_OUTPUT_ACTIVE);


  //struct timeval tv;
  //gettimeofday(&tv,NULL);
  ////time_t now = time(NULL);
  //LOG_INF("HI(tv_sec)=%d, LO(tv_sec)=%d, tv_usec=%d"
  //    ,(unsigned int)(tv.tv_sec>>32),(unsigned int)tv.tv_sec,(unsigned int)tv.tv_usec);

  //for (int i=0; i<10; ++i){
  //  LOG_INF("sys_slist_t: %d",sys_slist_len(&data_block));
  //  k_msleep(500);
  //  //sys_slist_peek_tail(sys_slist_t *list)
  //  sys_slist_append(&data_block,node);
  //}

  self_cycle_mode();

  return 0;

}
