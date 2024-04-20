#include "sensors.h"

int get_bmp280_t(const struct device* d, uint16_t* data){
  sensor_sample_fetch(d);
  struct sensor_value value;
  int ec = sensor_channel_get(d,SENSOR_CHAN_AMBIENT_TEMP,&value);
  if (ec!=0) return ec;
  data[0] = value.val1;
  data[1] = (uint16_t)(value.val2>>16);
  data[2] = (uint16_t)(value.val2 & 0xFFFF);
  return 0;
}

int get_bmp280_p(const struct device* d, uint16_t* data){
  sensor_sample_fetch(d);
  struct sensor_value value;
  int ec = sensor_channel_get(d,SENSOR_CHAN_PRESS,&value);
  if (ec!=0) return ec;
  data[0] = value.val1;
  data[1] = (uint16_t)(value.val2>>16);
  data[2] = (uint16_t)(value.val2 & 0xFFFF);
  return 0;
}

//int get_aht20_t(const struct device* d, uint16_t* data){
//  sensor_sample_fetch(d);
//  struct sensor_value value;
//  int ec = sensor_channel_get(d,SENSOR_CHAN_AMBIENT_TEMP,&value);
//  if (ec!=0) return ec;
//  data[0] = value.val1;
//  data[1] = (uint16_t)(value.val2>>16);
//  data[2] = (uint16_t)(value.val2 & 0xFFFF);
//  return 0;
//}
//
//int get_aht20_h(const struct device* d, uint16_t* data){
//  sensor_sample_fetch(d);
//  struct sensor_value value;
//  int ec = sensor_channel_get(d,SENSOR_CHAN_HUMIDITY,&value);
//  if (ec!=0) return ec;
//  data[0] = value.val1;
//  data[1] = (uint16_t)(value.val2>>16);
//  data[2] = (uint16_t)(value.val2 & 0xFFFF);
//  return 0;
//}
