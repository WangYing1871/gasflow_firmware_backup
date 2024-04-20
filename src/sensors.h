#ifndef sensors_H
#define sensors_H 1 
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
int get_bmp280_t(const struct device* bmp280,uint16_t*);
int get_bmp280_p(const struct device* bmp280,uint16_t*);
//int get_aht20_t(const struct device* bmp280,uint16_t*);
//int get_aht20_h(const struct device* bmp280,uint16_t*);
#endif
