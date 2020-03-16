#ifndef SENSOR_BS_BMP280_H__
#define SENSOR_BS_BMP280_H__

#include "sensor.h"
#include "bmp280.h"
#define BMP280_ADDR_DEFAULT BMP280_I2C_ADDR_SEC
#define BMP280_I2CBUS_NAME "i2c2"
int rt_hw_bmp280_init(const char *name, struct rt_sensor_config *cfg);
#endif

