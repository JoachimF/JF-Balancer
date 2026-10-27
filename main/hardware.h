#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include <mpu6050.h>

#define ADDR 0x68

void config_mpu(mpu6050_dev_t *dev1,mpu6050_dev_t *dev2) ;

#endif