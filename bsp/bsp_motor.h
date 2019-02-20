/*
 * bsp_motor.h
 *
 *  Created on: Feb 01, 2019
 *      Author: julian
 */

#ifndef BSP_MOTORCTRL_MOTOR_H_
#define BSP_MOTORCTRL_MOTOR_H_

#include <stdint.h>
#include <stdbool.h>

#include "bsp/bsp.h"

/**
 *  @brief Defines the number of supported motors. 
 */
#define BSP_NUM_MOTORS              4

/**
 * @brief Used to initialize the peripherals used to control the motors.
 * Hence, gipos are intialized in bspGpioInit()
 */
void bspMotorInit(void);

/**
* @brief to set the pwm value
* 
* @param val The new PWM value.
*/
void bspMotorSet(uint8_t id, int32_t val);

/**
* @brief To stop the motor by using the eddy break.
*/
void bspMotorStop(uint8_t id);

#endif /* BSP_MOTORCTRL_MOTOR_H_ */