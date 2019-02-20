/*
 * bsp-motorctrl, a board support package library for a STM32 based motor 
 * control PCB. It is designed to abstract access to HW features in a generic 
 * and simple way. Please note thet it should not conain any buissness logic.
 *
 * Copyright (C) 2019 Julian Friedrich
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>. 
 *
 * You can file issues at https://github.com/fjulian79/bsp-motorctrl
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