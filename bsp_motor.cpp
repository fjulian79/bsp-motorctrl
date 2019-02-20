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

#include <bsp/bsp_gpio.h>
#include <bsp/bsp_tty.h>
#include <bsp/bsp_motor.h>
#include <bsp/bsp_assert.h>

#include <generic/generic.h>

#include <stm32f1xx_ll_tim.h>

/**
 * @brief Defines the controll pins for a single motor, see bspMotorDesc
 */
typedef struct 
{
   bspGpioPin_t Ctrl_1;
   bspGpioPin_t Ctrl_2;

}bspMotor_t;

/**
 * @brief Used to map a motor id to gpio pins.
 */
const bspMotor_t bspMotorDesc[BSP_NUM_MOTORS] = 
{
   {BSP_GPIO_MOTOR1_1, BSP_GPIO_MOTOR1_2},
   {BSP_GPIO_MOTOR2_1, BSP_GPIO_MOTOR2_2},
   {BSP_GPIO_MOTOR3_1, BSP_GPIO_MOTOR3_2},
   {BSP_GPIO_MOTOR4_1, BSP_GPIO_MOTOR4_2}
};

void bspMotorSetTimerCC(uint8_t id, uint32_t val)
{
   bspAssert(id < BSP_NUM_MOTORS);

   switch (id)
   {  
      case 0:
         LL_TIM_OC_SetCompareCH1(TIM3, val);
         break;

      case 1:
         LL_TIM_OC_SetCompareCH2(TIM3, val);
         break;

      case 2:
         LL_TIM_OC_SetCompareCH3(TIM3, val);
         break;

      case 3:
         LL_TIM_OC_SetCompareCH4(TIM3, val);
         break;

      default:
         break;
   }
}

void bspMotorInit(void)
{
   LL_TIM_InitTypeDef timInit;

   timInit.Autoreload = BSP_MOTORPWM_MAX - 1;
   timInit.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   timInit.CounterMode = TIM_COUNTERMODE_UP;
   timInit.Prescaler = 0;
   timInit.RepetitionCounter = 0;
   LL_TIM_Init(TIM3, &timInit);

   /* Configure the capture compare channels */
   LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
   LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
   bspMotorSetTimerCC(0, 0);

   LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
   LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
   bspMotorSetTimerCC(1, 0);

   LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
   LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
   bspMotorSetTimerCC(2, 0);

   LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);;
   LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
   bspMotorSetTimerCC(3, 0);

   /* Enable the timer */
   LL_TIM_EnableCounter(TIM3);   
}

void bspMotorSet(uint8_t id, int32_t val)
{
   bspAssert(id < BSP_NUM_MOTORS);

   val = constrain(val, -BSP_MOTORPWM_MAX, BSP_MOTORPWM_MAX);

   if (val == 0)
   {
      bspGpioClear(bspMotorDesc[id].Ctrl_1);
      bspGpioClear(bspMotorDesc[id].Ctrl_2);
      val = BSP_MOTORPWM_MAX;
   }
   else if (val > 0)
   {
      bspGpioClear(bspMotorDesc[id].Ctrl_1);
      bspGpioSet(bspMotorDesc[id].Ctrl_2);
   }
   else
   {
      bspGpioSet(bspMotorDesc[id].Ctrl_1);
      bspGpioClear(bspMotorDesc[id].Ctrl_2);
      val = -val;
   }

   bspMotorSetTimerCC(id, (uint32_t) val);
}

void bspMotorStop(uint8_t id)
{
   bspAssert(id < BSP_NUM_MOTORS);

   bspMotorSetTimerCC(id, 0);
   bspGpioSet(bspMotorDesc[id].Ctrl_1);
   bspGpioSet(bspMotorDesc[id].Ctrl_2);
}