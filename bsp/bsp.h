/*
 * bsp-motorctrl, a board support package library for a STM32 based motor 
 * control PCB. It is designed to abstract access to HW features in a generic 
 * and simple way. Please note thet it should not conain any buissness logic.
 *
 * Copyright (C) 2020 Julian Friedrich
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

#ifndef BSP_MOTORCTRL_H_
#define BSP_MOTORCTRL_H_

#include "bsp_config.h"

#include <stm32f1xx.h>
#include <stm32f1xx_ll_system.h>
#include <stm32f1xx_ll_utils.h>

/**
 * @brief Definition of used GPIO lines.
 */
#define BSP_GPIO_A5                         BSP_GPIO_LED
#define BSP_GPIO_C13                        BSP_GPIO_BUTTON

#define BSP_GPIO_C1                         BSP_GPIO_LIPO1
#define BSP_GPIO_C2                         BSP_GPIO_LIPO2
#define BSP_GPIO_C3                         BSP_GPIO_LIPO3

#define BSP_GPIO_C6                         BSP_GPIO_MOTOR1_PWM
#define BSP_GPIO_B12                        BSP_GPIO_MOTOR1_1
#define BSP_GPIO_B13                        BSP_GPIO_MOTOR1_2
#define BSP_GPIO_A0                         BSP_GPIO_MOTOR1_ENC_1
#define BSP_GPIO_A4                         BSP_GPIO_MOTOR1_ENC_2

#define BSP_GPIO_C7                         BSP_GPIO_MOTOR2_PWM
#define BSP_GPIO_B14                        BSP_GPIO_MOTOR2_1
#define BSP_GPIO_B15                        BSP_GPIO_MOTOR2_2
#define BSP_GPIO_A1                         BSP_GPIO_MOTOR2_ENC_1
#define BSP_GPIO_A6                         BSP_GPIO_MOTOR2_ENC_2

#define BSP_GPIO_C8                         BSP_GPIO_MOTOR3_PWM
#define BSP_GPIO_A8                         BSP_GPIO_MOTOR3_1
#define BSP_GPIO_A11                        BSP_GPIO_MOTOR3_2
#define BSP_GPIO_B10                        BSP_GPIO_MOTOR3_ENC_1
#define BSP_GPIO_B2                         BSP_GPIO_MOTOR3_ENC_2

#define BSP_GPIO_C9                         BSP_GPIO_MOTOR4_PWM
#define BSP_GPIO_A12                        BSP_GPIO_MOTOR4_1
//#define BSP_GPIO_A15                        BSP_GPIO_MOTOR4_2 /* damaged on eval board */
#define BSP_GPIO_C12                        BSP_GPIO_MOTOR4_2
#define BSP_GPIO_B11                        BSP_GPIO_MOTOR4_ENC_1
#define BSP_GPIO_A7                         BSP_GPIO_MOTOR4_ENC_2

/**
 * @brief Definition of analog input lines.
 */
#define BSP_ADCCH_LIPO1                     LL_ADC_CHANNEL_11
#define BSP_ADCCH_LIPO2                     LL_ADC_CHANNEL_12
#define BSP_ADCCH_LIPO3                     LL_ADC_CHANNEL_13
#define BSP_ADCCH_INTTEMP                   LL_ADC_CHANNEL_TEMPSENSOR

/**
 * @brief Definitions of external interrupt lines.
 */
#define BSP_BUTTON_EXTI_LINE                LL_EXTI_LINE_13
#define BSP_BUTTON_GPIO_EXTI_LINE           LL_GPIO_AF_EXTI_LINE13
#define BSP_BUTTON_GPIO_EXTI_PORT           LL_GPIO_AF_EXTI_PORTC

/**
 * @brief TTY configuration
 */
#define TTY_USARTx                          USART2
#define TTY_USARTx_CLK_ENABLE()             LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2)

#define BSP_GPIO_A2                         BSP_GPIO_TTY_TX
#define BSP_GPIO_A3                         BSP_GPIO_TTY_RX

#define TTY_USARTx_IRQn                     USART2_IRQn
#define TTY_USARTx_IRQHandler               USART2_IRQHandler

/**
 * @brief Hence that the DMA channel to use is hard coded by ST.
 * See DMA channel assignment table below. 
 */
#define TTY_TXDMACH                         DMA1_Channel7
#define TTY_TXDMACH_IRQn                    DMA1_Channel7_IRQn
#define TTY_TXDMACH_IRQHandler              DMA1_Channel7_IRQHandler
#define TTY_TXDMACH_LLCH                    LL_DMA_CHANNEL_7
#define TTY_TXDMACH_ISACTIVEFLAG_TC()       LL_DMA_IsActiveFlag_TC7(DMA1)
#define TTY_TXDMACH_CLEARFLAG_TC()          LL_DMA_ClearFlag_TC7(DMA1)

/**
 * @brief DMA channel assignment from ST.
 * 
 * Please enote that this hard coded and can not be changed!
 */
#define BSP_DMACH_ADC1                      DMA1_Channel1

#define BSP_DMACH_SPI1_TX                   DMA1_Channel3
#define BSP_DMACH_SPI1_RX                   DMA1_Channel2
#define BSP_DMACH_SPI2_TX                   DMA1_Channel5
#define BSP_DMACH_SPI2_RX                   DMA1_Channel4

#define BSP_DMACH_USART1_TX                 DMA1_Channel4
#define BSP_DMACH_USART1_RX                 DMA1_Channel5
#define BSP_DMACH_USART2_TX                 DMA1_Channel7
#define BSP_DMACH_USART2_RX                 DMA1_Channel6
#define BSP_DMACH_USART3_TX                 DMA1_Channel2
#define BSP_DMACH_USART3_RX                 DMA1_Channel3

#define BSP_DMACH_I2C1_TX                   DMA1_Channel6
#define BSP_DMACH_I2C1_RX                   DMA1_Channel7
#define BSP_DMACH_I2C2_TX                   DMA1_Channel4
#define BSP_DMACH_I2C2_RX                   DMA1_Channel5

#define BSP_DMACH_TIM1_CH1                  DMA1_Channel2
#define BSP_DMACH_TIM1_CH3                  DMA1_Channel6
#define BSP_DMACH_TIM1_CH4                  DMA1_Channel4
#define BSP_DMACH_TIM1_UP                   DMA1_Channel5
#define BSP_DMACH_TIM1_TRIG                 DMA1_Channel4
#define BSP_DMACH_TIM1_COM                  DMA1_Channel4

#define BSP_DMACH_TIM2_CH1                  DMA1_Channel5
#define BSP_DMACH_TIM2_CH2                  DMA1_Channel7
#define BSP_DMACH_TIM2_CH3                  DMA1_Channel1
#define BSP_DMACH_TIM2_CH4                  DMA1_Channel7
#define BSP_DMACH_TIM2_UP                   DMA1_Channel2

#define BSP_DMACH_TIM3_CH1                  DMA1_Channel6
#define BSP_DMACH_TIM3_CH3                  DMA1_Channel2
#define BSP_DMACH_TIM3_CH4                  DMA1_Channel3
#define BSP_DMACH_TIM3_UP                   DMA1_Channel3
#define BSP_DMACH_TIM3_TRIG                 DMA1_Channel6

#define BSP_DMACH_TIM4_CH1                  DMA1_Channel1
#define BSP_DMACH_TIM4_CH2                  DMA1_Channel4
#define BSP_DMACH_TIM4_CH3                  DMA1_Channel5
#define BSP_DMACH_TIM4_UP                   DMA1_Channel7

#define BSP_DMAIRQn(_ch)                    _ch##_IRQn
#define BSP_DMAIRQHandler(_ch)              _ch##_IRQHandler

#ifndef __NVIC_PRIO_BITS

/*
 * @brief __NVIC_PRIO_BITS should be specified when CMSIS is being used.
 * 
 * 7 priority levels as default
 * */
#define __NVIC_PRIO_BITS                    3

#endif

/**
 * @brief Lower values mean higher priorities, see EFM32 app note AN0039.
 */
#define BSP_IRQPRIO_MAX                     0x00

#ifndef BSP_IRQPRIO_FREERTOS_SYSCALL

/**
 * @brief The highest interrupt priority that can be used by any interrupt 
 * service routine that makes calls to interrupt safe FreeRTOS API functions.
 * 
 * DO NOT CALL INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT
 * HAS A HIGHER PRIORITY THAN THIS!
 */
#define BSP_IRQPRIO_FREERTOS_SYSCALL        0x04

#endif

/**
 * @brief The lowest interrupt priority that can be used in a call to a
 * NVIC_SetPriority() function. Given by the fact that there are only
 * tree priority bits.
 */
#define BSP_IRQPRIO_MIN                     0x07

/**
 * @brief Generic type used as return value for public BSP functions where 
 * applicable.
 */
typedef enum
{
      BSP_OK = 0,           ///<! In case of success
      BSP_IDLE,             ///<! Recourse idle
      BSP_ERR,              ///<! Unspecified error
      BSP_EBUSY,            ///<! Recourse busy
      BSP_ETIMEOUT,         ///<! Timeout
      BSP_EEINVAL,          ///<! Invalid arguments
      BSP_ELOCK,            ///<! Locking error
      BSP_EEMPTY,           ///<! is empty 
      BSP_ENEMPTY,          ///<! is not empty 
      BSP_ERANGE,           ///<! Range violation
      BSP_ESIZE             ///<! Size violation
      
} bspStatus_t;

#if BSP_SYSTICK == BSP_ENABLED

#define BSP_MAX_DELAY                       UINT32_MAX

/**
 * @brief Used to the current sys tick counter value.
 *
 * @return  The sys tick value.
 */
uint32_t bspGetSysTick(void);

/**
 * @brief Used to implement a busy waiting delay for the given amount of time.
 *
 * @param delay     The delay in ms.
 */
void bspDelayMs(uint32_t delay);

#else /* BSP_SYSTICK == BSP_ENABLED */

#ifndef bspDelayMs

/**
 * @brief As last resort use the stm32cube ll implementation of a delay.
 */
#define bspDelayMs                          LL_mDelay

#endif

#endif /* BSP_SYSTICK == BSP_ENABLED */

/**
 * @brief Used to check if the current code is executed in the context of a 
 * interrupt.
 * 
 * ATTENTION: Not tested yet!
 */
bool bspInInterrupt(void);

/**
 * @brief Used to implement generic chip initialization
 */
void bspChipInit(void);

/**
 * @brief used to trigger a CPU reset.
 */
void bspResetCpu(void);

#endif /* BSP_MOTORCTRL_H_ */
