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

#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_system.h>
#include <stm32f1xx_ll_utils.h>
#include <stm32f1xx_ll_bus.h>

#include "bsp/bsp.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_tty.h"
#include "bsp/bsp_motor.h"
#include "bsp/bsp_adc.h" 
#include "bsp/bsp_exti.h"

inline bool bspIsInterrupt()
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}

#if BSP_SYSTICK == BSP_ENABLED

/**
 * @brief The bsp controls the sys tick. We have to maintain the tick counter, 
 * enable the interrupt and implement it
 */
volatile uint32_t sysTick = 0;

extern "C" void SysTick_Handler(void)
{
    sysTick++;
}

uint32_t bspGetSysTick(void)
{
  return sysTick;
}

void bspDelayMs(uint32_t delay)
{
    uint32_t tickstart = bspGetSysTick();

    /* Add a period to guarantee minimum wait */
    if (delay < BSP_MAX_DELAY)
        delay++;

    while((bspGetSysTick() - tickstart) < delay);
}

#endif /* BSP_SYSTICK == BSP_ENABLED */

/**
 * @brief All clock´s shall be managed here to keep the big picture.
 */
static inline void bspClockInit(void)
{
    int clk = 0;

    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

#ifndef BSP_CLOCKSRC_HSI

    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();
    
    while(LL_RCC_HSE_IsReady() != 1);

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    clk = 72000000;

#else

    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1);

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
    clk = 64000000;

#endif

    LL_RCC_PLL_Enable();
    while(LL_RCC_PLL_IsReady() != 1);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    LL_SetSystemCoreClock(clk);

    LL_Init1msTick(clk);

#if BSP_SYSTICK == BSP_ENABLED

    SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
    NVIC_SetPriority(SysTick_IRQn, BSP_IRQPRIO_SYSTICK);

#endif /* BSP_SYSTICK == BSP_ENABLED */

    /* For the motor PWM and external interrupts pins we need AFIO */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    
    /* Timer 2 and 3 are used for the motors */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    /* DMA is used for the tty, adc, etc. */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /* Enable ADC clock and set it's prescaler, max. ADC clock is 14MHz */
    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

    TTY_USARTx_CLK_ENABLE();
}

void bspChipInit(void)
{
    /* Turn on all need clocks at once */
    bspClockInit();

    /* Configure all pins used by the bsp */
    bspGpioInit();

    /* Intitialize the timers used for the motors */
    bspMotorInit();

    /* Initialize the analog digital converter */
    bspAdcInit();

    /* Configure the tty */
    bspTTYInit(BSP_TTY_BAUDRATE);

    /* External interrupts (Button)*/
    bspExtiInit();
}

void bspResetCpu(void)
{
    NVIC_SystemReset();
}

void bspTask(uint32_t ticks)
{
    static uint32_t bspTicks = 0;
    
    if (ticks - bspTicks >= 100)
    {
        bspTicks = ticks;
        bspAdcTask();
    }
}