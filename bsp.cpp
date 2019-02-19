/*
 * bsp.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: julian
 */

#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_system.h>
#include <stm32f1xx_ll_utils.h>
#include <stm32f1xx_ll_bus.h>

#include "bsp/bsp.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_tty.h"

inline bool bspIsInterrupt()
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}

#if BSP_SYSTICK == BSP_ENABLED

/**
 * The bsp controls the sys tick. We have to maintain the tick counter, enable
 * the interrupt and implement it
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
 * All clock´s shall be managed here to keep the big picture.
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
    NVIC_SetPriority(SysTick_IRQn, BSP_SYSTICK_IRQ_PRIO);

#endif /* BSP_SYSTICK == BSP_ENABLED */

    /* DMA is used for the tty etc. */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    TTY_USARTx_CLK_ENABLE();
}

void bspChipInit(void)
{
    /* Turn on all need clocks at once */
    bspClockInit();

    /* Configure all pins used by the bsp */
    bspGpioInit();

    /* Configure the tty */
    bspTTYInit(BSP_TTY_BAUDRATE);
}

