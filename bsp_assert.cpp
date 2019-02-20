#include <stdio.h>

#include <stm32f1xx_ll_system.h>
#include <stm32f1xx_ll_tim.h>

#include "bsp/bsp_gpio.h"
#include "bsp/bsp_tty.h"
#include "bsp/bsp_motor.h"

#if BSP_ASSERT == BSP_ENABLED

void bspAssertDoCall(const char *pFunc, int line)
{
    uint8_t idx = 0;
    char buffer[80];

    /* Disable all interrupts to prevent all further actions */
    __disable_irq();

    snprintf (buffer, 80, "\nAssertion in %s(%d)\n", pFunc, line);

    /* Do here all what's needed to prevent any kind of hardware damage or 
     * unintended actions */
    LL_TIM_OC_SetCompareCH1(TIM3, 0);
    LL_TIM_OC_SetCompareCH2(TIM3, 0);
    LL_TIM_OC_SetCompareCH3(TIM3, 0);
    LL_TIM_OC_SetCompareCH4(TIM3, 0);

    while(1)
    {
       
#if BSP_ASSERT_MESSAGE == BSP_ENABLED

        bspTTYAssertMessage(buffer);

#endif /* BSP_ASSERT_MESSAGE == BSP_ENABLED */

        idx=0;
        while(idx < (BSP_ASSERT_MESSAGE_MS/(BSP_ASSERT_LED_MS/2)))
        {
            bspGpioToggle(BSP_GPIO_LED);
            LL_mDelay(BSP_ASSERT_LED_MS/2);
            idx++;
        }
    }
}

#endif /* BSP_ASSERT == BSP_ENABLED */