/*
 * bsp_tty.h
 *
 *  Created on: Jun 4, 2017
 *      Author: julian
 */

#ifndef BSP_MOTORCTRL_TTY_H_
#define BSP_MOTORCTRL_TTY_H_

#include "bsp/bsp.h"

/**
 * Used to setup the usart.
 *
 * @param baud the desired baud rate
 */
void bspTTYInit(uint32_t baud);

/**
 * Used to transmit any data.
 *
 * @param pData     Pointer to the data to transmit.
 * @param siz       Number of bytes.
 *
 * @return          Number of transmitted bytes.
 */
bspStatus_t bspTTYSendData(uint8_t *pData, uint16_t siz);

/**
 * Used to check if there is data available on the TTY.
 *
 * @return  true if there date.
 *          false it there is no data
 */
bool bspTTYDataAvailable(void);

/**
 * Used to read a single character from the TTY.
 * Hence that this function will block.
 *
 * @return  The read character.
 */
char bspTTYGetChar(void);

#if BSP_ASSERT_MESSAGE == BSP_ENABLED

/**
 * Used to print a assert message from any possible context.
 * This function can be assumed to prevent the tty from working as usual. The 
 * root cause for that behaviour is that this function as to work in any 
 * possible context, also in every interrupt.
 */
void bspTTYAssertMessage(char *pChar);

#endif /* BSP_ASSERT_MESSAGE == BSP_ENABLED */

#endif /* BSP_MOTORCTRL_TTY_H_ */
