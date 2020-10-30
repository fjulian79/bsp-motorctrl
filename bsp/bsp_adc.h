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

#ifndef BSP_MOTORCTRL_BSP_ADC_H_
#define BSP_MOTORCTRL_BSP_ADC_H_

#include "bsp/bsp.h"

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief The number of supported battery cells.
 */
#define BSP_ADC_NUMCELLS                    3

/**
 * @brief Initializes the adc and all other needed peripheral units as given by
 * the configuration.
 */
void bspAdcInit(void);
/**
 * @brief Used to start a adc conversion.
 * 
 * @return true   If a conversion has been started.
 * @return false  If the ADC is currently busy.
 */
bool bspAdcStartConversion(void);

/**
 * @brief Used to determine if the ADC is busy or not.
 * 
 * @return true   If the adc is busy.
 * @return false  If the adc is idle.
 */
bool bspAdcIsBusy(void);

/**
 * @brief Used to convert adc temperature sensor raw values into °C/100.
 * 
 * @return int32_t The temperature in °C/100.
 */
int32_t bspAdcGetTemperature(void);

/**
 * @brief Used to get the voltage of the given cell. 
 * 
 * @param cell The cell number. Has to be smaller then BSP_ADC_NUMCELLS.
 * 
 * @return int32_t The cell voltage in mV.
 */
int32_t bspAdcGetCellVoltage(uint8_t cell);

/**
 * @brief Used to get the voltage of the weakest cell.
 * 
 * @return int32_t The cell voltage in mV.
 */
int32_t bspAdcGetMinCellVoltage(void);

/**
 * @brief Used to get the overall battery voltage.
 * 
 * @return int32_t The battery voltage in mV.
 */
int32_t bspAdcGetBatteryVoltage(void);

/**
 * @brief Implements the periodic adc tasks
 * 
 */
void bspAdcTask(void);

#endif /* BSP_MOTORCTRL_BSP_ADC_H_ */