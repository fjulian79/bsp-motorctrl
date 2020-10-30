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

#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_rcc.h"
#include <stm32f1xx_ll_dma.h>

#include "bsp/bsp.h"
#include "bsp/bsp_adc.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_assert.h"

#include <generic/generic.hpp>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief The reference value for 25°C in converted from mV to ADC digits. See
 * stm32f103 datasheet DocID13587 Rev 17.
 */
#define BSP_ADC_TEMP_V25                    1775

/**
 * @brief Used to convert adc digits into °C/100. Based on the he average slope
 * value provided by the datasheet. See stm32f103 datasheet DocID13587 Rev 17. 
 */
#define BSP_ADC_TEMP_NUM                    9593

/**
 * @brief Used to convert adc digits into °C/100. Based on the he average slope
 * value provided by the datasheet. See stm32f103 datasheet DocID13587 Rev 17. 
 */
#define BSP_ADC_TEMP_DEN                    512

/**
 * @brief Offset of 25°C ust to get a correct temerature value.
 */
#define BSP_ADC_TEMP_OFFS                   2500

/**
 * @brief Number of CPU Cycles to wait prior ADC Calibration
 */
#define ADC_CALIBDELAY          (LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32)

/**
 * @brief Interal adc raw values. This structure is used for the DMA transfer
 * and shall only accessed when the ADC is idle.
 * 
 */
typedef struct 
{
    uint16_t Cell[BSP_ADC_NUMCELLS];
    uint16_t Temperature;

} bspAdcRaw_t;

/**
 * @brief Internal adc values in a meaningsfull unit. Those values are updated 
 * by the bsp ADC task when a conversion has been compleated.
 */
typedef struct
{
    int32_t Lipo_mV[BSP_ADC_NUMCELLS];  /* Cell volatge in mV */
    int32_t Temperature_C;              /* Temperature in °C/100 */

}bspAdcValues_t;

/**
 * @brief The globals adc data instance.
 */
struct 
{
    bspAdcRaw_t Raw;

    bspAdcValues_t Value;

    bool Busy;

}bspAdcData;

extern "C" void DMA1_Channel1_IRQHandler(void)
{
    if(LL_DMA_IsActiveFlag_TC1(DMA1))
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        bspAdcData.Busy = false;
    }
}

void bspAdcDmaInit(void)
{
    LL_DMA_InitTypeDef dma;

    dma.Mode = LL_DMA_MODE_CIRCULAR;
    dma.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dma.Priority = LL_DMA_PRIORITY_HIGH;
    dma.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    dma.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    dma.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
    dma.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
    dma.PeriphOrM2MSrcAddress = LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA);
    dma.MemoryOrM2MDstAddress = (uint32_t) &bspAdcData.Raw;
    dma.NbData = sizeof(bspAdcRaw_t)/sizeof(uint16_t);
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &dma);

    NVIC_SetPriority(DMA1_Channel1_IRQn, BSP_IRQPRIO_ADC);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

void bspAdcInit(void)
{
    LL_ADC_InitTypeDef adcInit;
    LL_ADC_REG_InitTypeDef adcRegInit;
    volatile uint32_t tmp = 0;

    memset((void*) &bspAdcData.Raw, 0, sizeof(bspAdcRaw_t));
    memset((void*) &bspAdcData.Value, 0, sizeof(bspAdcValues_t));
    bspAdcData.Busy = false;

    bspAdcDmaInit();

    adcInit.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    adcInit.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
    LL_ADC_Init(ADC1, &adcInit);

    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

    adcRegInit.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    adcRegInit.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS;
    adcRegInit.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    adcRegInit.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    adcRegInit.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
    LL_ADC_REG_Init(ADC1, &adcRegInit);

    LL_ADC_SetChannelSamplingTime(ADC1, BSP_ADCCH_LIPO1, LL_ADC_SAMPLINGTIME_71CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, BSP_ADCCH_LIPO2, LL_ADC_SAMPLINGTIME_71CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, BSP_ADCCH_LIPO3, LL_ADC_SAMPLINGTIME_71CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, BSP_ADCCH_INTTEMP, LL_ADC_SAMPLINGTIME_239CYCLES_5);

    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, BSP_ADCCH_LIPO1);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, BSP_ADCCH_LIPO2);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, BSP_ADCCH_LIPO3);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, BSP_ADCCH_INTTEMP);
 
    /* Note: Variable divided by 2 to compensate partially CPU processing cycles.
     * (depends on compilation optimization). 
     * */
    tmp = ADC_CALIBDELAY >> 1;

    LL_ADC_Enable(ADC1);
    while(tmp != 0) tmp--;
    
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);

    bspAdcStartConversion();
}

bool bspAdcStartConversion(void)
{
    if(!bspAdcIsBusy())
    {
        LL_ADC_REG_StartConversionSWStart(ADC1);
        bspAdcData.Busy = true;
        return true;
    }

    return false;
}

bool bspAdcIsBusy(void)
{
    return bspAdcData.Busy;
}

int32_t bspAdcGetTemperature()
{
    return bspAdcData.Value.Temperature_C;
}

int32_t bspAdcGetCellVoltage(uint8_t cell)
{
    bspAssert(cell < BSP_ADC_NUMCELLS);

    return bspAdcData.Value.Lipo_mV[cell];
}

int32_t bspAdcGetMinCellVoltage(void)
{
    int32_t volt = 500;

    for(uint8_t cell = 0; cell < BSP_ADC_NUMCELLS; cell++)
        volt = min(volt, bspAdcData.Value.Lipo_mV[cell]);

    return volt;
}

int32_t bspAdcGetBatteryVoltage(void)
{
    int32_t batt = 0;

    for(uint8_t cell = 0; cell < BSP_ADC_NUMCELLS; cell++)
        batt += bspAdcData.Value.Lipo_mV[cell];

    return batt;
}

void bspAdcTask(void)
{
    if (!bspAdcIsBusy())
    {
        bspAdcData.Value.Temperature_C = BSP_ADC_TEMP_V25 - bspAdcData.Raw.Temperature;
        bspAdcData.Value.Temperature_C *= BSP_ADC_TEMP_NUM;
        bspAdcData.Value.Temperature_C /= BSP_ADC_TEMP_DEN;
        bspAdcData.Value.Temperature_C += BSP_ADC_TEMP_OFFS;

        /* These conversion values are not final as they correspond to the 
         * resistors used on the break out board.
         * */
        bspAdcData.Value.Lipo_mV[0] = ((int32_t)bspAdcData.Raw.Cell[0] * 215) >> 11;
        bspAdcData.Value.Lipo_mV[1] = ((int32_t)bspAdcData.Raw.Cell[1] * 935) >> 12;
        bspAdcData.Value.Lipo_mV[2] = ((int32_t)bspAdcData.Raw.Cell[2] * 1419) >> 12;

        /* We are interested in cell volatges so we have to calculate relative
         * values. 
         * */
        bspAdcData.Value.Lipo_mV[2] -= bspAdcData.Value.Lipo_mV[1];
        bspAdcData.Value.Lipo_mV[1] -= bspAdcData.Value.Lipo_mV[0];

        bspAdcStartConversion();
    }
}