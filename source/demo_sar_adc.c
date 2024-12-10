/******************************************************************************
 * File Name: demo_sar_adc.c
 *
 * Description: This is the source code for the PSoC Control C3 MCU SAR ADC
 *              example in the OOB Demo for ModusToolbox.
 *
 * Related Document: See README.md
 *
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Header Files
 *******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "print_message.h"
#include "oob_demo.h"
#include <stdio.h>
#include <string.h>


/*******************************************************************************
 * Macros
 *******************************************************************************/
#define ADC_VOLTAGE  3300


/*******************************************************************************
 * Global Variables
 *******************************************************************************/
/* The HPPASS interrupt configuration structure */
cy_stc_sysint_t hppass_intr_config =
{
    .intrSrc = pass_interrupt_mcpass_IRQn,
    .intrPriority = 0U,
};

/* HPPASS block ready status*/
volatile bool hppass_is_ready = false;

/* ADC conversion starting flag */
volatile bool start_adc_conversion = false;


/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void hppass_intr_handler(void);
void init_adc(void);
void read_adc_channel_result(void);
void main_sar_adc(void);


/*******************************************************************************
 * Function Definitions
 *******************************************************************************/
/*******************************************************************************
 * Function Name: init_adc
 ********************************************************************************
 * Summary:
 * This function is used to initilize the ADC channel.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void init_adc(void)
{
    /* Configure HPPASS interrupt */
    Cy_HPPASS_SetInterruptMask(CY_HPPASS_INTR_AC_INT);
    Cy_SysInt_Init(&hppass_intr_config, hppass_intr_handler);
    NVIC_EnableIRQ(hppass_intr_config.intrSrc);

    /* Start the HPPASS autonomous controller (AC) from state 0, didn't wait for HPPASS block ready */
    hppass_is_ready = false;
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, 0U))
    {
        CY_ASSERT(0);
    }

    /* Clear ADC conversion starting flag */
    start_adc_conversion = false;
    Cy_SysLib_Delay(100u);
}


/*******************************************************************************
 * Function Name: main_sar_adc
 ********************************************************************************
 * Summary:
 * This is the main function for the SAR ADC demo.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void main_sar_adc(void)
{
    init_adc();
    __enable_irq();

    printf("************************** Running SAR ADC demo **************************\r\n");
    printf("The SAR ADC demo started successfully. \r\n\n");
    printf("==========================================================================\r\n");
    printf("Instructions:\r\n");
    printf("==========================================================================\r\n");
    printf("Rotate POT (R1) to change ADC input\r\n");
    printf("==========================================================================\r\n\n");

    /* Check flag set by UART event handler. */
    while(!evt_switch)
    {
        start_adc_conversion = true;
        /* If flag is true, trigger SAR ADC every second */
        if(start_adc_conversion)
        {
            /* Check SAR ADC busy status */
            if(!Cy_HPPASS_SAR_IsBusy())
            {
                /* Read ADC channel result and print out the result */
                read_adc_channel_result();
            }
            Cy_SysLib_Delay(1000);
        }
    }
}


/*******************************************************************************
 * Function Name: hppass_intr_handler
 ********************************************************************************
 * Summary:
 * This function is the HPPASS interrupt handler (AC, etc.). Defined two states of
 * HPPASS autonomous controller (AC):
 * State 0 - Enable SAR and wait for block ready.
 * State 1 - Set AC interrupt and stop AC.
 * When AC perform to state 1, then send the AC interrupt to CPU to notify HPPASS
 * block is ready.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void hppass_intr_handler(void)
{
    uint32_t intrStatus = Cy_HPPASS_GetInterruptStatusMasked();
    /* Clear interrupt */
    Cy_HPPASS_ClearInterrupt(intrStatus);

    /* Check AC interrupt */
    if(CY_HPPASS_INTR_AC_INT == (intrStatus & CY_HPPASS_INTR_AC_INT))
    {
        hppass_is_ready = true;
    }
}


/*******************************************************************************
 * Function Name: read_adc_channel_result
 ********************************************************************************
 * Summary:
 * This function is the HPPASS SAR ADC channel result reading by polling.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void read_adc_channel_result(void)
{
    uint32_t result_status = 0;
    uint16_t channel_result = 0;
    float32_t volts = 0;

    /* Trigger SAR ADC group 0 conversion */
    Cy_HPPASS_SetFwTrigger(CY_HPPASS_TRIG_0_MSK);

    /* Wait for channel conversion done */
    do
    {
        result_status = Cy_HPPASS_SAR_Result_GetStatus();
    } while(!(result_status & CY_HPPASS_SAR_CHAN_12_MSK));

    /* Get channel data */
    channel_result = Cy_HPPASS_SAR_Result_ChannelRead(CY_HPPASS_SAR_CHAN_12_IDX);

    /* Convert the result to voltage */
    volts = Cy_HPPASS_SAR_CountsTo_Volts(CY_HPPASS_SAR_CHAN_12_IDX, ADC_VOLTAGE, channel_result);

    /* Print the ADC Value and Voltage */
    printf("ADC AN_B4 (POT) channel result = %04d, voltage = %.2f V\r\n", channel_result, volts);

    /* Clear result status */
    Cy_HPPASS_SAR_Result_ClearStatus(CY_HPPASS_SAR_CHAN_12_MSK);
}

/* [] END OF FILE */
