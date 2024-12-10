/******************************************************************************
 * File Name:   hrpwm.c
 *
 * Description: This is the source code for the PSOC Control C3 Out of Box
 * HRPWM demo Example for ModusToolbox.
 *
 * Related Document: See README.md
 *
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
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "oob_demo.h"
#include "print_message.h"
#include <stdio.h>


/*******************************************************************************
 * Macros
 ********************************************************************************/
#define BUFFER_SIZE                (8)
#define COMPARE_VALUE_DELTA        (1)
#define PWM_FREQ_MHZ               (240)
#define INTEGER_BITS               (6)
#define FRACTIONAL_MASK_BITS       (0x3F)
#define ADC_VOLTAGE_HRPWM          (3300)
#define ADC_HIGH_VALUE             (4095)


/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void read_adc_channel_result1(void);
void print_instructions(void);
void main_hrpwm(void);


/*******************************************************************************
 * Global Variables
 ********************************************************************************/
float32_t dc_value;
uint16_t  period; /* Variable to store period value of TCPWM block */
int16_t   compare0_value; /* Variable to store the CC0 value of TCPWM block */

uint16_t intr_period;
uint16_t frac_period;
uint8_t hrpwm_freq;


/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: main_hrpwm
********************************************************************************
* Summary:
* This is the main function for HRPWM demo
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
void main_hrpwm(void)
{
     /* Printing HRPWM message */
    printf("*************************** Running HRPWM Demo ***************************\r\n\n");
    printf("HRPWM (High Resolution Pulse Width Modulation) enhances the time resolution"
              "\r\nof traditional digital PWM (Pulse Width Modulation) signals.\r\n\n");
    printf("This demo uses HRPWM to generate a 6 MHz PWM output with more resolution"
              "\r\nthan what is possible with standard digital PWM with 240 MHz clock input.\r\n\n");
    printf("While standard digital PWM gives only 40 steps of resolution (0 to 39),"
              "\r\nHRPWM in PSOC Control C3 provides 64 fractional steps per digital step.\r\n\n");
    printf("----------          -------------          -----------\r\n");
    printf("| HRPWM  |--------->| RC Filter |--------->| SAR ADC |\r\n");
    printf("----------          -------------          -----------\r\n\n");
    printf("The HRPWM output is connected to the SAR ADC through an RC filter."
              "\r\nThe SAR ADC sampler gain is set to 6. This gives approx. 10 count"
              "\r\nvariation for each fractional step change.\r\n\n");

    printf("CPU frequency          : %u MHz\r\n", cy_delayFreqMhz);
    printf("HRPWM Clock frequency  : %u MHz\r\n", (uint8_t)PWM_FREQ_MHZ);

    /* Initialize and enable the TCPWM block */
    Cy_TCPWM_PWM_Init(USR_PWM_HW, USR_PWM_NUM,
                      &USR_PWM_config);
    Cy_TCPWM_PWM_Enable(USR_PWM_HW, USR_PWM_NUM);

    /* Fetch the initial values of period, CC0 and CC1 registers configured
    * through the design file */
    period = Cy_TCPWM_PWM_GetPeriod0(USR_PWM_HW, USR_PWM_NUM);
    compare0_value = Cy_TCPWM_PWM_GetCompare0Val(USR_PWM_HW, USR_PWM_NUM);

    /* Start the TCPWM block */
    Cy_TCPWM_TriggerStart_Single(USR_PWM_HW, USR_PWM_NUM);

     intr_period = period >> INTEGER_BITS;
     frac_period = period & FRACTIONAL_MASK_BITS;
     hrpwm_freq = (uint8_t)PWM_FREQ_MHZ/intr_period;

    printf("HRPWM output frequency : %3u MHz\r\n\n", hrpwm_freq);

     /* Initialize ADC pins */
     init_adc();

     /* print the HRPWM Instructions */
     print_instructions();

    /* Start the HPPASS autonomous controller (AC) from state 0, didn't wait for HPPASS block ready */
    hppass_is_ready = false;
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, 0U))
    {
        CY_ASSERT(0);
    }
    /* CLear ADC conversion starting flag */
    start_adc_conversion = false;

    /* Check flag set by UART event handler. */
     while(!evt_switch)
     {
          /* If 'Enter" key is pressed */
          if(rec_cmd == 0x0D)
          {
            rec_cmd = 0xff;

            /* Start the ADC Conversion */
            if(!start_adc_conversion)
               {
                    start_adc_conversion = true;
                    printf("Start the HRPWM\r\n");
               }
          }
          /* If flag is true, trigger SAR ADC every second */
          if(start_adc_conversion)
          {
               /* Check SAR ADC busy status */
               if(!Cy_HPPASS_SAR_IsBusy())
               {
                    /* Read ADC channel result and print out the result */
                    read_adc_channel_result1();
               }
               Cy_SysLib_Delay(1000);
          }
     }
}


/*******************************************************************************
* Function Name: print_instructions
********************************************************************************
* Summary:
* Prints set of instructions.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void print_instructions(void)
{
     printf("==========================================================================\r\n");
     printf("Instructions:\r\n");
     printf("==========================================================================\r\n");
     printf("Press 'w'     : To Increasing the Integer Part by 1 (64 Steps)\r\n");
     printf("Press 'e'     : To Increasing the Fractional Part by 1 (1 Step)\r\n");
     printf("Press 's'     : To Decreasing the Integer Part by 1 (64 Steps)\r\n");
     printf("Press 'd'     : To Decreasing the Fractional Part by 1 (1 Step)\r\n");
     printf("Press 'Enter' : To start the HRPWM - SAR ADC conversion\r\n");
     printf("==========================================================================\r\n");
     printf("Period: %u [Integer Part : %02u, Fractional Part : %02u]\r\n", period, intr_period, frac_period);
     printf("==========================================================================\r\n\n");
}


/*******************************************************************************
* Function Name: read_adc_channel_result1
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
void read_adc_channel_result1(void)
{
    uint16_t channel_result = 0;
    uint32_t result_status = 0;
    float32_t volts = 0;
    float32_t buffer_volt[BUFFER_SIZE];
    float32_t mean_value_volt;
    float32_t buffer_channel[BUFFER_SIZE];
    float32_t mean_value_channel;
    int32_t buffer_index = 0;

    mean_value_volt = 0;
    mean_value_channel = 0;

    /* Averaging the read data */
    for (int32_t average = 0; average < BUFFER_SIZE; average++)
    {
          /* Trigger SAR ADC group 0 conversion */
          Cy_HPPASS_SetFwTrigger(CY_HPPASS_TRIG_0_MSK);

          /* Wait for channel conversion done */
          do
          {
               result_status = Cy_HPPASS_SAR_Result_GetStatus();
          } while(!(result_status & CY_HPPASS_SAR_CHAN_8_MSK));

          /* Get channel data */
          channel_result = Cy_HPPASS_SAR_Result_ChannelRead(CY_HPPASS_SAR_CHAN_8_IDX);

          /* Saving 8 consecutive ADC values */
          buffer_channel[buffer_index] = channel_result;
          mean_value_channel += buffer_channel[average];

          /* mapping ADC values to 3.3 Volts */
          volts = Cy_HPPASS_SAR_CountsTo_Volts(CY_HPPASS_SAR_CHAN_8_IDX, ADC_VOLTAGE_HRPWM, channel_result);

          /* Saving 8 consecutive voltages */
          buffer_volt[buffer_index] = volts;
          mean_value_volt += buffer_volt[average];

          buffer_index = (buffer_index + 1) % BUFFER_SIZE;
          Cy_SysLib_Delay(10);
    }

    /* Averaging 8 consecutive ADC values to have a stable ADC value */
    mean_value_channel = mean_value_channel/BUFFER_SIZE;

    /* Averaging 8 consecutive voltages to have a stable voltage */
    mean_value_volt = mean_value_volt/BUFFER_SIZE;
    dc_value = (compare0_value * PERCENT_NUM);
    Cy_SysLib_Delay(10);

    /* If the CC0 value is less than 64, make the duty cycle 0 */
    dc_value = dc_value/period;
    if(compare0_value <= (LOWER_HRPPWM-1))
    {
         dc_value = 0;
    }

    int16_t integer_part1 = (int16_t)mean_value_channel;
    uint16_t integer_part2 = compare0_value >> INTEGER_BITS;
    uint16_t fractional_part = compare0_value & FRACTIONAL_MASK_BITS;

    /* print the HRPWM values in a readable format */
    printf("HRPWM - Period: %u, CC0: %04d [Integer Part : %02u, Fractional Part : "
           "%02u], Duty: %06.2f %%, ADC count = %04d", period, compare0_value,
           integer_part2, fractional_part, dc_value, integer_part1);

    /* If the ADC value goes above 4095, print the following message */
    if(integer_part1 == ADC_HIGH_VALUE)
    {
         printf(". ADC input at max value. Reduce CC0 Integer/Fractional Part\r\n");
    }

    /* If the ADC value goes below 64, print the following message */
    else if(compare0_value <= (LOWER_HRPPWM-1))
    {
         printf(". CC0 Value below 64 results in Duty = 0 %%\r\n");
    }
    else
    {
         printf("\r\n");
    }
    /* Clear result status */
    Cy_HPPASS_SAR_Result_ClearStatus(CY_HPPASS_SAR_CHAN_8_MSK);
}

/* [] END OF FILE */
