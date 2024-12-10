/*******************************************************************************
 * File Name:   print_message.c
 *
 * Description: Print message function for communication with terminal and
 * get the user input.
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_hal.h"
#include "print_message.h"
#include "oob_demo.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Bytes of data to be transmitted */
#define DATA_LENGTH    10
#define W_KEY          0x77
#define E_KEY          0x65
#define S_KEY          0x73
#define D_KEY          0x64
#define MUL_HEX        0x0F


/*******************************************************************************
 * Global Variables
 *******************************************************************************/
/* UART received command. */
int rec_cmd = 0;

volatile bool confirm_flag = false;

cy_en_scb_uart_status_t errorStatus;

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    DEBUG_UART_context;           /** UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj;           /** Debug UART HAL object  */

/* Populate interrupt configuration structure */
cy_stc_sysint_t UART_SCB_IRQ_cfg =
{
    .intrSrc      = DEBUG_UART_IRQ,
    .intrPriority = 3u,
};


/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void uart_port_init(void);
void uart_event_handler(uint32_t event);
void Isr_uart1_fifo(void);


/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: Isr_uart1_fifo
********************************************************************************
* Summary:
* This function is registered to be called when UART1 interrupt occurs.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Isr_uart1_fifo(void)
{
    Cy_SCB_UART_Interrupt(DEBUG_UART_HW, &DEBUG_UART_context);
}


/*******************************************************************************
 * Function Name: uart_port_init
 ********************************************************************************
 * Summary:
 * Initialize the uart port to print in the Serial monitor and get the user input.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void uart_port_init(void)
{
     cy_rslt_t result;
    /* Initialize retarget-io to use the debug UART port */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Registers a callback function that notifies that
    *  uart_callback_events occurred in the Cy_SCB_UART_Interrupt.*/
    Cy_SCB_UART_RegisterCallback(DEBUG_UART_HW, (cy_cb_scb_uart_handle_events_t)uart_event_handler, &DEBUG_UART_context);

    /* Configuring priority and enabling NVIC IRQ
    * for the defined Service Request line number */
    Cy_SysInt_Init(&UART_SCB_IRQ_cfg, Isr_uart1_fifo);
    NVIC_EnableIRQ(UART_SCB_IRQ_cfg.intrSrc);

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
}


/********************************************************************************
 * Function Name: uart_event_handler
 ********************************************************************************
 * Summary:
 * Uart interrupt event handler callback function
 *
 * Parameters:
 *  handler_arg: user defined argument
 *  event: uart interrupt event source
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void uart_event_handler(uint32_t event)
{
    if (event == CY_SCB_UART_TRANSMIT_ERR_EVENT)
    {
        CY_ASSERT(0);
        /* An error occurred in Tx */
        /* Insert application code to handle Tx error */
    }
    else if (event == CY_SCB_UART_TRANSMIT_DONE_EVENT)
    {
        Cy_SCB_UART_ClearRingBuffer(DEBUG_UART_HW, &DEBUG_UART_context);
        /* All Tx data has been transmitted */
        /* Insert application code to handle Tx done */
    }
    else if (event == CY_SCB_UART_RECEIVE_DONE_EVENT)
    {
        CY_ASSERT(0);
        /* All Rx data has been received */
        /* Insert application code to handle Rx done */
    }
    else if (event == CY_SCB_UART_RECEIVE_NOT_EMTPY)
    {
        /* Get input command */
        uint32_t read_value = Cy_SCB_UART_Get(DEBUG_UART_HW);
        rec_cmd = (uint8_t)read_value;

        /* Distinguish command */
        switch(rec_cmd)
        {
        case DEM_HELLO_WORD:
            /* Set demo_index */
            demo_index = DEM_HELLO_WORD & MUL_HEX;
            /* Change demo switch flag */
            evt_switch = true;
            break;
        case DEM_IO_INTR:
            /* Set demo_index */
            demo_index = DEM_IO_INTR & MUL_HEX;
            /* Change demo switch flag */
            evt_switch = true;
            break;
        case DEM_IO_ADC:
            /* Set demo_index */
            demo_index = DEM_IO_ADC & MUL_HEX;
            /* Change demo switch flag */
            evt_switch = true;
            break;
        case DEM_HRPWM:
            /* Set demo_index */
            demo_index = DEM_HRPWM & MUL_HEX;
            /* Change demo switch flag */
            evt_switch = true;
            break;
          case W_KEY:
               if(start_adc_conversion)
               {
                    /* Increment the CC0 value in multiples of 64 steps */
                     compare0_value += INTEGER_STEP;
                    /* If CC0 is higher than 422, make CC0 to 422 */
                    if(compare0_value > HIGHER_HRPPWM)
                         compare0_value = HIGHER_HRPPWM;

                    dc_value = (compare0_value * PERCENT_NUM);
               }
               break;
          case E_KEY:
               if(start_adc_conversion)
               {
                    /* Increment the CC0 value in terms of 1 step */
                    compare0_value += COMPARE_VALUE_DELTA;

                    /* If CC0 is higher than 422, make CC0 to 422 */
                    if(compare0_value > HIGHER_HRPPWM)
                         compare0_value = HIGHER_HRPPWM;

                    dc_value = (compare0_value * PERCENT_NUM);
               }
               break;
          case S_KEY:
               if(start_adc_conversion)
               {
                    /* Decrement the CC0 value in multiples of 64 steps */
                     compare0_value -= INTEGER_STEP;

                    /* If CC0 value is less than 64 and greater than 0, make the duty cycle 0 */
                    if(compare0_value < LOWER_HRPPWM && compare0_value >= 0)
                    {
                         dc_value = (compare0_value * PERCENT_NUM);
                    }
                    /* If CC0 value is less than 0, equate the CC0 to 0 */
                    else if(compare0_value <= 0)
                    {
                         compare0_value = 0;
                    }
               }
               break;
          case D_KEY:
               if(start_adc_conversion)
               {
                    /* Decrement the CC0 value in terms of 1 step */
                    compare0_value -= COMPARE_VALUE_DELTA;

                    /* If CC0 value is less than 64 and greater than 0, make the duty cycle 0 */
                    if(compare0_value < LOWER_HRPPWM && compare0_value >= 0)
                    {
                         dc_value = (compare0_value * PERCENT_NUM);
                    }
                    /* If CC0 value is less than 0, equate the CC0 to 0 */
                    else if(compare0_value <= 0)
                    {
                         compare0_value = 0;
                    }
               }
               break;
          }
          /* Set new values for CC0/1 compare buffers */
          Cy_TCPWM_PWM_SetCompare0BufVal(USR_PWM_HW, USR_PWM_NUM, compare0_value);
          /* Trigger compare swap with its buffer values */
          Cy_TCPWM_TriggerCaptureOrSwap_Single(USR_PWM_HW, USR_PWM_NUM);
          Cy_SysLib_Delay(50);
    }
}

/* [] END OF FILE */
