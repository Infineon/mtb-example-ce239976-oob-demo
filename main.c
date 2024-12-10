/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC Control C3 Out of Box demo
 *              Example for ModusToolbox.
 *
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


/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void startup_message(void);


/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
 * Global Variables
 ********************************************************************************/
 /* Demo switch flag */
bool evt_switch = true;

/* Demo project index */
uint8_t demo_index = 1u;

/* Array of demo projects */
void (*demoProject[DEMONUM])(void) = {
        main_helloworld,
        main_gpio_interrupt,
        main_sar_adc,
        main_hrpwm
};


/********************************************************************************
 * Function Name: startup_message
 ********************************************************************************
 * Summary:
 * prints demo options message on the UART debug port
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void startup_message(void)
{
    printf("\x1b[2J\x1b[;H");
    printf("**************************************************************************\r\n");
    printf("**  PSOC Control C3 MCU: Running the out-of-the-box (OOB) demo project  **\r\n");
    printf("**************************************************************************\r\n");
    printf("Enter an option from 1 - 4 to run the selected demo:\r\n");
    printf("\r\n");
    printf("\r\n");
    printf("1. Hello world\r\n");
    printf("2. GPIO interrupt\r\n");
    printf("3. SAR ADC basic\r\n");
    printf("4. HRPWM\r\n\n");
    printf("For more projects visit our code examples repositories:\r\n");
    printf("https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software\r\n");
    printf("For detailed steps refer to the README document\r\n\n");
    printf("\r\n");
}


/********************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * The main function performs the following actions:
 *  1. Initial UART component.
 *  2. Show 5 demos navigation interfaces on the UART serial terminal.
 *  3. Enter the "Hello world" demo (default demo) automatically
 *  4. You can enter 1~4 key for change the demos
 *
 * Parameters:
 *  None
 *
 * Return:
 *  int
 *
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure CM33 CPU GPIO interrupt vector for Port 0 */
    Cy_SysInt_Init(&intrCfg, gpio_interrupt_handler);
    NVIC_ClearPendingIRQ(intrCfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)intrCfg.intrSrc);

    /* Initialize UART port */
    uart_port_init();

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        if(evt_switch)
        {
            evt_switch = false;

            /* Initilally turn off LED1 and LED2 if Hello world is chosen */
            if(demo_index != 1)
            {
                /* Turn OFF LED1 Pin */
                Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF);

                /* Turn OFF LED2 Pin */
                Cy_GPIO_Write(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_PIN, CYBSP_LED_STATE_OFF);
            }
            startup_message();
            (*demoProject[demo_index-1])();
        }
        else
        {
            /* Will not run here */
        }
    }
}

/* [] END OF FILE */
