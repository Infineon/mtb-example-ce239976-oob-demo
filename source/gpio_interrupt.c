/******************************************************************************
 * File Name:   gpio_interrupt.c
 *
 * Description: This is the source code for the PSoC Control C3 Out of Box
 * GPIO Interrupt demo Example for ModusToolbox.
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
#define DELAY_SHORT_MS          (250)   /* milliseconds */
#define DELAY_LONG_MS           (500)   /* milliseconds */
#define LED_BLINK_COUNT         (4)


/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void blink_led(void);
void main_gpio_interrupt(void);
void gpio_interrupt_handler(void);


/*******************************************************************************
 * Global Variables
 ********************************************************************************/
uint32_t delay_led_blink = DELAY_LONG_MS;

cy_stc_sysint_t intrCfg =
{
    .intrSrc = ioss_interrupts_sec_gpio_5_IRQn, /* Interrupt source is GPIO port 5 interrupt */
    .intrPriority = 2UL                         /* Interrupt priority is 2 */
};


/*******************************************************************************
* Function Definitions
*******************************************************************************/
/********************************************************************************
 * Function Name: blink_led
 ********************************************************************************
 * Summary:
 * User defined blink led function
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void blink_led(void)
{
    uint32_t count = 0;
    for (count = 0; count < LED_BLINK_COUNT; count++)
    {
        /* Turn ON LED1 Pin */
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_ON);
        /* Delay between LED toggles */
        Cy_SysLib_Delay(delay_led_blink);

        /* Turn OFF LED1 Pin */
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF);
        /* Delay between LED toggles */
        Cy_SysLib_Delay(delay_led_blink);
    }
}


/********************************************************************************
 * Function Name: main_gpio_interrupt
 ********************************************************************************
 * Summary:
 *  System entrance point for the GPIO interrupt demo
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void main_gpio_interrupt(void)
{
    /* printing the message */
    printf("*********************** Running GPIO interrupt demo **********************\r\n");
    printf("GPIO Interrupt demo started successfully. \r\n");
    printf("Press the USER_BTN1 to blink USER_LED1 for four times with short delay and"
              "\r\nlong delay alternatively. \r\n");

    while(!evt_switch)
    {
    }
}


/********************************************************************************
 * Function Name: gpio_interrupt_handler
 ********************************************************************************
 * Summary:
 *   GPIO interrupt handler.
 *
 * Parameters:
 *  void
 *
 *******************************************************************************/
void gpio_interrupt_handler(void)
{
    switch(demo_index)
    {
    case 1:
          /* Read current button state from the user button on pin 0_4 */
          if (led_blink_flag)
          {
               led_blink_flag = false;
               Cy_TCPWM_TriggerStopOrKill_Single(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);
               printf("LED blinking paused\r\n");
               Cy_SysLib_Delay(200);
          }
          else /* Resume LED blinking by starting the timer */
          {
               led_blink_flag = true;
               Cy_TCPWM_TriggerStart_Single(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);
               printf("LED blinking resumed\r\n");
               Cy_SysLib_Delay(200);
          }
        /* Clear pin interrupt logic. Required to detect next interrupt */
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
        break;

    case 2:
          /* Update LED toggle delay */
          if (DELAY_LONG_MS == delay_led_blink)
          {
               delay_led_blink = DELAY_SHORT_MS;
               blink_led();
          }
          else
          {
               delay_led_blink = DELAY_LONG_MS;
               blink_led();
          }
        /* Clear pin interrupt logic. Required to detect next interrupt */
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
        break;

    case 3:
        /* Clear pin interrupt logic. Required to detect next interrupt */
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
        break;

    case 4:
        /* Clear pin interrupt logic. Required to detect next interrupt */
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
        break;

    default:
        break;
    }
}

/* [] END OF FILE */
