/******************************************************************************
 * File Name:   helloworld.c
 *
 * Description: This is the source code for the PSoC Control C3 Out of Box
 * Helloworld demo Example for ModusToolbox.
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
void timer_init(void);
void isr_timer(void);
static void led_statemanagement(uint8_t sta);
void main_helloworld(void);


/*******************************************************************************
* Function Definitions
*******************************************************************************/
const cy_stc_sysint_t intrCfg1 =
{
    .intrSrc = TCPWM_COUNTER_IRQ,
    .intrPriority = 7u
};


/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/*Led blinking status*/
volatile bool led_blink_flag = true;

volatile bool timer_interrupt_flag = false;

/* counter object used for blinking the LED state */
uint8_t led_statecounter = 0;


/*******************************************************************************
 * Function Name: main_helloworld
 ********************************************************************************
 * Summary:
 * This is the main function for CM33 CPU. It sets up a timer to trigger a
 * periodic interrupt. The main while loop checks for the status of a flag set
 * by the interrupt and toggles an LED at 1Hz to create an LED blinky. The
 * while loop also checks whether the 'BTN1' key was pressed and
 * stops/restarts LED blinking.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void main_helloworld(void)
{
    printf("************************ Running Hello world demo ************************\r\n");
    printf("Hello World!!!\r\n");
    printf("Press the Enter key to pause or resume blinking of the user LEDs.\r\n");
    printf("Alternatively, use USER BTN1 to pause or resume the blinking.\r\n");
    printf("\r\n");

    /* Initialize timer used for toggling the LED */
    timer_init();

    /* Check flag set by UART event handler. */
    while(!evt_switch)
    {
        if(rec_cmd == 0x0D)
        {
            rec_cmd = 0xff;

            /* Pause LED blinking by stopping the timer */
            if (led_blink_flag)
            {
                led_blink_flag = false;
                Cy_TCPWM_TriggerStopOrKill_Single(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);
                printf("LED blinking paused \r\n");
            }
            else /* Resume LED blinking by starting the timer */
            {
                led_blink_flag = true;
                Cy_TCPWM_TriggerStart_Single(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);
                printf("LED blinking resumed \r\n");
            }
        }
        /* Check if timer elapsed (interrupt fired) and toggle the LED */
        if (timer_interrupt_flag)
        {
            /* Clear the flag */
            timer_interrupt_flag = false;

            led_statecounter++;
            if(led_statecounter>3)
            {
                led_statecounter = 0;
            }

            /* switch the USER LED state */
            led_statemanagement(led_statecounter);
        }
    }
    led_statecounter = 0;
}


/********************************************************************************
 * Function Name: timer_init
 ********************************************************************************
 * Summary:
 * This function creates and configures a Timer object. The timer ticks
 * continuously and produces a periodic interrupt on every terminal count
 * event. The period is defined by the 'period' and 'compare_value' of the
 * timer configuration structure 'led_blink_timer_cfg'. Without any changes,
 * this application is designed to produce an interrupt every 1 second.
 *
 * Parameters:
 *  none
 *
 *******************************************************************************/
void timer_init(void)
{
    /* Enable interrupts */
    __enable_irq();

    /*TCPWM Counter Mode initial*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM, &TCPWM_COUNTER_config))
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized counter */
    Cy_TCPWM_Counter_Enable(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);

    /* Configure GPIO interrupt */
    Cy_TCPWM_SetInterruptMask(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM, CY_GPIO_INTR_EN_MASK);

    /* Configure CM33 CPU GPIO interrupt vector for Port 0 */
    Cy_SysInt_Init(&intrCfg1, isr_timer);
    NVIC_EnableIRQ(TCPWM_COUNTER_IRQ);

    /* Start the counter */
    Cy_TCPWM_TriggerStart_Single(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);
}


/*******************************************************************************
 * Function Name: isr_timer
 ********************************************************************************
 * Summary:
 * This is the interrupt handler function for the timer interrupt.
 *
 * Parameters:
 *    callback_arg    Arguments passed to the interrupt callback
 *    event            Timer/counter interrupt triggers
 *
 *******************************************************************************/
void isr_timer(void)
{
    uint32_t interrupts = Cy_TCPWM_GetInterruptStatusMasked(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);

    /* Clear the interrupt */
    Cy_TCPWM_ClearInterrupt(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM, interrupts);

    if (0UL != (CY_TCPWM_INT_ON_TC & interrupts))
    {
        /* Set the interrupt flag and process it from the main while(1) loop */
        timer_interrupt_flag = true;
    }
}


/*******************************************************************************
 * Function Name: led_statemanagement
 ********************************************************************************
 * Summary:
 * This function is for switching LEDs states.
 *
 * Parameters:
 *  sta: Next states of LEDs
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void led_statemanagement(uint8_t sta)
{
    /* array contains two LEDs display status 00 - 01 - 10 - 011 */
    uint8_t array[] = {0,1,2,3};

    /* Turn on/off the LED depending upon the bit set/cleared*/
    Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, (array[sta]&0x01) ? LED_ON : LED_OFF);
    Cy_GPIO_Write(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_PIN, (array[sta]&0x02) ? LED_ON : LED_OFF);
}

/* [] END OF FILE */
