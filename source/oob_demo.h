/******************************************************************************
 * File Name:   oob_demo.h
 *
 * Description: oob_demo header encompases and manages all the inter function
 * calls between each Code Examples.
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

#ifndef _OOB_DEMO_H_
#define _OOB_DEMO_H_

#include "cy_pdl.h"
#include "cybsp.h"


/*******************************************************************************
 * Macros
 *******************************************************************************/
/* demo project number */
#define     DEMONUM              7
/* default command */
#define     CMD_DEFAULT          0xFF
#define     DEM_HELLO_WORD       0x31
#define     DEM_IO_INTR          0x32
#define     DEM_IO_ADC           0x33
#define     DEM_HRPWM            0x34
#define     DEM_EEPROM           0x35
#define     LOWER_HRPPWM         64
#define     INTEGER_STEP         64
#define     HIGHER_HRPPWM        2496
#define     PERCENT_NUM          100

/* LED states */
#define     LED_ON               (0)
#define     LED_OFF              (1)


/*******************************************************************************
 * External Functions
 *******************************************************************************/
extern void main_helloworld(void);
extern void main_gpio_interrupt(void);
extern void main_sar_adc(void);
extern void main_hrpwm(void);
extern void gpio_interrupt_handler(void);

/* Array of demo projects */
extern void (*demoProject[DEMONUM])(void);


/*******************************************************************************
 * External Variables
 *******************************************************************************/
/* demo switch flag */
extern bool evt_switch;

/* System Interrupt call */
extern cy_stc_sysint_t intrCfg;

/* demo project index */
extern uint8_t demo_index;

/* Flag to start and stop led_blinking */
extern volatile bool led_blink_flag;

/* The HPPASS interrupt configuration structure */
extern cy_stc_sysint_t hppass_intr_config;

/* HPPASS block ready status*/
extern volatile bool hppass_is_ready;

/* ADC conversion starting flag */
extern volatile bool start_adc_conversion;

/* Delay for GPIO Interrput */
extern uint32_t delay_led_blink;
#endif
