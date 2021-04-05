/* ========================================
 *
 * Copyright CMU Biorobotics Lab
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF CMU Biorobotics Lab.
 *
 * Created 02/25/2020 by npaiva@andrew.cmu.edu
 * 
 * Adapated from Cypress USBUART demo.
 * ========================================
*/

#include "usb_comm.h"
#include "stdio.h"
#include "stdbool.h"

#define USBFS_DEVICE    (0u)
#define USBUART_BUFFER_SIZE (64u)
#define OUT_BUFFER_SIZE     (512u)
#define LINE_STR_LENGTH     (20u)

static uint8_t in_buffer[USBUART_BUFFER_SIZE];
static uint16_t in_buffer_ct;
static uint16_t in_buffer_ind;

static uint8_t out_buffer[OUT_BUFFER_SIZE];
static uint16_t out_buffer_ct;
//static uint16_t out_buffer_ind;
    
void init_usb_comm(){
    /* Start USBFS operation with 5-V operation. */
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
}

/* Non-blocking */
void usb_print(uint8_t *buffer, uint8_t count){
    if(USBUART_CDCIsReady()){
        /* Send data back to host. */
        USBUART_PutData(buffer, count);
    }
}

/** @brief Function that reintializes USB communication if the configuration changes 
           Returns true if a reinitialization occured. */
bool usb_configuration_reinit(void) {
    /* Host can send double SET_INTERFACE request. */
    if (0u != USBUART_IsConfigurationChanged())
    {
        /* Initialize IN endpoints when device is configured. */
        if (0u != USBUART_GetConfiguration())
        {
            /* Enumeration is done, enable OUT endpoint to receive data 
             * from host. */
            USBUART_CDC_Init();
            return true;
        }
    }
    return false;
}

/* usb_put_string:
    Non-Blocking helper function that sends data over the USB port if it is enabled
*/
void usb_put_string(const char *str){
    //If USB is enabled
    if(USBUART_GetConfiguration()){
        
        if(USBUART_CDCIsReady()){
            if(out_buffer_ct > 0){
                uint8_t ind = 0;
                while(OUT_BUFFER_SIZE - out_buffer_ct > 1 && str[ind] != 0){
                    out_buffer[out_buffer_ct] = str[ind];
                    out_buffer_ct++;
                }
                out_buffer[out_buffer_ct] = 0x00;
                /*strncpy((char *)out_buffer + out_buffer_ct, str, 
                OUT_BUFFER_SIZE - out_buffer_ct);*/
            }
            USBUART_PutString(str);
            out_buffer_ct = 0;
        } else {
            uint8_t ind = 0;
            while(OUT_BUFFER_SIZE - out_buffer_ct > 1 && str[ind] != 0){
                out_buffer[out_buffer_ct] = str[ind];
                out_buffer_ct++;
            }
            out_buffer[out_buffer_ct] = 0x00;
        }
    }
}

/* usb_put_char:
    Non-Blocking helper function that sends data over the USB port if it is enabled
*/
void usb_put_char(uint8_t c){
    //If USB is enabled
    if(USBUART_GetConfiguration()){
        
        if(USBUART_CDCIsReady()){
            USBUART_PutChar(c);
        }
    }
}

uint16_t usb_num_available(bool *reconfigured){
    bool result;
    if(in_buffer_ind >= in_buffer_ct) {
        in_buffer_ct = 0;
        in_buffer_ind = 0;
        
        result = usb_configuration_reinit();
        if (reconfigured != NULL)
        {
            *reconfigured = result;
        }

        /* Service USB CDC when device is configured. */
        if (USBUART_GetConfiguration()){
            /* Check for input data from host. */
            //process_usb_in(128);
            if(USBUART_DataIsReady()){
                in_buffer_ct = USBUART_GetAll((uint8_t *)in_buffer);
            }
        }
    }
    
    return in_buffer_ct - in_buffer_ind;
}
    
uint8_t usb_get_char(bool *reconfigured){
    bool result;
    /* If we have data, return the next character */
    if(in_buffer_ct > 0 && in_buffer_ind < in_buffer_ct){
        uint8_t val = in_buffer[in_buffer_ind];
        in_buffer_ind++;
        return val;
    } else {
        in_buffer_ct = 0;
        in_buffer_ind = 0;
        
        
        result = usb_configuration_reinit();
        if (reconfigured != NULL)
        {
            *reconfigured = result;
        }

        /* Service USB CDC when device is configured. */
        if (USBUART_GetConfiguration()){
            /* Check for input data from host. */
            //process_usb_in(128);
            if(USBUART_DataIsReady()){
                in_buffer_ct = USBUART_GetAll((uint8_t *)in_buffer);
            }
        }
    }
    
    return 0;
}

void process_usb_comm(){
    /* Host can send double SET_INTERFACE request. */
    if (USBUART_IsConfigurationChanged()) {
        /* Initialize IN endpoints when device is configured. */
        if (USBUART_GetConfiguration()) {
            /* Enumeration is done, enable OUT endpoint to receive data 
             * from host. */
            USBUART_CDC_Init();
        }
    }

    /* Service USB CDC when device is configured. */
    if (USBUART_GetConfiguration()){
        /* Check for input data from host. */
        //process_usb_in(128);
    }
}



/* [] END OF FILE */
