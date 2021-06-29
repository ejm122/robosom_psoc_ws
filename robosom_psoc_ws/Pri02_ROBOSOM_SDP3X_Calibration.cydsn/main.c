/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "stdio.h"
#include <sdpsensor.h>
#include <sfm3x00.h>

//ADC
void print_adc_via_usbuart(void);
int16 adc_val_int16 = 0;
float32 adc_value = 0;
float flow = 0;
//sys_clock_cur_ms = 0;

//// USBUART
//#define USBFS_DEVICE    (0u)
//#define USBUART_BUFFER_SIZE (256u)
//uint16 count;
//uint8 buffer[USBUART_BUFFER_SIZE];
//void USBUART_user_check_init(void);
//void USBUART_user_echo(void);
//
//// Testing Function
//void print_sensor_via_usbuart(int n, float pressure, float temp, float32 flow, uint8_t crc_check_num);
//
//// System clock
//uint32 sys_clock_cur_ms = 0;
//float sys_clock_cur_us_in_ms = 0;
//float temp;
//float pressure;
//void sys_clock_ms_callback(void); // 1ms callback interrupt function


int main(void)
{
    uint8_t led_test = 0;
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    // USBUART Init
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    USBUART_CDC_Init();
    
    // I2C Init
    I2C_1_Start();
    
    /* Start the ADC components */
    //ADC_DelSig_1_Start();
    
    /* Start the ADC conversion */
    //ADC_DelSig_1_StartConvert();
    
    //Initialize for flow sensor
//      int ret = init(); //if return 0, then successful initialization
//      while (ret != 0) {
//            CyDelay(500);
//            init();
//    }
    
    //sfmBegin(); 
    
    // PWM Block Init
    PWM_LED_Start();
   
    /* Turn off LEDs */
    Led_Red_Write(0);
    Led_Green_Write(0);
    Led_Blue_Write(0);
    Led_Key_Write(1);
    
    CyDelay(100);
    
    // Start system 1ms tick
    CySysTickStart();
    CySysTickSetCallback(0, sys_clock_ms_callback);
    CySysTickEnableInterrupt();
       
    int n = 0;
    for(;;)
    {
            sfmBegin();
        
        //n = n+1; //for tracing sensor readouts
//        
          //VolFlow = k*sqrt(delta_p) + bias
            //int ret = readSample(); //triggered measurement
            //pressure = getDifferentialPressure();
            //temp = getTemperature();
            //while (!(ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_RETURN_STATUS)));
            //adc_value = ADC_DelSig_1_CountsTo_Volts(ADC_DelSig_1_GetResult32());
            //flow = (adc_value-0.48)*25; //ADC at 0 flow settles at 0.45-0.48V, this equation was derived from plots in the datasheet
            //USBUART_user_check_init();
            //print_sensor_via_usbuart(n,pressure,temp,flow,ret);
            //flow = sfmReadFlow();
            //print_sensor_via_usbuart(n,sfmSerialNumber,sfmFlowOffset,flow,sfmFlowScale);
            
            led_test++;
            
            Led_Red_Write((led_test >> 0) & 0x01);
            Led_Green_Write((led_test >> 1) & 0x01);
            Led_Blue_Write((led_test >> 2) & 0x01);   

            if (Led_Key_Read() == 0)
            {
                led_test = 0;
            }
              
            
            CyDelay(10);  
    }
}

/// USBUART Routin
//void USBUART_user_check_init(void) {
//    /* Host can send double SET_INTERFACE request. */
//    if (0u != USBUART_IsConfigurationChanged())
//    {
//        /* Initialize IN endpoints when device is configured. */
//        if (0u != USBUART_GetConfiguration())
//        {
//            /* Enumeration is done, enable OUT endpoint to receive data 
//             * from host. */
//            USBUART_CDC_Init();
//        }
//    }
//}
//
//
//void USBUART_user_echo(void) {
//    /* Service USB CDC when device is configured. */
//    if (0u != USBUART_GetConfiguration())
//    {
//        /* Check for input data from host. */
//        if (0u != USBUART_DataIsReady())
//        {
//            /* Read received data and re-enable OUT endpoint. */
//            count = USBUART_GetAll(buffer);
//
//            if (0u != count)
//            {
//                /* Wait until component is ready to send data to host. */
//                while (0u == USBUART_CDCIsReady())
//                {
//                }
//
//                /* Send data back to host. */
//                USBUART_PutData(buffer, count);
//
//                /* If the last sent packet is exactly the maximum packet 
//                *  size, it is followed by a zero-length packet to assure
//                *  that the end of the segment is properly identified by 
//                *  the terminal.
//                */
//                if (USBUART_BUFFER_SIZE == count)
//                {
//                    /* Wait until component is ready to send data to PC. */
//                    while (0u == USBUART_CDCIsReady())
//                    {
//                    }
//
//                    /* Send zero-length packet to PC. */
//                    USBUART_PutData(NULL, 0u);
//                }
//            }
//        }
//    }
//}
//
//void print_sensor_via_usbuart(int n, float pressure, float temp, float32 flow, uint8_t crc_check_num)
//{   
//    sys_clock_cur_us_in_ms = (float)CySysTickGetValue() * (1/(float)cydelay_freq_hz);
//
//    while (0u == USBUART_CDCIsReady());
//
//    sprintf((char *)buffer, "%d. pressure: %f temperature: %f Flow: %f CRC(4): %d\n",n, pressure, temp, flow, crc_check_num);
//   
//    //count = sizeof(buffer);
//    /* Send data back to host. */
//    //USBUART_PutData(buffer, count);
//    USBUART_PutString((char8 *)buffer);
//}
//
// 1ms system tick callback interrupt function
//void sys_clock_ms_callback(void){
//    sys_clock_cur_ms ++; // increment ms counter by 1
//}

/* [] END OF FILE */
