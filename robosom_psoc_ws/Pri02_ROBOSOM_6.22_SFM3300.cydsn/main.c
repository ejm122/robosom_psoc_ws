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
#include <i2chelper.c>

//sfmsensor
#define sfm3300i2c 0x40

// USBUART
#define USBFS_DEVICE    (0u)
#define USBUART_BUFFER_SIZE (256u)
uint16 count;
uint8 buffer[USBUART_BUFFER_SIZE];
void USBUART_user_check_init(void);
void USBUART_user_echo(void);

// Testing Function
void print_sensor_via_usbuart(int n, float pressure, float temp, float32 flow, uint8_t crc_check_num);
void print_sfm(uint32_t flow, uint32_t err1, uint32_t err2);
void print_status();
void print_first_sfm(uint8_t read1, uint8_t read2);

// System clock
uint32 sys_clock_cur_ms = 0;
float sys_clock_cur_us_in_ms = 0;
float temp;
float pressure;
void sys_clock_ms_callback(void); // 1ms callback interrupt function

//crc function
uint8_t SMF3000_CheckCrc (uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum); 
void print_ID(uint32_t id);

//for crc
typedef enum{
CHECKSUM_ERROR = 0x04
}etError;
typedef unsigned char u8t;

int main(void)
{
    uint8_t led_test = 0;
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    // USBUART Init
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    USBUART_CDC_Init();
    
    // I2C Init
    I2C_1_Start();
    
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
  
    
    int a = 0;
    int b = 0;
    
    // Reset -----------------------------------------------------
    
    const uint8_t CMD_LEN_1 = 2;
    uint8_t cmd_1[2] = { 0x20,0x00 }; //soft reset
    if(i2c_write(sfm3300i2c, cmd_1, CMD_LEN_1, 0) != 0) {//write command
        a = 1;

    }
    I2C_1_MasterSendStop(); /* Send Stop */    
    CyDelay(10);
    
    // Serial Number -----------------------------------------------------
    
    const uint8_t CMD_LEN_2 = 2;
    uint8_t cmd_2[2] = { 0x31,0xAE }; //first serial number
    if(i2c_write(sfm3300i2c, cmd_2, CMD_LEN_2, 0) != 0) {//write command
        a = 2;
    }
    
    const uint8_t DATA_LEN_3 = 3;
        uint8_t data_3[3] = { 0 };
        if(i2c_reRead(sfm3300i2c, data_3, DATA_LEN_3) != 0) {
            b = 1;
        }
 
    const uint8_t CMD_LEN_4 = 2;
    uint8_t cmd_4[2] = { 0x31,0xAF }; //second serial number
    if(i2c_write(sfm3300i2c, cmd_4, CMD_LEN_4, 0) != 0) {//write command
        a = 3;
    }  
    
    const uint8_t DATA_LEN_5 = 3;
        uint8_t data_5[3] = { 0 };
        if(i2c_reRead(sfm3300i2c, data_5, DATA_LEN_5) != 0) {
            b = 2;
        }
    int count;
    
    // prints serial number ------------------------------------------------------

    uint16_t upperBytes = (data_3[0] << 8) | data_3[1];
    uint16_t lowerBytes = (data_5[0] << 8) | data_5[1];
    uint32_t sfmSerialNumber = ((uint32_t)upperBytes << 16) | lowerBytes;
    
    USBUART_user_check_init();
    print_ID(sfmSerialNumber);
    
    // Continuous measurement:
    
    const uint8_t CMD_LEN_6 = 2;
    uint8_t cmd_6[2] = { 0x10,0x00 }; //continuous measurement
    if(i2c_write(sfm3300i2c, cmd_6, CMD_LEN_6, 0) != 0) {//write command
        a = 4;
    }
    //I2C_1_MasterSendStop(); /* Send Stop */
    
    const uint8_t DATA_LEN_7 = 3;
    uint8_t data_7[3] = { 0 };
    uint32_t err1 = i2c_firstRead(sfm3300i2c, data_7, DATA_LEN_7);
    USBUART_user_check_init();
    print_first_sfm(data_7[0], data_7[1]);
    //CyDelay(10);
    const uint8_t DATA_LEN_8 = 3;
    
 while(1) {  
   
    uint8_t data_8[3] = { 0 };
    uint32_t err2 = i2c_read(sfm3300i2c, data_8, DATA_LEN_8);

    uint8_t crc_check = SMF3000_CheckCrc(data_8, 2, data_8[2]);
    uint16_t flow_raw = (data_8[0] << 8) | data_8[1];
    //float flow = ((float)flow_raw - 32768 / 120);
    USBUART_user_check_init();
    print_sfm(flow_raw, err1,err2);
    CyDelay(100);
}
    
    
    
    //print_sensor_via_usbuart(1,flow_raw,a,b,crc_check);
//}
//        for(;;) 
//        {
//            
//            const uint8_t Flow_Read_Size = 3;
//            uint8_t data_8[3] = { 0 };
//            uint32_t err = 0;
//            err = i2c_read(sfm3300i2c, data_8, 3);
//            //{
//            //    b = 4;
//            //}
//            uint8_t crc_check = SMF3000_CheckCrc(data_8, 2, data_8[2]);
//            uint16_t flow_raw = (data_8[0] << 8) | data_8[1];
//            //float flow = ((float)flow_raw - 32768 / 120);
//            USBUART_user_check_init();
//            print_sfm(flow_raw, err);
//            //print_sensor_via_usbuart(1,flow_raw,a,b,crc_check);
//            
//        }
//        
        
    
//I2C_1_MasterSendStop();
//  CyDelay(100);
////  Wire.beginTransmission(sfm3300i2c);
////  Wire.write(0x20); // soft reset
////  Wire.write(0x00); // command 0x2000
////  Wire.endTransmission();
//  //delay(100);
//
//    const uint8_t CMD_LEN = 2;
//    uint8_t cmd[2] = { 0x10, 0x00 }; //continuous measurement)
////
//    if(i2c_write(sfm3300i2c, cmd, CMD_LEN, 0) != 0) {//write command
//        a = 1;
//    }
//    CyDelay(10);
//    const uint8_t DATA_LEN = 3; 
//        uint8_t data[3] = { 0 };
//        if(i2c_reRead(sfm3300i2c, data, DATA_LEN) != 0) {
//            b = 2;
//        }
//    
//    int n = 0;
//    for(;;)
//    {
//        
//        n = n+1; //for tracing sensor readouts
////        
//         
//        const uint8_t DATA_LEN = 3;
//        uint8_t data[3] = { 0 };
//        if(i2c_read(sfm3300i2c, data, DATA_LEN) != 0) {
//            b = 2;
//        }
//        
//        //CyDelay(10);  
//        uint8_t crc_check = SMF3000_CheckCrc(data, 2, data[2]);
//        uint16_t flow_raw = (data[0] << 8) | data[1];
//        float flow = ((float)flow_raw - 32768 / 120);
//        USBUART_user_check_init();
//        print_sensor_via_usbuart(n,flow_raw,a,b,crc_check);
//          
//            led_test++;
//            
//            Led_Red_Write((led_test >> 0) & 0x01);
//            Led_Green_Write((led_test >> 1) & 0x01);
//            Led_Blue_Write((led_test >> 2) & 0x01);   
//
//            if (Led_Key_Read() == 0)
//            {
//                led_test = 0;
//            }
//              
//            
//    }
}

/// USBUART Routin
void USBUART_user_check_init(void) {
    /* Host can send double SET_INTERFACE request. */
    if (0u != USBUART_IsConfigurationChanged())
    {
        /* Initialize IN endpoints when device is configured. */
        if (0u != USBUART_GetConfiguration())
        {
            /* Enumeration is done, enable OUT endpoint to receive data 
             * from host. */
            USBUART_CDC_Init();
        }
    }
}


void USBUART_user_echo(void) {
    /* Service USB CDC when device is configured. */
    if (0u != USBUART_GetConfiguration())
    {
        /* Check for input data from host. */
        if (0u != USBUART_DataIsReady())
        {
            /* Read received data and re-enable OUT endpoint. */
            count = USBUART_GetAll(buffer);

            if (0u != count)
            {
                /* Wait until component is ready to send data to host. */
                while (0u == USBUART_CDCIsReady())
                {
                }

                /* Send data back to host. */
                USBUART_PutData(buffer, count);

                /* If the last sent packet is exactly the maximum packet 
                *  size, it is followed by a zero-length packet to assure
                *  that the end of the segment is properly identified by 
                *  the terminal.
                */
                if (USBUART_BUFFER_SIZE == count)
                {
                    /* Wait until component is ready to send data to PC. */
                    while (0u == USBUART_CDCIsReady())
                    {
                    }

                    /* Send zero-length packet to PC. */
                    USBUART_PutData(NULL, 0u);
                }
            }
        }
    }
}

void print_sensor_via_usbuart(int n, float pressure, float temp, float32 flow, uint8_t crc_check_num)
{   
    sys_clock_cur_us_in_ms = (float)CySysTickGetValue() * (1/(float)cydelay_freq_hz);

    while (0u == USBUART_CDCIsReady());

    sprintf((char *)buffer, "%d. pressure: %f temperature: %f Flow: %f CRC(4): %d\n",n, pressure, temp, flow, crc_check_num);
   
    //count = sizeof(buffer);
    /* Send data back to host. */
    //USBUART_PutData(buffer, count);
    USBUART_PutString((char8 *)buffer);
}

void print_sfm(uint32_t flow, uint32_t err1, uint32_t err2) 
{   
    while (0u == USBUART_CDCIsReady());

    sprintf((char *)buffer, "Raw flow: %x\t, err1 %x\t, err2 %x\n", flow, err1,err2 );
   
    //count = sizeof(buffer);
    /* Send data back to host. */
    //USBUART_PutData(buffer, count);
    USBUART_PutString((char8 *)buffer);
}

void print_first_sfm(uint8_t read1, uint8_t read2)
{   
    while (0u == USBUART_CDCIsReady());

    sprintf((char *)buffer, "First Byte: %x\t, Second Byte: %x\t", read1, read2);
   
    //count = sizeof(buffer);
    /* Send data back to host. */
    //USBUART_PutData(buffer, count);
    USBUART_PutString((char8 *)buffer);
}

void print_status() 
{   
    while (0u == USBUART_CDCIsReady());

    sprintf((char *)buffer, "Working....");
   
    //count = sizeof(buffer);
    /* Send data back to host. */
    //USBUART_PutData(buffer, count);
    USBUART_PutString((char8 *)buffer);
}

void print_ID(uint32_t id)
{   
    while (0u == USBUART_CDCIsReady());

    sprintf((char *)buffer, "%x\n", id);
   
    //count = sizeof(buffer);
    /* Send data back to host. */
    //USBUART_PutData(buffer, count);
    USBUART_PutString((char8 *)buffer);
}

// 1ms system tick callback interrupt function
void sys_clock_ms_callback(void){
    sys_clock_cur_ms ++; // increment ms counter by 1
}

//CRC
#define POLYNOMIAL 0x131 //P(x)=x^8+x^5+x^4+1 = 100110001
//============================================================
uint8_t SMF3000_CheckCrc (uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
//============================================================
//calculates checksum for n bytes of data 
//and compares it with expected checksum
//input: data[] checksum is built based on this data
// nbrOfBytes checksum is built for n bytes of data
// checksum expected checksum
//return: error: CHECKSUM_ERROR = checksum does not match
// 0 = checksum matches
//============================================================
{
u8t crc = 0;
u8t byteCtr;
//calculates 8-Bit checksum with given polynomial
for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
{ crc ^= (data[byteCtr]);
 for (u8t bit = 8; bit > 0; --bit)
 { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
 else crc = (crc << 1);
 }
}
if (crc != checksum) return CHECKSUM_ERROR;
else return 0;
}

/* [] END OF FILE */
