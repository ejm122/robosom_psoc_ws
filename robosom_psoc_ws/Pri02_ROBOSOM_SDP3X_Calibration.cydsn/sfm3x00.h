/*
 * =====================================================================================
 *
 *       Filename:  SFM3X000.h
 *
 *    Description:  Senserion SFM3X00 library
 *
 *        Version:  1.0
 *        Created:  04/16/2020 16:59:40
 *
 *   Organization:  Public Invention
 *
 *        License:  Senserion BSD 3-Clause License
 *
 * =====================================================================================
 */
#include <stdint.h>
#include "stdio.h"
#include "project.h"

#ifndef SFM3X00_H
#define SFM3X00_H
    
//// max and min for SFM3400
//#define SFM3400_MIN 0x80   // 128 decimal
//#define SFM3400_MAX 0xFF80 // 65408 decimal
//
//// max and min for SFM3200
//#define SFM3200_MIN 0x0F4F // 3913 decimal
//#define SFM3200_MAX 0xFB0C // 64268 decimal
//    
//// max and min for SFM3300:

    
// I2C commands
// see application note: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/5_Mass_Flow_Meters/Sensirion_Mass_Flow_Meters_Application_Note_SFM3xxx.pdf
#define sfmSensorAddress                0x81
#define READ_SCALE_FACTOR               0x30DE
#define READ_FLOW_OFFSET                0x30DF
#define START_CONTINUOUS_MEASUREMENT    0x1000
#define SOFT_RESET                      0x2000
// ** see application note before using temperature measurement
#define START_TEMPERATURE_MEASUREMENT   0x1001

#define READ_SERIAL_NUMBER_U            0x31AE
#define READ_SERIAL_NUMBER_L            0x31AF
#define READ_ARTICLE_NUMBER_U           0x31E3
#define READ_ARTICLE_NUMBER_L           0x31E4
    
 // USBUART
#define USBFS_DEVICE    (0u)
#define USBUART_BUFFER_SIZE (256u)
uint16 count;
uint8 buffer[USBUART_BUFFER_SIZE];
void USBUART_user_check_init(void);
//void USBUART_user_echo(void);

// Testing Function
void print_sensor_via_usbuart(int n, float pressure, float temp, float32 flow, uint8_t crc_check_num);

// System clock
//uint32 sys_clock_cur_ms = 0;
//float sys_clock_cur_us_in_ms = 0;
float temp;
float pressure;

void sys_clock_ms_callback(); // 1ms callback interrupt function



uint32_t sfmSerialNumber;
uint32_t sfmArticleNumber;
float    sfmFlowOffset;
float    sfmFlowScale;
uint16_t sfmMinFlow;
uint16_t sfmMaxFlow;

    // send I2C command to sensor
    //void sendCommand(uint16_t command);

    // read 2 bytes of data from the sensor
    //uint16_t readData();

    // reads offste values and starts flow measurment
    void sfmBegin();

    // initialize sensor with flow offset and scale values
    void sfmSetupFlowSensor();

    // reads raw flow and calculates the real flow based on sensor values
    // returns flow in slm
    float sfmReadFlow();

    // read the serial number
    uint32_t sfmRequestSerialNumber();

    // read the article number
    uint32_t sfmRequestArticleNumber();

    // read the scale factor
    uint16_t sfmRequestScaleFactor();

    // read sensor flow offset avlue
    uint16_t sfmRequestOffset();

    // start measuring flow
    void sfmStartContinuousMeasurement();

    // returns 0 if measurment is within bounds
    // returns 1 if measurment is not within bounds
    
//    int checkRange(uint16_t rawFlow); //returns 0/1
//    int checkRange(float computedFlow); //returns 0/1

#endif