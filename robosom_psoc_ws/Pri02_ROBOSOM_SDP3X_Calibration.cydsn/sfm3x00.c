/*
 * =====================================================================================
 *
 *       Filename:  SFM3X000.cpp
 *
 *    Description:  Senserion SFM3X00 library
 *
 *        Version:  1.0
 *        Created:  04/16/2020 16:59:40
 *
 *   Organization:  Public Invention
 *
 *        License:  Sensirion BSD 3-Clause License
 *
 * =====================================================================================
 */

#include "sfm3x00.h"
#include "sdpsensor.h"
#include "i2chelper.h"
#include "i2chelper.c"
#include "project.h"
#include "stdio.h"
float sys_clock_cur_us_in_ms = 0;
uint32 sys_clock_cur_ms = 0;

//* From Arduino Library /////////


//sendCommand(uint16_t command) //sameas int8_t i2c_write(uint8_t addr, const uint8_t* data, uint16_t count, int appendCrc)
// i2c_write(sfmSensorAddress, sfmData, 2, 0)
//{
//  //Serial.println();
//  //Serial.println(command, HEX);
//  uint8_t b1 = (command & 0xFF00) >> 8;
//  //Serial.println(b1, HEX);
//  uint8_t b0 = command & 0x00FF;
//  //Serial.println(b0, HEX);
//
//  Wire.beginTransmission(byte(this->sensorAddress));
//  Wire.write(byte(b1));
//  Wire.write(byte(b0));
//  Wire.endTransmission();
//}


//readData()// same as: int8_t i2c_read(uint8_t addr, uint8_t* data, uint16_t count) 
//int8_t i2c_read(sfmSensorAddress, sfmData, 2) //reads two pieces of data
//{
//  uint8_t b[2];
//
//  Wire.requestFrom(this->sensorAddress, 2);
//
//  b[1] = Wire.read();
//  b[0] = Wire.read();
//
//  uint16_t c {0};
//  c = (b[1] << 8) | b[0];
//
//  //Serial.println(c, HEX);
//  //Serial.println(c, DEC);
//
//  return c;
//}

/* End from Arduino Library */



uint32_t sfmRequestSerialNumber()
{
    uint8_t data[6] = { 0 };
   i2c_write_SFM3xxx(sfmSensorAddress,READ_SERIAL_NUMBER_U,data,6);

        uint16_t upperBytes = (data[0] << 8) | data[1];
        uint16_t lowerBytes = (data[3] << 8) | data[4];
   
        sfmSerialNumber = ((uint32_t)upperBytes << 16) | lowerBytes;
        USBUART_user_check_init();
        print_sensor_via_usbuart((float)sfmSerialNumber,1,1,1,1);
        return sfmSerialNumber;

//  uint16_t command = READ_SERIAL_NUMBER_U;
//  const uint8_t CMD_LEN = 2;
//  uint8_t b1 = (command & 0xFF00) >> 8;
//  uint8_t b0 = command & 0x00FF;
//  uint8_t cmd[2] = { b1, b0 }; //break command into two bits
//
//  //sendCommand(READ_SERIAL_NUMBER_U);
//  if(i2c_write(sfmSensorAddress, cmd, CMD_LEN, 0) != 0) { //if not written correctly
//    USBUART_user_check_init();
//    print_sensor_via_usbuart(1,1,1,1,1);
//   }
//    
//  //uint16_t upperBytes = readData(); - read in first 2 bytes of serial number
//    const uint8_t DATA_LEN = 3;
//    uint8_t data[3] = { 0 };
//    
//    if(i2c_read(sfmSensorAddress, data, DATA_LEN) !=0) { //if not read correctly
//        USBUART_user_check_init();
//        print_sensor_via_usbuart(2,2,2,2,2);
//    }
//    uint16_t upperBytes = (data[1] << 8) | data[0];
//
//
//  uint16_t command2 = READ_SERIAL_NUMBER_L;
//  const uint8_t CMD_LEN2 = 2;
//  uint8_t b1_2 = (command2 & 0xFF00) >> 8;
//  uint8_t b0_2 = command2 & 0x00FF;
//  uint8_t cmd2[2] = { b1_2, b0_2 };
//
//  //  sendCommand(READ_SERIAL_NUMBER_L);
//    if(i2c_write(sfmSensorAddress, cmd2, CMD_LEN2, 0) != 0) { //if not written correctly
//        USBUART_user_check_init();
//        print_sensor_via_usbuart(3,3,3,3,3);
//    }
//
//  //uint16_t lowerBytes = readData();
//    const uint8_t DATA_LEN2 = 3;
//    uint8_t data2[3] = { 0 };
//    if(i2c_read(sfmSensorAddress, data2, DATA_LEN2) != 0) { //if not read correctly
//        USBUART_user_check_init();
//        print_sensor_via_usbuart(4,4,4,4,4);
//    }
//    uint16_t lowerBytes = (data2[1] << 8) | data2[0];
//    
//  sfmSerialNumber = ((uint32_t)upperBytes << 16) | lowerBytes;
//
}


//uint32_t sfmRequestArticleNumber()
//{
//  sendCommand(READ_ARTICLE_NUMBER_U);
//
//  uint16_t upperBytes = readData();
//
//  sendCommand(READ_ARTICLE_NUMBER_L);
//
//  uint16_t lowerBytes = readData();
//
//  uint32_t articleNumber {0};
//
//  articleNumber = ((uint32_t)upperBytes << 16) | lowerBytes;
//
// return articleNumber;
//}


//
//uint16_t sfmRequestScaleFactor()
//{
//    
//  //sendCommand(READ_SCALE_FACTOR);
//  uint16_t command = READ_SCALE_FACTOR;
//  const uint8_t CMD_LEN = 2;
//  uint8_t b1 = (command & 0xFF00) >> 8;
//  uint8_t b0 = command & 0x00FF;
//  uint8_t cmd[2] = { b1, b0 };
//  i2c_write(sfmSensorAddress, cmd, CMD_LEN, 0);
//
//  //int16_t scaleFactor = readData();
//    const uint8_t DATA_LEN = 2;
//    uint8_t data[2] = { 0 };
//   i2c_read(sfmSensorAddress, data, DATA_LEN);
//
//    uint16_t sfmScaleFactor;
//    sfmScaleFactor = (data[1] << 8) | data[0];
//
//
//  return sfmScaleFactor;
//}
//
//
//uint16_t sfmRequestOffset()
//{
////  sendCommand(READ_FLOW_OFFSET);
//  uint16_t command = READ_FLOW_OFFSET;
//  const uint8_t CMD_LEN = 2;
//  uint8_t b1 = (command & 0xFF00) >> 8;
//  uint8_t b0 = command & 0x00FF;
//  uint8_t cmd[2] = { b1, b0 };
//  i2c_write(sfmSensorAddress, cmd, CMD_LEN, 0);
////
////  uint16_t offset = readData();
//    const uint8_t DATA_LEN = 2;
//    uint8_t data[2] = { 0 };
//   i2c_read(sfmSensorAddress, data, DATA_LEN);
//
//    uint16_t sfmFlowOffset;
//    sfmFlowOffset = (data[1] << 8) | data[0];
//    
//  return sfmFlowOffset;
//}
//

void sfmSetupFlowSensor()
{
  sfmSerialNumber = (float)sfmRequestSerialNumber();
  //sfmArticleNumber = requestArticleNumber();
  //sfmFlowOffset   = (float)sfmRequestOffset();
  //sfmFlowScale    = (float)sfmRequestScaleFactor();

//  if(this-> flowScale == 800.0)
//  {
//    this->minFlow = SFM3400_MIN;
//    this->maxFlow = SFM3400_MAX;
//  }
//  else if(this-> flowScale == 120.0)
//  {
//    this->minFlow = SFM3200_MIN;
//    this->maxFlow = SFM3200_MAX;
//  }
}

//
//void sfmStartContinuousMeasurement()
//{
//  //sendCommand(START_CONTINUOUS_MEASUREMENT);
//  uint16_t command = START_CONTINUOUS_MEASUREMENT;
//  const uint8_t CMD_LEN = 2;
//  uint8_t b1 = (command & 0xFF00) >> 8;
//  uint8_t b0 = command & 0x00FF;
//  uint8_t cmd[2] = { b1, b0 };
//  i2c_write(sfmSensorAddress, cmd, CMD_LEN, 0);
//}


void sfmBegin()
{
  sfmSetupFlowSensor();
  //sfmStartContinuousMeasurement();
}



//float sfmReadFlow()
//{
//  //uint16_t rawFlow = readData();
//    const uint8_t DATA_LEN = 2;
//    uint8_t data[2] = { 0 };
//    i2c_read(sfmSensorAddress, data, DATA_LEN);
//    int16_t rawFlow   = (int16_t)data[0] << 8 | data[1];
//
//  float flow = ((float)rawFlow - sfmFlowOffset) / sfmFlowScale;
//
//  return flow;
//}


//bool SFM3X00::checkRange(uint16_t rawFlow)
//{
//  return ((rawFlow <= this->minFlow) || (rawFlow >= this-> maxFlow));
//}
//
//bool SFM3X00::checkRange(float computedFlow)
//{
//  float min_f = ((float)this->minFlow - this->flowOffset) / this->flowScale;
//  float max_f = ((float)this->maxFlow - this->flowOffset) / this->flowScale;
//
//  return ((computedFlow <= min_f) || (computedFlow >= max_f));
//}

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

// 1ms system tick callback interrupt function
void sys_clock_ms_callback(void){
    sys_clock_cur_ms ++; // increment ms counter by 1
}