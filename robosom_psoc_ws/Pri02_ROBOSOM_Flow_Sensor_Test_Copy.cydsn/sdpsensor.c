/*
 *  Copyright (c) 2018, Sensirion AG <joahnnes.winkelmann@sensirion.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Sensirion AG nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


//SDP3X Commands
//    const uint8_t StartContMassFlowAvg[2]     = { 0x36, 0x03 };
//    const uint8_t StartContMassFlow[2]        = { 0x36, 0x08 };
//    const uint8_t StartContDiffPressureAvg[2] = { 0x36, 0x15 };
//    const uint8_t StartContDiffPressure[2]    = { 0x36, 0x1E };
//    const uint8_t StopCont[2]                 = { 0x3F, 0xF9 };
//    const uint8_t TrigMassFlow[2]             = { 0x36, 0x24 };
//    const uint8_t TrigMassFlowStretch[2]      = { 0x37, 0x26 };
//    const uint8_t TrigDiffPressure[2]         = { 0x36, 0x2F };
//    const uint8_t TrigDiffPressureStretch[2]  = { 0x37, 0x2D };
//    const uint8_t ReadInfo1[2]                = { 0x36, 0x7C };
//    const uint8_t ReadInfo2[2]                = { 0xE1, 0x02 };
//    const uint8_t SoftReset[2]                = { 0x00, 0x06 };
//   
//    //
//    const uint32_t SDP31_PID      = 0x03010188;
//    const uint32_t SDP32_PID      = 0x03010288;
//    const uint8_t SDP31_DiffScale = 60;
//    const uint8_t SDP32_DiffScale = 240;
//    const uint8_t SDP3X_TempScale = 200;

#include "sdpsensor.h"
#include "i2chelper.h"
#include "i2chelper.c"
#include "project.h"
#include "stdio.h"
 uint8_t SDP3X_I2C_ADDR_DEFAULT = 0x21;

//---------------------------------------------------------------------------------------------------

int init()
//This function reads in the product ID, and can initiate continuous measurement
{
  // try to read product id
  const uint8_t CMD_LEN = 2;
  uint8_t cmd0[2] = { 0x36, 0x7C };
  uint8_t cmd1[2] = { 0xE1, 0x02 };

  const uint8_t DATA_LEN = 18;
  uint8_t data[18] = { 0 };

  uint8_t ret = i2c_write(SDP3X_I2C_ADDR_DEFAULT, cmd0, CMD_LEN, 0); //0 is for appendCRC
  if (ret != 0) {
    return 1;
  }
  ret = i2c_write(SDP3X_I2C_ADDR_DEFAULT, cmd1, CMD_LEN, 0);
  if (ret != 0) {
    return 2;
  }
  ret = i2c_read(SDP3X_I2C_ADDR_DEFAULT, data, DATA_LEN);
  if (ret != 0) {
    return 3;
    
    //For continuous measurement (Mass flow temp compensation): (remove readSample command in main.c forever loop, and uncomment readContSample)
    uint8_t cmd_avg_till_read[2] = { 0x36, 0x03 };
    uint8_t cmd_no_avg[2] = { 0x36, 0x08 };
    
    //start continuous measurement
    uint8_t ret = i2c_write(SDP3X_I2C_ADDR_DEFAULT, cmd_avg_till_read, CMD_LEN, 0); //0 is for appendCRC
        if (ret != 0) {
            return 4;
         }
        
        int status;
        
        status = I2C_1_MasterSendStart(SDP3X_I2C_ADDR_DEFAULT, I2C_1_READ_XFER_MODE); //send read header
    
  }
  return 0;
}

//---------------------------------------------------------------------------------------------------

int readSample()
//This function reads triggered measurements
{
uint8_t crc1_bytes[2];
uint8_t crc2_bytes[2];
uint8_t crc3_bytes[2];

  const uint8_t CMD_LEN = 2;
  uint8_t cmd[2] = { 0x36, 0x24 }; //Triggered Measurment, mass flow temp compensation (no clock stretching)

  const uint8_t DATA_LEN = 9;
  uint8_t data[9] = { 0 };

  if (i2c_write(SDP3X_I2C_ADDR_DEFAULT, cmd, CMD_LEN, 0) != 0) {
    return 1;
  }

  
  CyDelay(45); // theoretically 45ms

  if (i2c_read(SDP3X_I2C_ADDR_DEFAULT, data, DATA_LEN) != 0) {
    return 2;
  }

  //check CRC
  crc1_bytes[0] = data[0];
  crc1_bytes[1] = data[1];
  crc2_bytes[0] = data[3];
  crc2_bytes[1] = data[4];
  crc3_bytes[0] = data[6];
  crc3_bytes[1] = data[7];
  
  uint8_t crc1 = data[2];
  uint8_t crc2 = data[5];
  uint8_t crc3 = data[8];

  uint8_t crc1_check = crc8(crc1_bytes, 2);
  uint8_t crc2_check = crc8(crc2_bytes, 2);
  uint8_t crc3_check = crc8(crc3_bytes, 2);
if (crc1 != crc1_check || crc2 != crc2_check || crc3 != crc3_check) {
    return 3;
  }

//collect data
  int16_t dp_raw   = (int16_t)data[0] << 8 | data[1]; //Saves 16 bits of dp in uint16
  int16_t temp_raw = (int16_t)data[3] << 8 | data[4];
  int16_t dp_scale  = (int16_t)data[6] << 8 | data[7];
 
  mDifferentialPressure = dp_raw / (float)dp_scale;
  mTemperature = temp_raw / 200.0;

return 4;
}

float getDifferentialPressure()
{
  return mDifferentialPressure;
}

float getTemperature()
{
  return mTemperature;
}

//------------------------------------------------------------------------------

int readContSample() 
//This function reads continuous measurements (initial read header sent in initialization)
{
uint8_t crc1_bytes[2];
uint8_t crc2_bytes[2];
uint8_t crc3_bytes[2];

  const uint8_t DATA_LEN = 9;
  uint8_t data[9] = { 0 };

    int i;
    for(i=0; i<DATA_LEN; i++)
        {
            if(i < DATA_LEN-1)
            {
                data[i] = I2C_1_MasterReadByte(I2C_1_ACK_DATA); /* non-last byte send ACK */
            }
            else
            {
                data[i] = I2C_1_MasterReadByte(I2C_1_NAK_DATA); /* last byte send NACK - maybe don't to this? ACK every time? */
            }
        }

  //check CRC
  crc1_bytes[0] = data[0];
  crc1_bytes[1] = data[1];
  crc2_bytes[0] = data[3];
  crc2_bytes[1] = data[4];
  crc3_bytes[0] = data[6];
  crc3_bytes[1] = data[7];
  
  uint8_t crc1 = data[2];
  uint8_t crc2 = data[5];
  uint8_t crc3 = data[8];

  uint8_t crc1_check = crc8(crc1_bytes, 2);
  uint8_t crc2_check = crc8(crc2_bytes, 2);
  uint8_t crc3_check = crc8(crc3_bytes, 2);
if (crc1 != crc1_check || crc2 != crc2_check || crc3 != crc3_check) {
    return 3;
  }

//collect data
  int16_t dp_raw   = (int16_t)data[0] << 8 | data[1]; //Saves 16 bits of dp in uint16
  int16_t temp_raw = (int16_t)data[3] << 8 | data[4];
  int16_t dp_scale  = (int16_t)data[6] << 8 | data[7];
 
  mDifferentialPressure = dp_raw / (float)dp_scale;
  mTemperature = temp_raw / 200.0;

return 4;
}