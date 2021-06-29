/*
 *  Copyright (c) 2015-2018, Sensirion AG <joahnnes.winkelmann@sensirion.com>
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

#include "i2chelper.h"
#include "sdpsensor.h"
#include "project.h"
#include "stdio.h"

//---------------------------------------------------------------------------------------------------------

//uint8_t i2c_write_SFM3xxx(uint8_t addr, uint16_t register_addr, uint8_t* data, uint16_t count)
////Basic I2C write function, sends write header and then commands
//{
//    int status;
//    int i;
//    status = I2C_1_MasterSendStart(addr, I2C_1_WRITE_XFER_MODE); //begin transmission in write-mode
//    if(I2C_1_MSTR_NO_ERROR == status) /* Check if transfer completed without errors */
//    {
//        /* Send register-address to write */
//        uint8_t reg_MSB = (register_addr >> 8) & 0xFF;
//        uint8_t reg_LSB = register_addr & 0xFF;
//        status = I2C_1_MasterWriteByte(reg_MSB);
//        status |= I2C_1_MasterWriteByte(reg_LSB);
//    }
//    if(status == I2C_1_MSTR_NO_ERROR) //If there is no error
//    {
//        for (i = 1; i < 500; ++i)
//        {
//        status = I2C_1_MasterSendRestart(addr, I2C_1_READ_XFER_MODE); //continue transmission in read-mode
//        /* Read array of count bytes */
//        for(i=0; i<count; i++)
//        {
//            if(i < count-1)
//            {
//                data[i] = I2C_1_MasterReadByte(I2C_1_ACK_DATA); /* non-last byte send ACK */
//            }
//            else
//            {
//                data[i] = I2C_1_MasterReadByte(I2C_1_NAK_DATA); /* last byte send NACK */
//            }
//        }
//        uint16_t flow_raw = (data[0] << 8) | data[1];
//        USBUART_user_check_init();
//        print_sensor_via_usbuart(n,flow,a,b,(float)data[2]);
//        }
//    }
//    I2C_1_MasterSendStop(); /* Send Stop */    
//    return status;
//}

//
int8_t i2c_read(uint8_t addr, uint8_t* data, uint16_t count)
//basic I2C read function, sends read header and saves data received
{
    int status;
    int i;

    /* Read data array */
    //status = I2C_1_MasterSendRestart(addr, I2C_1_READ_XFER_MODE);
    status = I2C_1_MasterSendStart(addr, I2C_1_READ_XFER_MODE); //send read header

    if(I2C_1_MSTR_NO_ERROR == status) /* Check if transfer completed without errors */
    // if(1) /* Cheating */
    {
        /* Read array of 9 bytes */
        for(i=0; i<count; i++)
        {
            if(i < count-1)
            {
                data[i] = I2C_1_MasterReadByte(I2C_1_ACK_DATA); /* non-last byte send ACK */
            }
            else
            {
                data[i] = I2C_1_MasterReadByte(I2C_1_NAK_DATA); /* last byte send NACK - maybe don't to this? ACK every time? */
            }
        }
    }

    I2C_1_MasterSendStop(); /* Send Stop */

    return status;
    // return 0; /* Cheating */
}
//-----------------------------------------------------------------------------------------------------

int8_t i2c_reRead(uint8_t addr, uint8_t* data, uint16_t count)
//basic I2C read function, sends read header and saves data received
{
    int status;
    int i;

    /* Read data array */
    //status = I2C_1_MasterSendRestart(addr, I2C_1_READ_XFER_MODE);
    status = I2C_1_MasterSendRestart(addr, I2C_1_READ_XFER_MODE); //send read header

    if(I2C_1_MSTR_NO_ERROR == status) /* Check if transfer completed without errors */
    // if(1) /* Cheating */
    {
        /* Read array of 9 bytes */
        for(i=0; i<count; i++)
        {
            if(i < count-1)
            {
                data[i] = I2C_1_MasterReadByte(I2C_1_ACK_DATA); /* non-last byte send ACK */
            }
            else
            {
                data[i] = I2C_1_MasterReadByte(I2C_1_NAK_DATA); /* last byte send NACK - maybe don't to this? ACK every time? */
            }
        }
    }

    I2C_1_MasterSendStop(); /* Send Stop */

    return status;
    // return 0; /* Cheating */
}

//-----------------------------------------------------------------------------------------------------------
int8_t i2c_firstRead(uint8_t addr, uint8_t* data, uint16_t count)
//basic I2C read function, sends read header and saves data received
{
    int status;
    int i;

    /* Read data array */
    
    //status = I2C_1_MasterSendRestart(addr, I2C_1_READ_XFER_MODE);
    status = I2C_1_MasterSendRestart_Modified(addr, I2C_1_READ_XFER_MODE); //send read header
    //status = I2C_1_MasterSendStart_Modified(addr, I2C_1_READ_XFER_MODE); //send read header
    if(I2C_1_MSTR_NO_ERROR == status)
    //if(I2C_1_MSTR_ERR_LB_NAK == status) /* Check if transfer completed without errors */
    //if(1) /* Cheating */
    {
        /* Read array of 9 bytes */
        for(i=0; i<count; i++)
        {
            if(i < count-1)
            {
                data[i] = I2C_1_MasterReadByte_Modified(I2C_1_ACK_DATA); /* non-last byte send ACK */
            }
            else
            {
                data[i] = I2C_1_MasterReadByte_Modified(I2C_1_NAK_DATA); /* last byte send NACK - maybe don't to this? ACK every time? */
            }
        }
    }

    I2C_1_MasterSendStop(); /* Send Stop */

    return status;
    // return 0; /* Cheating */
}

//-----------------------------------------------------------------------------------------------------------

int8_t i2c_write(uint8_t addr, const uint8_t* data, uint16_t count, int appendCrc)
//Basic I2C write function, sends write header and then commands
{
    int status;
    int i;
    
    status = I2C_1_MasterSendStart(addr, I2C_1_WRITE_XFER_MODE); //begin transmission
    if(I2C_1_MSTR_NO_ERROR == status) /* Check if transfer completed without errors */
    {
 /* Send array of 2 bytes (commands) */
        for(i=0; i<count; i++) {
        status = I2C_1_MasterWriteByte(data[i]);
        
        if(status != I2C_1_MSTR_NO_ERROR) //If there is an error
            {
            break;
            }
        }
        if (appendCrc) { //if you are appending crc to end of command
            uint8_t crc = crc8(data, count); //make crc
            status = I2C_1_MasterWriteByte(crc);//write crc
            
      }
    }
 //I2C_1_MasterSendStop(); /* Send Stop */    

return status;
}

//------------------------------------------------------------------------------------------    

uint8_t crc8(const uint8_t data[], uint8_t len)
//Generates check sum based on data received (compare to check sum sent with data)
{
  // adapted from SHT21 sample code from http://www.sensirion.com/en/products/humidity-temperature/download-center/

  uint8_t crc = 0xff;
  uint8_t byteCtr;
  for (byteCtr = 0; byteCtr < len; ++byteCtr) {
    crc ^= (data[byteCtr]);
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

//--------------------------------------------------------------
/*******************************************************************************
* Function Name: I2C_1_MasterSendRestart_Modified
********************************************************************************
*
* Summary:
*  Generates ReStart condition and sends slave address with read/write bit.
*
* Parameters:
*  slaveAddress:  7-bit slave address.
*  R_nW:          Zero, send write command, non-zero send read command.
*
* Return:
*  Status error - Zero means no errors.
*
* Side Effects:
*  This function is entered without a "byte complete" bit set in the I2C_CSR
*  register. It does not exit until it is set.
*
* Global variables:
*  I2C_1_state - The global variable used to store a current state of
*                           the software FSM.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I2C_1_MasterSendRestart_Modified(uint8 slaveAddress, uint8 R_nW)
      
{
    uint8 errStatus;

    errStatus = I2C_1_MSTR_NOT_READY;

    /* Check if START condition was generated */
    if(I2C_1_CHECK_MASTER_MODE(I2C_1_MCSR_REG))
    {
        /* Set address and read/write flag */
        slaveAddress = (uint8) (slaveAddress << I2C_1_SLAVE_ADDR_SHIFT);
        if(0u != R_nW)
        {
            slaveAddress |= I2C_1_READ_FLAG;
            I2C_1_state = I2C_1_SM_MSTR_RD_ADDR;
        }
        else
        {
            I2C_1_state = I2C_1_SM_MSTR_WR_ADDR;
        }

        /* Hardware actions: write address and generate ReStart */
        I2C_1_DATA_REG = slaveAddress;
        I2C_1_GENERATE_RESTART_MANUAL;

        /* Wait until address has been transferred */
        while(I2C_1_WAIT_BYTE_COMPLETE(I2C_1_CSR_REG))
        {
        }

    #if(I2C_1_MODE_MULTI_MASTER_ENABLED)
        if(I2C_1_CHECK_LOST_ARB(I2C_1_CSR_REG))
        {
            I2C_1_BUS_RELEASE_MANUAL;

            /* Master lost arbitrage: reset FSM to IDLE */
            I2C_1_state = I2C_1_SM_IDLE;
            errStatus = I2C_1_MSTR_ERR_ARB_LOST;
        }
        else
    #endif /* (I2C_1_MODE_MULTI_MASTER_ENABLED) */

        if(I2C_1_CHECK_ADDR_NAK(I2C_1_CSR_REG))
        {
            /* Address has been NACKed: reset FSM to IDLE */
            //I2C_1_state = I2C_1_SM_IDLE;
            //errStatus = I2C_1_MSTR_ERR_LB_NAK;
            errStatus = I2C_1_MSTR_NO_ERROR;
        }
        else
        {
            /* ReStart was sent without errors */
            errStatus = I2C_1_MSTR_NO_ERROR;
        }
    }

    return(errStatus);
}

/*******************************************************************************
* Function Name: I2C_1_MasterReadByte_Modified
********************************************************************************
*
* Summary:
*  Reads one byte from a slave and ACK or NACK the transfer. A valid Start or
*  ReStart condition must be generated before this call this function. Function
*  do nothing if Start or Restart condition was failed before call this
*  function.
*
* Parameters:
*  acknNack:  Zero, response with NACK, if non-zero response with ACK.
*
* Return:
*  Byte read from slave.
*
* Side Effects:
*  This function is entered without a "byte complete" bit set in the I2C_CSR
*  register. It does not exit until it is set.
*
* Global variables:
*  I2C_1_state - The global variable used to store a current
*                           state of the software FSM.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I2C_1_MasterReadByte_Modified(uint8 acknNak) 
{
    uint8 theByte;

    theByte = 0u;

    /* Check if START condition was generated */
    if(I2C_1_CHECK_MASTER_MODE(I2C_1_MCSR_REG))
    {
        /* When address phase needs to release bus and receive byte,
        * then decide ACK or NACK
        */
        if(I2C_1_SM_MSTR_RD_ADDR == I2C_1_state)
        {
            I2C_1_READY_TO_READ_MANUAL;
            I2C_1_state = I2C_1_SM_MSTR_RD_DATA;
        }

        /* DELTED THIS: Wait until data byte has been received */
        //while(I2C_1_WAIT_BYTE_COMPLETE(I2C_1_CSR_REG))
        //{
        //}

        theByte = I2C_1_DATA_REG;

        /* Command ACK to receive next byte and continue transfer.
        *  Do nothing for NACK. The NACK will be generated by
        *  Stop or ReStart routine.
        */
        if(acknNak != 0u) /* Generate ACK */
        {
            I2C_1_ACK_AND_RECEIVE_MANUAL;
        }
        else              /* DELETED THIS: Do nothing for the follwong NACK */
        {
            //I2C_1_state = I2C_1_SM_MSTR_HALT;
            I2C_1_ACK_AND_RECEIVE_MANUAL;
        }
    }

    return(theByte);
}

/*******************************************************************************
* Function Name: I2C_1_MasterSendStart_Modified
********************************************************************************
*
* Summary:
*  Generates Start condition and sends slave address with read/write bit.
*
* Parameters:
*  slaveAddress:  7-bit slave address.
*  R_nW:          Zero, send write command, non-zero send read command.
*
* Return:
*  Status error - Zero means no errors.
*
* Side Effects:
*  This function is entered without a "byte complete" bit set in the I2C_CSR
*  register. It does not exit until it is set.
*
* Global variables:
*  I2C_1_state - The global variable used to store a current state of
*                           the software FSM.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I2C_1_MasterSendStart_Modified(uint8 slaveAddress, uint8 R_nW)
      
{
    uint8 errStatus;

    errStatus = I2C_1_MSTR_NOT_READY;

    /* If IDLE, check if bus is free */
    if(I2C_1_SM_IDLE == I2C_1_state)
    {
        /* If bus is free, generate Start condition */
        if(I2C_1_CHECK_BUS_FREE(I2C_1_MCSR_REG))
        {
            /* Disable interrupt for manual master operation */
            I2C_1_DisableInt();

            /* Set address and read/write flag */
            slaveAddress = (uint8) (slaveAddress << I2C_1_SLAVE_ADDR_SHIFT);
            if(0u != R_nW)
            {
                slaveAddress |= I2C_1_READ_FLAG;
                I2C_1_state = I2C_1_SM_MSTR_RD_ADDR;
            }
            else
            {
                I2C_1_state = I2C_1_SM_MSTR_WR_ADDR;
            }

            /* Hardware actions: write address and generate Start */
            I2C_1_DATA_REG = slaveAddress;
            I2C_1_GENERATE_START_MANUAL;

            /* Wait until address is transferred */
            while(I2C_1_WAIT_BYTE_COMPLETE(I2C_1_CSR_REG))
            {
            }

        #if(I2C_1_MODE_MULTI_MASTER_SLAVE_ENABLED)
            if(I2C_1_CHECK_START_GEN(I2C_1_MCSR_REG))
            {
                I2C_1_CLEAR_START_GEN;

                /* Start condition was not generated: reset FSM to IDLE */
                I2C_1_state = I2C_1_SM_IDLE;
                errStatus = I2C_1_MSTR_ERR_ABORT_START_GEN;
            }
            else
        #endif /* (I2C_1_MODE_MULTI_MASTER_SLAVE_ENABLED) */

        #if(I2C_1_MODE_MULTI_MASTER_ENABLED)
            if(I2C_1_CHECK_LOST_ARB(I2C_1_CSR_REG))
            {
                I2C_1_BUS_RELEASE_MANUAL;

                /* Master lost arbitrage: reset FSM to IDLE */
                I2C_1_state = I2C_1_SM_IDLE;
                errStatus = I2C_1_MSTR_ERR_ARB_LOST;
            }
            else
        #endif /* (I2C_1_MODE_MULTI_MASTER_ENABLED) */

            if(I2C_1_CHECK_ADDR_NAK(I2C_1_CSR_REG))
            {
                /* Address has been NACKed: reset FSM to IDLE */
                //I2C_1_state = I2C_1_SM_IDLE;
                //errStatus = I2C_1_MSTR_ERR_LB_NAK;
                 /* Start was sent without errors */
                errStatus = I2C_1_MSTR_NO_ERROR;
            }
            else
            {
                /* Start was sent without errors */
                errStatus = I2C_1_MSTR_NO_ERROR;
            }
        }
        else
        {
            errStatus = I2C_1_MSTR_BUS_BUSY;
        }
    }

    return(errStatus);
}

