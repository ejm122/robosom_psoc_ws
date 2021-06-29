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
//*/
//#include<FlowSensor.h>
/* [] END OF FILE */

//To read (1) Write the register adress you want to read from to sensor
        //(2) Use MasterReadBuf to store what is contained in register into the read buffer

//        uint16 Flow_Read() {
//            uint8 Write_Buf[1] = {0}; //buffer which stores register that must be read from
//            Write_Buf[0] = FLOW_ADDR; //idk there is no specific register so just the secondary address, put this in the write buffer
//            
//            uint16 Read_Buf[1] = {0}; //Buffer that will store the value read from the register
//            
//            I2C_1_MasterWriteBuf(FLOW_ADDR, (uint8 *)Write_Buf, 1,I2C_1_MODE_NO_STOP); //Send write buffer
//            while(I2C_1_MasterStatus() & I2C_1_MSTAT_WR_CMPLT) == 0)() // while writing still happening
//            
//            I2C_1_MasterReadBuf(FLOW_ADDR, (uint8 *)Read_Buf, 1, I2C_1_MODE_REPEAT_START); //not sure what this does?
//            while((I2C_1_MasterStatus() & I2C_1_MSTAT_RD_CMPLT) == 0)()
//            
//            return Read_Buf[0]; //return what it just read
//        }
//        
//        void flow_Write(const uint8_t value) { //value is the command
//            uint8 Write_Buf[2] = {0}; //make write buffer of size 2 to contain both the address adn the command
//            Write_Buf[0] = FLOW_ADDR;
//            Write_Buf[1] = value;
//            
//            I2C_1_MasterWriteBuf(FLOW_ADDR, (uint8 *)Write_Buf, 2, I2C_1_MODE_COMPLETE_XFER); //mode is complete transfer
//            while((I2C_1_MasterStatus() & I2C_1_MSTAT_WR_CMPLT) == 0)() //wait till write is complete
//            
//            return;
//        }
//