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

int main(void)
{   uint8 ch;
    uint8 a;
    //CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
        UART_Start();
        UART_UartPutString("Testing out Tutorials");
    for(;;)
    {
        /* Place your application code here. */
       ch = UART_UartGetChar();
    
        if(0u != ch)
        {   UART_UartPutChar(ch);
        }
        
         //Pin_Red_Write( ~ Pin_Red_Read() ); //Toggle red
        //CyDelay(2000); //0.5 second delay
         
    }
}

/* [] END OF FILE */
