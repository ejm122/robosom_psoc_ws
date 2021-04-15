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
#ifndef BLASER_PROTOCOL_H_
#define BLASER_PROTOCOL_H_
    
#include <stdint.h>
    
void blaser_comms_register(uint8_t (*read_char)(), uint16_t (*num_available)(), 
                           void (*put_data)(uint8_t *buffer, uint16_t count));

void blaser_comms_deregister(void);
int8_t send_comms_cmd(uint8_t cmd_ID);
int8_t read_comms_cmd(void);

#endif /* BLASER_PROTOCOL_H_ */

/* [] END OF FILE */
