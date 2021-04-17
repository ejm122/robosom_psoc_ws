/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 *
 * Created 3/26/2021 by ebcohen@andrew.cmu.edu
 * ========================================
*/
#ifndef BLASER_MESSAGES_PSOC_H_
#define BLASER_MESSAGES_PSOC_H_
    
#include <stdint.h>
#include "blaser_messages.h"

/* Public MCU-specific function definitions */
uint8_t get_laser_val(void);
uint8_t get_pulse_state(void);

#endif /* BLASER_MESSAGES_PSOC_H_ */
/* [] END OF FILE */
