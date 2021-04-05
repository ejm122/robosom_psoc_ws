/* ========================================
 *
 * Copyright CMU Biorobotics Lab
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF CMU Biorobotics Lab.
 *
 * Created 02/25/2020 by npaiva@andrew.cmu.edu
 * ========================================
*/

#include "project.h"
#include <stdbool.h>

void init_usb_comm();
void usb_print(uint8_t *buffer, uint8_t count);
void usb_put_string(const char *str);
void usb_put_char(uint8_t c);
uint8_t usb_get_char(bool *reconfigured);
uint16_t usb_num_available(bool *reconfigured);
bool usb_configuration_reinit(void);

/* [] END OF FILE */
