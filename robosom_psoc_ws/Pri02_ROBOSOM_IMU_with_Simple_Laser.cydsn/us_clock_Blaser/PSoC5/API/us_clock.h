/* ========================================
 *
 * Copyright CMU Biorobotics Lab
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF CMU Biorobotics Lab.
 *
 * Created 12/12/2019 by npaiva@andrew.cmu.edu
 * 
 * This file is used to generate a microsecond-accurate clock
 * in hardware. Should only require CPU intervention once every
 * hour. Based on a counter tied to a 1 MHz clock. The period of
 * the counter is chosen such that it wraps around once an hour. 
 * 
 * Another 32 bit variable is used to keep track of how many hours
 * the clock has been up for. 
 * 
 * Total uptime should not exceed 490,293 years.
 * ========================================
*/

#include <stdint.h>

#define TIME_US_IN_MS       (1000u)
#define TIME_US_IN_SEC      (1000u * TIME_US_IN_MS)
#define TIME_US_IN_MIN      (60u * TIME_US_IN_SEC)
#define TIME_US_IN_HR       (60u * TIME_US_IN_MIN)
#define TIME_S_IN_HR        (3600u)

uint32_t    cur_time_us();
uint32_t    cur_time_ms();
uint32_t    cur_time_s();
uint32_t    cur_time_m();
uint32_t    cur_time_hr();
uint32_t    uptime_s();
uint32_t    second_rounded_us();
void        print_time(char *buffer, uint8_t buffer_len);

void `$INSTANCE_NAME`_Start();

/* [] END OF FILE */
