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
 * Hardware based us resolution counter. 
 * Should only require CPU attention once an hour.
 * 
 * Modified slightly to fit needs of Blaser project by
 * ebcohen@andrew.cmu.edu
 * ========================================
*/

#include "`$INSTANCE_NAME`_us_clock.h"
#include "`$INSTANCE_NAME`_us_counter.h"
#include "`$INSTANCE_NAME`_us_counter_2.h"
#include "`$INSTANCE_NAME`_us_clock_driver.h"
#include "`$INSTANCE_NAME`_us_isr.h"

#include "stdio.h"

uint32_t uptime_hr;

/* Global functions for time */
uint32_t cur_time_us(){
    uint32_t lower = `$INSTANCE_NAME`_us_counter_ReadPeriod() - 
        `$INSTANCE_NAME`_us_counter_ReadCounter();
    uint32_t higher = (`$INSTANCE_NAME`_us_counter_2_ReadPeriod() - 
        `$INSTANCE_NAME`_us_counter_2_ReadCounter()) << 16;
    return higher | lower;
}

uint32_t second_rounded_us() {
    return (cur_time_us() % TIME_US_IN_SEC);
}

uint32_t uptime_s() {
    return (cur_time_s() + (uptime_hr * TIME_S_IN_HR));
}

uint32_t cur_time_ms(){
    return cur_time_us() / TIME_US_IN_MS;
}
uint32_t cur_time_s(){
    return cur_time_us() / TIME_US_IN_SEC;
}
uint32_t cur_time_m(){
    return cur_time_us() / TIME_US_IN_MIN;
}
uint32_t cur_time_hr(){
    return uptime_hr + (cur_time_us() / TIME_US_IN_HR);
}

/* Helper function for printing out time easily */
void print_time(char *buffer, uint8_t buffer_len){
    snprintf(buffer, buffer_len, "Uptime: %lu:%lu:%lu",
        cur_time_hr(),
        cur_time_m(),
        cur_time_s()
    );
}

/* Private interrupt handler 
    Note: the period of the counter is chosen so that TC
    occurs once an hour.
*/
CY_ISR(`$INSTANCE_NAME`_us_tc_handler){
    //Should keep count of long term uptime
    uptime_hr++;
    //Shouldn't need to clear this
}

/* Init function */
void `$INSTANCE_NAME`_Start(){
    `$INSTANCE_NAME`_us_counter_Start();
    `$INSTANCE_NAME`_us_counter_2_Start();
    `$INSTANCE_NAME`_us_isr_StartEx(`$INSTANCE_NAME`_us_tc_handler);
    
    uptime_hr = 0;
}

/* [] END OF FILE */
