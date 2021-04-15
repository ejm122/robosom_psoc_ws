/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * blaser_messages.c
 *
 * Created by ebcohen@andrew.cmu.edu
 *
 * This file provides blaser-interfacing commands
 * for MCU-side communication.
 *
 * Some PC-side communication functions are included
 * for reference, and may be augmented to fit
 * specific implementation needs.
 * 
 * While functionality of blaser_messages.c can differ
 * between the PC/MCU, blaser_messages.h MUST be kept
 * in sync. It may be worth investigating automated ways
 * to ensure versions of this file match between MCU/PC
 * in the future.
 * 
 * Some of the getter/setter functions here may
 * be better suited in seperate files.
 * ========================================
*/
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <math.h>

#include "blaser_messages.h"
// Include BMI160 Library and PSOC HAL
#include "../Library/BMI160/bmi160.h"
#include "../Library/BMI160/bmi160_psoc.h"
#include "blaser_protocol.h"
#include "error_codes.h"

/** @brief Reference global system clock */
extern uint32_t sys_clock_cur_ms;

/* Static definitions */
static int8_t handle_post_imu_data(uint8_t *out_buf, uint16_t len);
static int8_t read_msg_psoc(uint8_t cmd_ID, uint8_t *in_buf);
static uint16_t send_msg_psoc(uint8_t cmd_ID, uint8_t *out_buf, uint16_t len);

/** @brief Global vars that can be accessed via get cmds */
uint8_t led_val;
uint8_t laser_val = 1;

/** @brief Maps from MSG_ID to the packet's length */
uint16_t get_len_MSG_ID(uint8_t cmd_ID) {
    switch (cmd_ID) 
    {
        case BLSR_POST_IMU_DATA:
            return BLASER_POST_IMU_DATA_SIZE;
        default:
            return 0;
    }
    return 0;
}

/** @brief Wrapper for PSoC-Side. Modify this file to support read_msg_pc
           for communication from PC -> PSoC */
int8_t read_msg(uint8_t cmd_ID, uint8_t *in_buf) 
{
    return read_msg_psoc(cmd_ID, in_buf);   
}

int16_t send_msg(uint8_t cmd_ID, uint8_t *out_buf, uint16_t len) 
{
    return send_msg_psoc(cmd_ID, out_buf, len);
}


int8_t send_post_imu_data(uint8_t *out_buf, uint16_t len) 
{
    int8_t rslt;
    struct bmi160_dev sensor;
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;
    union blaser_post_imu_data_msg msg;
    uint32_t cur_time_ms;
    
    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_post_imu_data) != BLASER_POST_IMU_DATA_SIZE));
    
    rslt = BMI160_OK;//bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &sensor);
    cur_time_ms = sys_clock_cur_ms;
    
    if (rslt != BMI160_OK) 
    {
        return ERR_SENSOR_READ;
    }
    
    if (len < BLASER_POST_IMU_DATA_SIZE) 
    {
        return ERR_BUFF_OVERFLOW;   
    }
    
    /* Initialize the message struct- Order does not matter with identifier usage */
    msg.struct_data = (struct blaser_post_imu_data) 
    {
        .sys_clock_cur_ms_BYTE_3 = GET_BYTE_IDX(cur_time_ms, 3),
        .sys_clock_cur_ms_BYTE_2 = GET_BYTE_IDX(cur_time_ms, 2),
        .sys_clock_cur_ms_BYTE_1 = GET_BYTE_IDX(cur_time_ms, 1),
        .sys_clock_cur_ms_BYTE_0 = GET_BYTE_IDX(cur_time_ms, 0),
        .accel_x_BYTE_3 = 1,//GET_BYTE_IDX(accel.x, 3),
        .accel_x_BYTE_2 = 2,//GET_BYTE_IDX(accel.x, 2),
        .accel_x_BYTE_1 = 3,//GET_BYTE_IDX(accel.x, 1),
        .accel_x_BYTE_0 = 4,//GET_BYTE_IDX(accel.x, 0),
        .accel_y_BYTE_3 = 5,//GET_BYTE_IDX(accel.y, 3),
        .accel_y_BYTE_2 = 6,//GET_BYTE_IDX(accel.y, 2),
        .accel_y_BYTE_1 = 7,//GET_BYTE_IDX(accel.y, 1),
        .accel_y_BYTE_0 = 8,//GET_BYTE_IDX(accel.y, 0),
        .accel_z_BYTE_3 = 9,//GET_BYTE_IDX(accel.z, 3),
        .accel_z_BYTE_2 = 10,//GET_BYTE_IDX(accel.z, 2),
        .accel_z_BYTE_1 = 11,//GET_BYTE_IDX(accel.z, 1),
        .accel_z_BYTE_0 = 12,//GET_BYTE_IDX(accel.z, 0),
        .gyro_x_BYTE_3 = 13,//GET_BYTE_IDX(gyro.x, 3),
        .gyro_x_BYTE_2 = 14,//GET_BYTE_IDX(gyro.x, 2),
        .gyro_x_BYTE_1 = 15,//GET_BYTE_IDX(gyro.x, 1),
        .gyro_x_BYTE_0 = 16,//GET_BYTE_IDX(gyro.x, 0),
        .gyro_y_BYTE_3 = 17,//GET_BYTE_IDX(gyro.y, 3),
        .gyro_y_BYTE_2 = 18,//GET_BYTE_IDX(gyro.y, 2),
        .gyro_y_BYTE_1 = 19,//GET_BYTE_IDX(gyro.y, 1),
        .gyro_y_BYTE_0 = 20,//GET_BYTE_IDX(gyro.y, 0),
        .gyro_z_BYTE_3 = 21,//GET_BYTE_IDX(gyro.z, 3),
        .gyro_z_BYTE_2 = 22,//GET_BYTE_IDX(gyro.z, 2),
        .gyro_z_BYTE_1 = 23,//GET_BYTE_IDX(gyro.z, 1),
        .gyro_z_BYTE_0 = 24,//GET_BYTE_IDX(gyro.z, 0),
    };
    
    /* Write msg data to output buffer */
    memcpy(out_buf, msg.data, BLASER_POST_IMU_DATA_SIZE);
    return NO_ERR;
}

int8_t recv_set_laser_led_pwm(uint8_t *in_buf)
{
    union blaser_set_laser_led_pwm_msg msg;
    
    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_set_laser_led_pwm) != BLASER_SET_LASER_LED_PWM_SIZE));
    
    memcpy(msg.data, in_buf, BLASER_SET_LASER_LED_PWM_SIZE);
    
    // Masking to fit previous declaration, can use full byte if needed.
    laser_val = msg.struct_data.laser_pwm & 0b01111111; // Mask for last 7 bits;
    led_val = msg.struct_data.led_pwm & 0b01111111; // Mask for last 7 bits;
    
    return NO_ERR;
}

int8_t send_set_laser_led_pwm(uint8_t *out_buf, uint16_t len) 
{
    union blaser_set_laser_led_pwm_msg msg;
    
    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_set_laser_led_pwm) != BLASER_SET_LASER_LED_PWM_SIZE));
    
    if (len < BLASER_SET_LASER_LED_PWM_SIZE) 
    {
        return ERR_BUFF_OVERFLOW;   
    }
    
    // For demo purposes - Could set vals to some GUI-configurable inputs
    msg.struct_data = (struct blaser_set_laser_led_pwm) 
    {
        .laser_pwm = 0x50, 
        .led_pwm = 0x60,
    };
    
    memcpy(out_buf, msg.data, BLASER_SET_LASER_LED_PWM_SIZE);
    
    return NO_ERR;
}


uint8_t get_laser_val(void)
{
    return laser_val;
}

uint8_t get_led_val(void)
{
    return led_val;   
}

static int8_t read_msg_psoc(uint8_t cmd_ID, uint8_t *in_buf) {
    uint8_t status = NO_ERR;

    // Length check already occured before invoking read_msg.
    switch (cmd_ID) 
    {
        case BLSR_SET_LASER_LED_PWM:
            status = recv_set_laser_led_pwm(in_buf);
            break;
        case BLSR_SET_LASER_PWM:
        case BLSR_SET_LED_PWM:
        case BLSR_SET_PULSE_STATE:
        default:
            return ERR_UNKNOWN_CMD;
    }
    
    if (status == NO_ERR) 
    {
        //Send ACK cmd
        status = send_comms_cmd(BLSR_POST_ACK);   
    }
    return status;
}

static uint16_t send_msg_psoc(uint8_t cmd_ID, uint8_t *out_buf, uint16_t len)
{
    /* Different fcns invoked based on send_cmd ID - For now,
       only need to support IMU report of data with a timestamp.
       Output unformatted data into out_buf, return len of msg.
    */
    uint8_t status = NO_ERR;
    
    switch (cmd_ID) 
    {
        case BLSR_POST_IMU_DATA:
            status = handle_post_imu_data(out_buf, len);
            break;
        case BLSR_POST_ACK:
            status = NO_ERR;
            break;
        case BLSR_POST_ERR_DETAILED:
        case BLSR_POST_ERR:
        default:
            return ERR_UNKNOWN_CMD;
    }
    /* Optionally, can report errors once detailed send supported */
    
    return status;
}

static uint16_t send_msg_pc(uint8_t cmd_ID, uint8_t *out_buf, uint16_t len)
{
    /* Different fcns invoked based on send_cmd ID - For now,
       only need to support IMU report of data with a timestamp.
       Output unformatted data into out_buf, return len of msg.
    */
    uint8_t status = NO_ERR;
    
    switch (cmd_ID) 
    {
        case BLSR_SET_LASER_LED_PWM:
            status = send_laser_led_pwm(in_buf);
            break;
        case BLSR_SET_LASER_PWM:
        case BLSR_SET_LED_PWM:
        case BLSR_SET_PULSE_STATE:
        default:
            return ERR_UNKNOWN_CMD;
    }
    /* Optionally, can report errors once detailed send supported */
    
    return status;
}


/* [] END OF FILE */
