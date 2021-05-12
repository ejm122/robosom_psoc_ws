/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * blaser_messages_psoc.c
 *
 * Created 3/26/2021 by ebcohen@andrew.cmu.edu
 *
 * This file provides blaser-interfacing commands
 * for MCU-side communication.
 *
 * While functionality of blaser_messages_psoc/pc.c can differ
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

#include "blaser_messages_psoc.h"
// Include BMI160 Library and PSOC HAL
#include "../Library/BMI160/bmi160.h"
#include "../Library/BMI160/bmi160_psoc.h"
#include "blaser_protocol.h"
#include "error_codes.h"

/** @brief Reference global system clock */
extern uint32 t_us;
extern uint32 t_s;
extern uint32 t_exposure_us;
extern uint32 t_exposure_s;

/** @brief Reference main IMU configuration values */
extern struct bmi160_dev sensor;

/* Static definitions for mcu-specific functions. */
static int8_t read_msg_psoc(uint8_t cmd_ID, uint8_t *in_buf);
static int8_t send_msg_psoc(uint8_t cmd_ID, uint8_t *out_buf, uint16_t len);

/** Global vars that can be accessed via get cmds */
/** @brief LED PWM output intensity, from 0 -> 127 */
uint8_t led_val;
/** @brief Laser PWM output intensity, from 0 -> 127 */
uint8_t laser_val = 1;
/** @brief Var that stores pulse state */
uint8_t pulse_state_val = PULSE_STATE_ALTERNATE;
/** @brief Var that stores last read cmd for sending an ack */
uint8_t ack_cmd = 0;
/** @brief Var that stores last read cmd's status for sending ack */
int8_t  ack_cmd_status = NO_ERR;

/** @brief Stores IMU's FOC_CONF between invocations */
struct bmi160_foc_conf foc_conf;

/** @brief Maps from MSG_ID to the packet's length
    @param cmd_ID message header ID to get length of 
    @return Length of payload for given message ID 
 */
uint16_t get_len_MSG_ID(uint8_t cmd_ID) {
    switch (cmd_ID) 
    {
        case BLSR_POST_IMU_DATA:
            return BLASER_POST_IMU_DATA_SIZE;
        case BLSR_SET_LASER_LED_PWM:
            return BLASER_SET_LASER_LED_PWM_SIZE;
        case BLSR_POST_ACK:
            return BLASER_POST_ACK_SIZE;
        case BLSR_SET_TRIGGER_PULSE:
            return BLASER_SET_TRIGGER_PULSE_SIZE;
        case BLSR_SET_PULSE_STATE:
            return BLASER_SET_PULSE_STATE_SIZE;
        case BLSR_POST_EXPOSURE_TIME:
            return BLASER_POST_EXPOSURE_TIME_SIZE;
        case BLSR_SET_MCU_TIME:
            return BLASER_SET_MCU_TIME_SIZE;
        case BLSR_SET_IMU_FOC_CONF:
            return BLASER_SET_IMU_FOC_CONF_SIZE;
        case BLSR_SET_IMU_OFFSETS:
        case BLSR_POST_IMU_OFFSETS:
            return BLASER_IMU_OFFSETS_SIZE;
        case BLSR_SET_EXECUTE_CMD:
            return BLASER_SET_EXECUTE_CMD_SIZE;
        default:
            return 0;
    }
    return 0;
}

/** @brief Wrapper for PSoC-Side. Modify this file to support read_msg_pc
           for communication from PC -> PSoC
    @param uint8_t cmd_ID message header ID read
    @param uint8_t *in_buf Payload input buffer for reading
    @return NO_ERR on success, otherwise negative error 
 */
int8_t read_msg(uint8_t cmd_ID, uint8_t *in_buf) 
{
    return read_msg_psoc(cmd_ID, in_buf);   
}

/** @brief Wrapper for PSoC-Side. Modify this file to support send_msg_pc
           for communication from PC -> PSoC 
    @param uint8_t cmd_ID message header ID to send
    @param uint8_t *out_buf Payload output buffer for sending
    @param uint16_t len Length of payload output buffer
    @return NO_ERR on success, otherwise negative error 
 */
int8_t send_msg(uint8_t cmd_ID, uint8_t *out_buf, uint16_t len) 
{
    return send_msg_psoc(cmd_ID, out_buf, len);//send_msg_psoc(cmd_ID, out_buf, len);
}

/** @brief MCU-side cmd that reads data from onboard
           IMU, and packages it onto the send payload along
           with a current timestamp.
    @param uint8_t *out_buf Payload output buffer for sending
    @param uint16_t len Length of payload output buffer
    @return NO_ERR on success, otherwise negative error or BMI160's error
 */
int8_t send_post_imu_data(uint8_t *out_buf, uint16_t len) 
{
    int8_t rslt;
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;
    union blaser_post_imu_data_msg msg;
    
    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_post_imu_data) != BLASER_POST_IMU_DATA_SIZE));
    
    rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &sensor);
    uint32_t cur_time_us = t_us;
    uint32_t cur_time_s = t_s;
    
    if (rslt != BMI160_OK) 
    {
        return rslt;
    }
    
    if (len < BLASER_POST_IMU_DATA_SIZE) 
    {
        return ERR_BUFF_OVERFLOW;   
    }
    
    /* Initialize the message struct- Order does not matter with identifier usage */
    msg.struct_data = (struct blaser_post_imu_data) 
    {
        .secs_BYTE_3 = GET_BYTE_IDX(cur_time_s, 3),
        .secs_BYTE_2 = GET_BYTE_IDX(cur_time_s, 2),
        .secs_BYTE_1 = GET_BYTE_IDX(cur_time_s, 1),
        .secs_BYTE_0 = GET_BYTE_IDX(cur_time_s, 0),
        
        .usecs_BYTE_3 = GET_BYTE_IDX(cur_time_us, 3),
        .usecs_BYTE_2 = GET_BYTE_IDX(cur_time_us, 2),
        .usecs_BYTE_1 = GET_BYTE_IDX(cur_time_us, 1),
        .usecs_BYTE_0 = GET_BYTE_IDX(cur_time_us, 0),
        
        .accel_x_BYTE_3 = GET_BYTE_IDX(accel.x, 3),
        .accel_x_BYTE_2 = GET_BYTE_IDX(accel.x, 2),
        .accel_x_BYTE_1 = GET_BYTE_IDX(accel.x, 1),
        .accel_x_BYTE_0 = GET_BYTE_IDX(accel.x, 0),
        
        .accel_y_BYTE_3 = GET_BYTE_IDX(accel.y, 3),
        .accel_y_BYTE_2 = GET_BYTE_IDX(accel.y, 2),
        .accel_y_BYTE_1 = GET_BYTE_IDX(accel.y, 1),
        .accel_y_BYTE_0 = GET_BYTE_IDX(accel.y, 0),
        
        .accel_z_BYTE_3 = GET_BYTE_IDX(accel.z, 3),
        .accel_z_BYTE_2 = GET_BYTE_IDX(accel.z, 2),
        .accel_z_BYTE_1 = GET_BYTE_IDX(accel.z, 1),
        .accel_z_BYTE_0 = GET_BYTE_IDX(accel.z, 0),
        
        .gyro_x_BYTE_3 = GET_BYTE_IDX(gyro.x, 3),
        .gyro_x_BYTE_2 = GET_BYTE_IDX(gyro.x, 2),
        .gyro_x_BYTE_1 = GET_BYTE_IDX(gyro.x, 1),
        .gyro_x_BYTE_0 = GET_BYTE_IDX(gyro.x, 0),
        
        .gyro_y_BYTE_3 = GET_BYTE_IDX(gyro.y, 3),
        .gyro_y_BYTE_2 = GET_BYTE_IDX(gyro.y, 2),
        .gyro_y_BYTE_1 = GET_BYTE_IDX(gyro.y, 1),
        .gyro_y_BYTE_0 = GET_BYTE_IDX(gyro.y, 0),
        
        .gyro_z_BYTE_3 = GET_BYTE_IDX(gyro.z, 3),
        .gyro_z_BYTE_2 = GET_BYTE_IDX(gyro.z, 2),
        .gyro_z_BYTE_1 = GET_BYTE_IDX(gyro.z, 1),
        .gyro_z_BYTE_0 = GET_BYTE_IDX(gyro.z, 0),
    };
    
    /* Write msg data to output buffer */
    memcpy(out_buf, msg.data, BLASER_POST_IMU_DATA_SIZE);
    return NO_ERR;
}

/** @brief MCU-side cmd that reads data from onboard
           IMU, and packages it onto the send payload along
           with a current timestamp.
    @param uint8_t *out_buf Payload output buffer for sending
    @param uint16_t len Length of payload output buffer
    @return NO_ERR on success, otherwise negative error or BMI160's error
 */
int8_t send_post_imu_offsets(uint8_t *out_buf, uint16_t len) 
{
    union blaser_imu_offsets_msg msg;
    int retval = BMI160_OK;
    struct bmi160_offsets offsets;

    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_imu_offsets) != BLASER_IMU_OFFSETS_SIZE));
    
    retval = bmi160_get_offsets(&offsets, &sensor);
    
    if (retval != BMI160_OK) 
    {
        return retval;
    }
    
    if (len < BLASER_IMU_OFFSETS_SIZE) 
    {
        return ERR_BUFF_OVERFLOW;   
    }
    
    /* Initialize the message struct- Order does not matter with identifier usage */
    msg.struct_data = (struct blaser_imu_offsets) 
    {
        .off_acc_x_BYTE_1 = GET_BYTE_IDX(offsets.off_acc_x, 1),
        .off_acc_x_BYTE_0 = GET_BYTE_IDX(offsets.off_acc_x, 0),
        
        .off_acc_y_BYTE_1 = GET_BYTE_IDX(offsets.off_acc_y, 1),
        .off_acc_y_BYTE_0 = GET_BYTE_IDX(offsets.off_acc_y, 0),
        
        .off_acc_z_BYTE_1 = GET_BYTE_IDX(offsets.off_acc_z, 1),
        .off_acc_z_BYTE_0 = GET_BYTE_IDX(offsets.off_acc_z, 0),
        
        .off_gyro_x_BYTE_3 = GET_BYTE_IDX(offsets.off_gyro_x, 3),
        .off_gyro_x_BYTE_2 = GET_BYTE_IDX(offsets.off_gyro_x, 2),
        .off_gyro_x_BYTE_1 = GET_BYTE_IDX(offsets.off_gyro_x, 1),
        .off_gyro_x_BYTE_0 = GET_BYTE_IDX(offsets.off_gyro_x, 0),
        
        .off_gyro_y_BYTE_3 = GET_BYTE_IDX(offsets.off_gyro_y, 3),
        .off_gyro_y_BYTE_2 = GET_BYTE_IDX(offsets.off_gyro_y, 2),
        .off_gyro_y_BYTE_1 = GET_BYTE_IDX(offsets.off_gyro_y, 1),
        .off_gyro_y_BYTE_0 = GET_BYTE_IDX(offsets.off_gyro_y, 0),
        
        .off_gyro_z_BYTE_3 = GET_BYTE_IDX(offsets.off_gyro_z, 3),
        .off_gyro_z_BYTE_2 = GET_BYTE_IDX(offsets.off_gyro_z, 2),
        .off_gyro_z_BYTE_1 = GET_BYTE_IDX(offsets.off_gyro_z, 1),
        .off_gyro_z_BYTE_0 = GET_BYTE_IDX(offsets.off_gyro_z, 0),
    };
    
    /* Write msg data to output buffer */
    memcpy(out_buf, msg.data, BLASER_IMU_OFFSETS_SIZE);
    return NO_ERR;
}

/** @brief MCU-side cmd that posts the IMU's FOC offsets
    @param uint8_t *out_buf Payload output buffer for sending
    @param uint16_t len Length of payload output buffer
    @return NO_ERR on success, otherwise negative error 
 */
int8_t send_post_exposure_time(uint8_t *out_buf, uint16_t len) 
{
    union blaser_post_exposure_time_msg msg;
    
    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_post_exposure_time) != BLASER_POST_EXPOSURE_TIME_SIZE));
    
    /** Potentially would want to disable interrupts here */
    uint32_t cur_time_us = t_exposure_us;
    uint32_t cur_time_s = t_exposure_s;
    
    if (len < BLASER_POST_EXPOSURE_TIME_SIZE) 
    {
        return ERR_BUFF_OVERFLOW;   
    }
    
    /* Initialize the message struct- Order does not matter with identifier usage */
    msg.struct_data = (struct blaser_post_exposure_time) 
    {
        .secs_BYTE_3 = GET_BYTE_IDX(cur_time_s, 3),
        .secs_BYTE_2 = GET_BYTE_IDX(cur_time_s, 2),
        .secs_BYTE_1 = GET_BYTE_IDX(cur_time_s, 1),
        .secs_BYTE_0 = GET_BYTE_IDX(cur_time_s, 0),

        .usecs_BYTE_3 = GET_BYTE_IDX(cur_time_us, 3),
        .usecs_BYTE_2 = GET_BYTE_IDX(cur_time_us, 2),
        .usecs_BYTE_1 = GET_BYTE_IDX(cur_time_us, 1),
        .usecs_BYTE_0 = GET_BYTE_IDX(cur_time_us, 0),
    };
    
    /* Write msg data to output buffer */
    memcpy(out_buf, msg.data, BLASER_POST_EXPOSURE_TIME_SIZE);
    return NO_ERR;
}

/** @brief MCU-side function for syncing timestamps.
           Sets usec and sec counters.
    @param uint8_t *in_buf Payload input buffer for reading
    @return NO_ERR on success, otherwise negative error 
  */
int8_t recv_set_mcu_time(uint8_t *in_buf)
{
    union blaser_set_mcu_time_msg msg;
    uint32_t recv_us;
    uint32_t recv_s;

    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_set_mcu_time) != BLASER_SET_MCU_TIME_SIZE));
    
    memcpy(msg.data, in_buf, BLASER_SET_MCU_TIME_SIZE);

    // Reconstruct payload data
    recv_s = SET_BYTE_IDX(0L, 3, msg.struct_data.secs_BYTE_3);
    recv_s = SET_BYTE_IDX(recv_s, 2, msg.struct_data.secs_BYTE_2);
    recv_s = SET_BYTE_IDX(recv_s, 1, msg.struct_data.secs_BYTE_1);
    recv_s = SET_BYTE_IDX(recv_s, 0, msg.struct_data.secs_BYTE_0);

    recv_us = SET_BYTE_IDX(0L, 3, msg.struct_data.usecs_BYTE_3);
    recv_us = SET_BYTE_IDX(recv_us, 2, msg.struct_data.usecs_BYTE_2);
    recv_us = SET_BYTE_IDX(recv_us, 1, msg.struct_data.usecs_BYTE_1);
    recv_us = SET_BYTE_IDX(recv_us, 0, msg.struct_data.usecs_BYTE_0);

    __disable_irq();
    t_us = recv_us;
    t_s = recv_s;
    __enable_irq();
    
    return NO_ERR;
}

/** @brief MCU-side function for setting PWM values
           of both LED and Laser PWM drivers.
    @param uint8_t *in_buf Payload input buffer for reading
    @return NO_ERR on success, otherwise negative error 
  */
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

/** @brief MCU-side function for setting pulse state
    @param uint8_t *in_buf Payload input buffer for reading
    @return NO_ERR on success, otherwise negative error 
  */
int8_t recv_set_pulse_state(uint8_t *in_buf)
{
    union blaser_set_pulse_state_msg msg;
    
    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_set_pulse_state) != BLASER_SET_PULSE_STATE_SIZE));
    
    memcpy(msg.data, in_buf, BLASER_SET_PULSE_STATE_SIZE);
    
    if (msg.struct_data.pulse_mask >= PULSE_STATE_INVALID_RANGE)
    {
        return ERR_UNKNOWN_CMD;
    }
    
    pulse_state_val = msg.struct_data.pulse_mask;
    
    return NO_ERR;
}

/** @brief MCU-side function for triggering a post CMD
    @param uint8_t *in_buf Payload input buffer for reading
    @return NO_ERR on success, otherwise negative error 
  */
int8_t recv_set_execute_cmd(uint8_t *in_buf)
{
    union blaser_set_execute_cmd_msg msg;
    
    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_set_execute_cmd) != BLASER_SET_EXECUTE_CMD_SIZE));
    
    memcpy(msg.data, in_buf, BLASER_SET_EXECUTE_CMD_SIZE);
    uint8_t cmd_id = msg.struct_data.cmd_id;
    
    return send_comms_cmd(cmd_id);
}

/** @brief MCU-side function for setting MCU's IMU's FOC configuration
    @param uint8_t *in_buf Payload input buffer for reading
    @return NO_ERR on success, otherwise BMI160's error
  */
int8_t recv_set_imu_foc_conf(uint8_t *in_buf)
{
    union blaser_set_imu_foc_conf_msg msg;
    int retval = BMI160_OK;
    struct bmi160_offsets offsets;

    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_set_imu_foc_conf) != BLASER_SET_IMU_FOC_CONF_SIZE));
    
    memcpy(msg.data, in_buf, BLASER_SET_IMU_FOC_CONF_SIZE);
    
    foc_conf.acc_off_en = msg.struct_data.acc_off_en;
    foc_conf.foc_acc_x = msg.struct_data.foc_acc_x;
    foc_conf.foc_acc_y = msg.struct_data.foc_acc_y;
    foc_conf.foc_acc_z = msg.struct_data.foc_acc_z;
    foc_conf.foc_gyr_en = msg.struct_data.foc_gyr_en;
    foc_conf.gyro_off_en = msg.struct_data.gyro_off_en;
    
    retval = bmi160_start_foc(&foc_conf, &offsets, &sensor);
    
    if (retval != BMI160_OK)
    {
        return retval;
    }
    
    return NO_ERR;
}

/** @brief MCU-side function for setting MCU's IMU's FOC offsets
    @param uint8_t *in_buf Payload input buffer for reading
    @return NO_ERR on success, otherwise BMI160's error
  */
int8_t recv_set_imu_offsets(uint8_t *in_buf)
{
    union blaser_imu_offsets_msg msg;
    int retval = BMI160_OK;
    struct bmi160_offsets offsets;

    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_imu_offsets) != BLASER_IMU_OFFSETS_SIZE));
    
    memcpy(msg.data, in_buf, BLASER_IMU_OFFSETS_SIZE);
    
    offsets.off_acc_x = (int8_t) (SET_BYTE_IDX(0U, 1, msg.struct_data.off_acc_x_BYTE_1));
    offsets.off_acc_x = (int8_t) (SET_BYTE_IDX(offsets.off_acc_x, 0, msg.struct_data.off_acc_x_BYTE_0));
    
    offsets.off_acc_y = (int8_t) (SET_BYTE_IDX(0U, 1, msg.struct_data.off_acc_y_BYTE_1));
    offsets.off_acc_y = (int8_t) (SET_BYTE_IDX(offsets.off_acc_y, 0, msg.struct_data.off_acc_y_BYTE_0));
    
    offsets.off_acc_z = (int8_t) (SET_BYTE_IDX(0U, 1, msg.struct_data.off_acc_z_BYTE_1));
    offsets.off_acc_z = (int8_t) (SET_BYTE_IDX(offsets.off_acc_z, 0, msg.struct_data.off_acc_z_BYTE_0));
    
    offsets.off_gyro_x = (int16_t) (SET_BYTE_IDX(0L, 3, msg.struct_data.off_gyro_x_BYTE_3));
    offsets.off_gyro_x = (int16_t) (SET_BYTE_IDX(offsets.off_gyro_x, 2, msg.struct_data.off_gyro_x_BYTE_2));
    offsets.off_gyro_x = (int16_t) (SET_BYTE_IDX(offsets.off_gyro_x, 1, msg.struct_data.off_gyro_x_BYTE_1));
    offsets.off_gyro_x = (int16_t) (SET_BYTE_IDX(offsets.off_gyro_x, 0, msg.struct_data.off_gyro_x_BYTE_0));
    
    offsets.off_gyro_y = (int16_t) (SET_BYTE_IDX(0L, 3, msg.struct_data.off_gyro_y_BYTE_3));
    offsets.off_gyro_y = (int16_t) (SET_BYTE_IDX(offsets.off_gyro_y, 2, msg.struct_data.off_gyro_y_BYTE_2));
    offsets.off_gyro_y = (int16_t) (SET_BYTE_IDX(offsets.off_gyro_y, 1, msg.struct_data.off_gyro_y_BYTE_1));
    offsets.off_gyro_y = (int16_t) (SET_BYTE_IDX(offsets.off_gyro_y, 0, msg.struct_data.off_gyro_y_BYTE_0));
    
    offsets.off_gyro_z = (int16_t) (SET_BYTE_IDX(0L, 3, msg.struct_data.off_gyro_z_BYTE_3));
    offsets.off_gyro_z = (int16_t) (SET_BYTE_IDX(offsets.off_gyro_z, 2, msg.struct_data.off_gyro_z_BYTE_2));
    offsets.off_gyro_z = (int16_t) (SET_BYTE_IDX(offsets.off_gyro_z, 1, msg.struct_data.off_gyro_z_BYTE_1));
    offsets.off_gyro_z = (int16_t) (SET_BYTE_IDX(offsets.off_gyro_z, 0, msg.struct_data.off_gyro_z_BYTE_0));

    retval = bmi160_start_foc(&foc_conf, &offsets, &sensor);
    
    if (retval != BMI160_OK)
    {
        return retval;
    }
    
    return NO_ERR;
}

/** @brief MCU-side function that generates a trigger_active pulse.
           Payload-less cmd, so takes in no input. 
 */
int8_t recv_set_trigger_pulse(void)
{
    Trigger_Reg_Write(1);
    return NO_ERR;
}

/** @brief MCU-side function for sending command 
           acknowledged packet with error code of result.
    @param uint8_t *out_buf Payload output buffer for sending
    @param uint16_t len Length of payload output buffer
    @return NO_ERR on success, otherwise negative error 
 */
int8_t send_post_ack(uint8_t *out_buf, uint16_t len)
{
    union blaser_post_ack_msg msg;
    
    /* Generate a compile-time error if there's a type mismatch between data-array and the struct entries */
    BUILD_BUG_ON((sizeof(struct blaser_post_ack) != BLASER_POST_ACK_SIZE));

    if (len < BLASER_POST_ACK_SIZE) 
    {
        return ERR_BUFF_OVERFLOW;   
    }

    msg.struct_data = (struct blaser_post_ack)
    {
        .message_id_acked = ack_cmd,
        .message_status_BYTE_1 = GET_BYTE_IDX(ack_cmd_status, 1),
        .message_status_BYTE_0 = GET_BYTE_IDX(ack_cmd_status, 0),
    };
    
    memcpy(out_buf, msg.data, BLASER_POST_ACK_SIZE);
    
    return NO_ERR;
}

/** @brief Helper function to retrieve Laser PWM val 
    @return PWM val of Laser
 */
uint8_t get_laser_val(void)
{
    return laser_val;
}

/** @brief Helper function to retrieve LED PWM val 
    @return PWM val of LED
 */
uint8_t get_led_val(void)
{
    return led_val;   
}

/** @brief Helper function to retrieve pulse state val
    @return pulse state enum
 */
uint8_t get_pulse_state(void)
{
    return pulse_state_val;   
}


/** @brief MCU packet handler- Replies to read packets.
    @param uint8_t cmd_ID message header ID read
    @param uint8_t *in_buf Payload input buffer for reading
    @return NO_ERR on success, otherwise negative error 
 */
static int8_t read_msg_psoc(uint8_t cmd_ID, uint8_t *in_buf) {
    uint8_t status = NO_ERR;

    // Length check already occured before invoking read_msg.
    switch (cmd_ID) 
    {
        case BLSR_SET_LASER_LED_PWM:
            status = recv_set_laser_led_pwm(in_buf);
            break;
        case BLSR_SET_TRIGGER_PULSE:
            /* Payload-less cmd */
            status = recv_set_trigger_pulse();
            break;
        case BLSR_SET_PULSE_STATE:
            status = recv_set_pulse_state(in_buf);
            break;
        case BLSR_SET_MCU_TIME:
            status = recv_set_mcu_time(in_buf);
            break;
        case BLSR_SET_IMU_FOC_CONF:
            status = recv_set_imu_foc_conf(in_buf);
            break;
        case BLSR_SET_IMU_OFFSETS:
            status = recv_set_imu_offsets(in_buf);
            break;
        case BLSR_SET_EXECUTE_CMD:
            status = recv_set_execute_cmd(in_buf);
            break;
        default:
            return ERR_UNKNOWN_CMD;
    }

    /* On MCU side, ACK cmds are sent after reading a msg */
    ack_cmd = cmd_ID;
    ack_cmd_status = status;
    status = send_comms_cmd(BLSR_POST_ACK);   

    return status;
}

/** @brief MCU packet sender, transmits messages to PC
    @param uint8_t cmd_ID message header ID to send
    @param uint8_t *out_buf Payload output buffer for sending
    @param uint16_t len Length of payload output buffer
    @return NO_ERR on success, otherwise negative error 
 */
static int8_t send_msg_psoc(uint8_t cmd_ID, uint8_t *out_buf, uint16_t len)
{
    /* Different fcns invoked based on send_cmd ID - For now,
       only need to support IMU report of data with a timestamp.
       Output unformatted data into out_buf, return len of msg.
    */
    uint8_t status = NO_ERR;
    
    switch (cmd_ID) 
    {
        case BLSR_POST_IMU_DATA:
            status = send_post_imu_data(out_buf, len);
            break;
        case BLSR_POST_EXPOSURE_TIME:
            status = send_post_exposure_time(out_buf, len);
            break;
        case BLSR_POST_IMU_OFFSETS:
            status = send_post_imu_offsets(out_buf, len);
        case BLSR_POST_ACK:
            status = send_post_ack(out_buf, len);
            break;
        default:
            return ERR_UNKNOWN_CMD;
    }
    /* Optionally, can report errors once detailed send supported */
    
    return status;
}


/* [] END OF FILE */
