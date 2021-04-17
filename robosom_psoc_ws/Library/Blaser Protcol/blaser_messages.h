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
#ifndef BLASER_MESSAGES_H_
#define BLASER_MESSAGES_H_
    
#include <stdint.h>
/* TODO - Verify Endian-ness across different platforms. */
    
    
/** @brief Detailed debug info enable */
#define __ENABLE_DEBUG_STR
    
/* Message data sizes- NOTE, TEST SIZES TO STRUCTS TO ENSURE THEY MATCH! */
#define BLASER_POST_IMU_DATA_SIZE (28)
#define BLASER_POST_ACK_SIZE (3) 
#define BLASER_SET_PULSE_STATE_SIZE (1)
#define BLASER_SET_LASER_LED_PWM_SIZE (2)
#define BLASER_SET_TRIGGER_PULSE_SIZE (0)
    
#define GET_BYTE_IDX(data, x) ((((data & (0xFF << (8*x))) >> (8*x))) & 0xFF)
#define SET_BYTE_IDX(data, byte, x) (((data & (~(0xFF << (8*x))) | (byte << (8*x))
    
/** @brief Cool compilation black magic stolen from the linux kernel.
    See https://scaryreasoner.wordpress.com/2009/02/28/checking-sizeof-at-compile-time/ for more info.
  */
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

/** @brief Enum that encodes message IDs to/from Blaser */
enum blaser_msg_id_encoding {
    BLSR_NO_CMD = 0,
    BLSR_POST_IMU_DATA = 1,
    BLSR_POST_ACK = 2,
    BLSR_SET_LASER_LED_PWM = 3,
    BLSR_SET_PULSE_STATE = 4,
    BLSR_SET_TRIGGER_PULSE = 5
} blaser_msg_id;

/** @brief Describes different lighting states for the LED/Laser */
enum blaser_pulse_state {
    PULSE_STATE_ALL_OFF = 0,
    PULSE_STATE_LED = 1,
    PULSE_STATE_LASER = 2,
    PULSE_STATE_ALTERNATE = 3,
    PULSE_STATE_INVALID_RANGE = 4
} pulse_state;

/* Structs are declared with byte-sizes and should be 1-byte aligned,
   however explicit pack macro is used to ensure this.
 */
#pragma pack(push, 1)

/* Data structs must be declared with byte-sized elements to avoid padding issues. */

/** @brief Posts IMU data and a timestamp in ms. */
struct blaser_post_imu_data {
    uint8_t sys_clock_cur_ms_BYTE_3; /* uint32_t sys_clock_cur_ms */
    uint8_t sys_clock_cur_ms_BYTE_2;
    uint8_t sys_clock_cur_ms_BYTE_1;
    uint8_t sys_clock_cur_ms_BYTE_0;
    uint8_t accel_x_BYTE_3; /* int16_t accel_x */
    uint8_t accel_x_BYTE_2;
    uint8_t accel_x_BYTE_1;
    uint8_t accel_x_BYTE_0;
    uint8_t accel_y_BYTE_3; /* int16_t accel_y */
    uint8_t accel_y_BYTE_2;
    uint8_t accel_y_BYTE_1;
    uint8_t accel_y_BYTE_0;
    uint8_t accel_z_BYTE_3; /* int16_t accel_z */
    uint8_t accel_z_BYTE_2;
    uint8_t accel_z_BYTE_1;
    uint8_t accel_z_BYTE_0;
    uint8_t gyro_x_BYTE_3; /* int16_t gyxo_x */
    uint8_t gyro_x_BYTE_2;
    uint8_t gyro_x_BYTE_1;
    uint8_t gyro_x_BYTE_0;
    uint8_t gyro_y_BYTE_3; /* int16_t gyro_y */
    uint8_t gyro_y_BYTE_2;
    uint8_t gyro_y_BYTE_1;
    uint8_t gyro_y_BYTE_0;
    uint8_t gyro_z_BYTE_3; /* int16_t gyro_z */
    uint8_t gyro_z_BYTE_2;
    uint8_t gyro_z_BYTE_1;
    uint8_t gyro_z_BYTE_0;
};

/** @brief Serialization union for blaser_post_imu_data struct */
union blaser_post_imu_data_msg {
    struct blaser_post_imu_data struct_data;
    uint8_t data[BLASER_POST_IMU_DATA_SIZE];
};

/** @brief UNIMPLEMENTED- Could be used
    for debug strings in the future, using
    a const-sized char array. 

    Alternatively, simplistic error output
    could be used to send a packet with an
    error code. 

struct blaser_post_err {
    uint8_t message_id_erred;
};

struct blaser_post_err_detailed {
    uint8_t message_id_erred;
    uint8_t err_string_len;
    const char *err_string;
};
*/

/** @brief ACK message send from MCU->PC after a cmd.
           Includes the return status of a message.
 */
struct blaser_post_ack {
    uint8_t message_id_acked;
    uint8_t message_status_BYTE_1; /* int8_t message_status */
    uint8_t message_status_BYTE_0;
};

/** @brief Serialization union for blaser_post_ack struct */
union blaser_post_ack_msg {
    struct blaser_post_ack struct_data;
    uint8_t data[BLASER_POST_ACK_SIZE];
};

/** @brief UNIMPLEMENTED 
struct blaser_set_laser_pwm {
    uint8_t laser_pwm;
};
*/

/** @brief UNIMPLEMENTED 
struct blaser_set_led_pwm {
    uint8_t led_pwm;
};
*/

/** @brief Sets the laser PWM and led PWM values */
struct blaser_set_laser_led_pwm {
    uint8_t laser_pwm;
    uint8_t led_pwm;
};

/** @brief Serialization union for blaser_set_laser_led_pwm struct */
union blaser_set_laser_led_pwm_msg {
    struct blaser_set_laser_led_pwm struct_data;
    uint8_t data[BLASER_SET_LASER_LED_PWM_SIZE];
};

/** @brief Message structure for setting laser/led pulse state.
    Can be configured to LED, Laser, both off, or alternating
    on different frames.
    TODO - Could add other settings able to be changed here.
    For now, can be set to a val matching pulse_state enum.
 */
struct blaser_set_pulse_state {
    uint8_t pulse_mask;
};

/** @brief Serialization union for blaser_set_pulse_state struct */
union blaser_set_pulse_state_msg {
    struct blaser_set_pulse_state struct_data;
    uint8_t data[BLASER_SET_PULSE_STATE_SIZE];
};

#pragma pack(pop)

/* Public function definitions */
uint16_t get_len_MSG_ID(uint8_t cmd_ID);
int8_t read_msg(uint8_t cmd_ID, uint8_t *in_buf);
int8_t send_msg(uint8_t cmd_ID, uint8_t *out_buf, uint16_t len);

#endif /* BLASER_MESSAGES_H_ */
/* [] END OF FILE */
