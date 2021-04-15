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
#ifndef BLASER_MESSAGES_H_
#define BLASER_MESSAGES_H_
    
#include <stdint.h>
/* TODO - Endian-ness checking, to support varying endian architectures. 
   As a temporary measure, generate an error if we use a big-endian system when we 
   don't support big-endian message packing
 */
    
    
/** @brief Detailed debug info enable */
#define __ENABLE_DEBUG_STR
    
/* Message data sizes- NOTE, TEST SIZES TO STRUCTS TO ENSURE THEY MATCH! */
#define BLASER_POST_IMU_DATA_SIZE (28)
// For now, ACKs have no payload. Can expand later to report more info.
#define BLASER_POST_ACK (0) 
#define BLASER_SET_LASER_LED_PWM_SIZE (2)
    
#define GET_BYTE_IDX(data, x) ((data & (0xFF << (8*x))) >> (8*x))
#define SET_BYTE_IDX(data, byte, x) (((data & (~(0xFF << (8*x))) | (byte << (8*x))
    
/** @brief Cool compilation black magic stolen from the linux kernel.
    See https://scaryreasoner.wordpress.com/2009/02/28/checking-sizeof-at-compile-time/ for more info.
  */
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

/** @brief Enum that encodes message IDs to/from Blaser */
enum blaser_msg_id_encoding {
    BLSR_NO_CMD = 0,
    BLSR_POST_IMU_DATA = 1,
    BLSR_POST_ERR = 2,
    BLSR_POST_ERR_DETAILED = 3,
    BLSR_POST_ACK = 4,
    BLSR_SET_LASER_PWM = 5,
    BLSR_SET_LED_PWM = 6,
    BLSR_SET_LASER_LED_PWM = 7,
    BLSR_SET_PULSE_STATE = 8
} blaser_msg_id;

/** Structures that define order of blaser cmds - TODO potentially find a generic way to prevent mistakes when changing variables? 
    - Potential idea to have a lookup table of CMD-ID -> Parsing strings, and use sscanf/snprintf, but seems prone to errors.
    - If structs were in elements of uint8_t, could set variables in a message and 'pack' them over serial, would avoid errors when 
      reorganizing them. Potentially slower due to packing time. Easily done via unions of uint8_t size. 

    - Alternative - use uint32_t for all variables, pad as needed. Use pre-defined num-variable strings to deal with # of variables in a struct
      For now, will stick with manual helper methods to encode/decode messages, but for the future, would benefit from improving this design for future embedded projects.
*/


/* Data structs must be declared with byte-sized elements to avoid padding issues. */
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

union blaser_post_imu_data_msg {
    struct blaser_post_imu_data struct_data;
    uint8_t data[BLASER_POST_IMU_DATA_SIZE];
};

struct blaser_post_err {
    uint8_t message_id_erred;
};

struct blaser_post_err_detailed {
    uint8_t message_id_erred;
    uint8_t err_string_len;
    const char *err_string;
};

//struct blaser_post_ack {
//    uint8_t message_id_acked;
//};

struct blaser_set_laser_pwm {
    uint8_t laser_pwm;
};

struct blaser_set_led_pwm {
    uint8_t led_pwm;
};

struct blaser_set_laser_led_pwm {
    uint8_t laser_pwm;
    uint8_t led_pwm;
};

union blaser_set_laser_led_pwm_msg {
    struct blaser_set_laser_led_pwm struct_data;
    uint8_t data[BLASER_SET_LASER_LED_PWM_SIZE];
};

/** TODO - Could add other flags able to be changed here, like IMU settings */
struct blaser_set_pulse_state {
    uint8_t pulse_mask;
};

/* Public function definitions */
uint16_t get_len_MSG_ID(uint8_t cmd_ID);
int8_t read_msg(uint8_t cmd_ID, uint8_t *in_buf);
int16_t send_msg(uint8_t cmd_ID, uint8_t *out_buf, uint16_t len);
uint8_t get_laser_val(void);

#endif /* BLASER_MESSAGES_H_ */
/* [] END OF FILE */
