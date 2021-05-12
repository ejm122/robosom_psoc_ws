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

/** @brief Checksum enable define */
#define CHKSUM_ACTIVE (1)

/* Message data sizes- NOTE, TEST SIZES TO STRUCTS TO ENSURE THEY MATCH! */
#define BLASER_POST_IMU_DATA_SIZE (32)
#define BLASER_POST_ACK_SIZE (3) 
#define BLASER_SET_PULSE_STATE_SIZE (1)
#define BLASER_SET_LASER_LED_PWM_SIZE (2)
#define BLASER_SET_TRIGGER_PULSE_SIZE (0)
#define BLASER_POST_EXPOSURE_TIME_SIZE (8)
#define BLASER_SET_MCU_TIME_SIZE (8)
#define BLASER_SET_IMU_FOC_CONF_SIZE (6)
#define BLASER_IMU_OFFSETS_SIZE (18)
#define BLASER_SET_EXECUTE_CMD_SIZE (1)

#define GET_BYTE_IDX(data, x) ((((data & (0xFFL << (8*x))) >> (8*x))) & 0xFFL)
#define SET_BYTE_IDX(data, byte, x) ((data & (~(0xFFL << (8*(x))))) | (byte << (8*(x))))
    
/** @brief Cool compilation black magic stolen from the linux kernel.
    See https://scaryreasoner.wordpress.com/2009/02/28/checking-sizeof-at-compile-time/ for more info.
  */
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

/** @brief Enum that encodes message IDs to/from Blaser */
/* Set Cmds indiciate PC -> MCU, post cmds indiciate MCU -> PC */
enum blaser_msg_id_encoding {
    BLSR_NO_CMD = 0,
    BLSR_POST_IMU_DATA = 1,
    BLSR_POST_ACK = 2,
    BLSR_SET_LASER_LED_PWM = 3,
    BLSR_SET_PULSE_STATE = 4,
    BLSR_SET_TRIGGER_PULSE = 5,
    BLSR_POST_EXPOSURE_TIME = 6,
    BLSR_SET_MCU_TIME = 7,
    BLSR_SET_IMU_FOC_CONF = 8,
    BLSR_POST_IMU_OFFSETS = 9,
    BLSR_SET_IMU_OFFSETS = 10,
    BLSR_SET_EXECUTE_CMD = 11
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

/** @brief Posts IMU data and a timestamp in secs, usecs. */
struct blaser_post_imu_data {
    uint8_t secs_BYTE_3; /* uint32_t secs */
    uint8_t secs_BYTE_2;
    uint8_t secs_BYTE_1;
    uint8_t secs_BYTE_0;
    uint8_t usecs_BYTE_3; /* uint32_t usecs */
    uint8_t usecs_BYTE_2;
    uint8_t usecs_BYTE_1;
    uint8_t usecs_BYTE_0;
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

/** @brief Posts timestamp of exposure_frame signal in secs, usecs. */
struct blaser_post_exposure_time {
    uint8_t secs_BYTE_3; /* uint32_t secs */
    uint8_t secs_BYTE_2;
    uint8_t secs_BYTE_1;
    uint8_t secs_BYTE_0;
    uint8_t usecs_BYTE_3; /* uint32_t usecs */
    uint8_t usecs_BYTE_2;
    uint8_t usecs_BYTE_1;
    uint8_t usecs_BYTE_0;
};

/** @brief Serialization union for blaser_post_exposure_time struct */
union blaser_post_exposure_time_msg {
    struct blaser_post_exposure_time struct_data;
    uint8_t data[BLASER_POST_EXPOSURE_TIME_SIZE];
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

/** @brief Sets the MCU's IMU's fast offset compenstation
           values. See the BMI160 datasheet for more info,
           section 2.9.1.
 */
struct blaser_set_imu_foc_conf {
    /*! Enabling FOC in gyro
     * Assignable macros :
     *  - BMI160_ENABLE  0x1
     *  - BMI160_DISABLE 0x0
     */
    uint8_t foc_gyr_en;

    /*! Accel FOC configurations
     * Assignable macros :
     *  - BMI160_FOC_ACCEL_DISABLED    0x0
     *  - BMI160_FOC_ACCEL_POSITIVE_G  0x1
     *  - BMI160_FOC_ACCEL_NEGATIVE_G  0x2
     *  - BMI160_FOC_ACCEL_0G          0x3
     */
    uint8_t foc_acc_x;
    uint8_t foc_acc_y;
    uint8_t foc_acc_z;

    /*! Enabling offset compensation for accel in data registers
     * Assignable macros :
     *  - BMI160_ENABLE  0x1
     *  - BMI160_DISABLE 0x0
     */
    uint8_t acc_off_en;

    /*! Enabling offset compensation for gyro in data registers
     * Assignable macros :
     *  - BMI160_ENABLE  0x1
     *  - BMI160_DISABLE 0x0
     */
    uint8_t gyro_off_en;
};

/** @brief Serialization union for blaser_post_imu_data struct */
union blaser_set_imu_foc_conf_msg {
    struct blaser_set_imu_foc_conf struct_data;
    uint8_t data[BLASER_SET_IMU_FOC_CONF_SIZE];
};

/** @brief Sets/Gets the MCU's IMU's fast offset compenstation
           offsets. See the BMI160 datasheet for more info,
           section 2.9.1.
           Shared for the CMDs BLSR_SET_IMU_OFFSETS and
           BLSR_POST_IMU_OFFSETS
 */
struct blaser_imu_offsets {
    /*! Accel offset for x axis */
    uint8_t off_acc_x_BYTE_1; /* int8_t off_acc_x */
    uint8_t off_acc_x_BYTE_0;

    /*! Accel offset for y axis */
    uint8_t off_acc_y_BYTE_1; /* int8_t off_acc_y */
    uint8_t off_acc_y_BYTE_0;

    /*! Accel offset for z axis */
    uint8_t off_acc_z_BYTE_1; /* int8_t off_acc_z */
    uint8_t off_acc_z_BYTE_0;

    /*! Gyro offset for x axis */
    uint8_t off_gyro_x_BYTE_3; /* int16_t off_gyro_x */
    uint8_t off_gyro_x_BYTE_2;
    uint8_t off_gyro_x_BYTE_1;
    uint8_t off_gyro_x_BYTE_0;
    
    /*! Gyro offset for y axis */
    uint8_t off_gyro_y_BYTE_3; /* int16_t off_gyro_y */
    uint8_t off_gyro_y_BYTE_2;
    uint8_t off_gyro_y_BYTE_1;
    uint8_t off_gyro_y_BYTE_0;

    /*! Gyro offset for z axis */
    uint8_t off_gyro_z_BYTE_3; /* int16_t off_gyro_z */
    uint8_t off_gyro_z_BYTE_2;
    uint8_t off_gyro_z_BYTE_1;
    uint8_t off_gyro_z_BYTE_0;
};

/** @brief Serialization union for blaser_post_imu_data struct */
union blaser_imu_offsets_msg {
    struct blaser_imu_offsets struct_data;
    uint8_t data[BLASER_IMU_OFFSETS_SIZE];
};

/** @brief Sets MCU time to input of secs, usecs. */
struct blaser_set_mcu_time {
    uint8_t secs_BYTE_3; /* uint32_t secs */
    uint8_t secs_BYTE_2;
    uint8_t secs_BYTE_1;
    uint8_t secs_BYTE_0;
    uint8_t usecs_BYTE_3; /* uint32_t usecs */
    uint8_t usecs_BYTE_2;
    uint8_t usecs_BYTE_1;
    uint8_t usecs_BYTE_0;
};

/** @brief Serialization union for blaser_set_mcu_time struct */
union blaser_set_mcu_time_msg {
    struct blaser_set_mcu_time struct_data;
    uint8_t data[BLASER_SET_MCU_TIME_SIZE];
};


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

/** @brief Triggers the MCU to execute a cmd with input cmdID */
struct blaser_set_execute_cmd {
    uint8_t cmd_id;
};

/** @brief Serialization union for blaser_set_execute_cmd struct */
union blaser_set_execute_cmd_msg {
    struct blaser_set_execute_cmd struct_data;
    uint8_t data[BLASER_SET_EXECUTE_CMD_SIZE];
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
