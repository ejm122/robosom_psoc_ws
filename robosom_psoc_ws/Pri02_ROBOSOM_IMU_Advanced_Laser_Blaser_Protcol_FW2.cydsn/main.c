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
#include "project.h"
#include "stdio.h"
#include "usb_comm.h"
#include "blaser_protocol.h"
#include "blaser_messages_psoc.h"

// Include BMI160 Library and PSOC HAL
#include "../Library/BMI160/bmi160.h"
#include "../Library/BMI160/bmi160_psoc.h"

struct bmi160_dev sensor;
struct bmi160_sensor_data accel;
struct bmi160_sensor_data gyro;
uint16_t step_count = 0;//stores the step counter value


int8_t imu_bmi160_init(void);
int8_t imu_bmi160_config(void);
int8_t imu_bmi160_enable_step_counter(void);
int8_t imu_bmi160_read_acc_gyo(void);
int8_t imu_bmi160_read_steps(void);

// USBUART
#define USBFS_DEVICE    (0u)
#define USBUART_BUFFER_SIZE (256u)
uint16 count;
uint8 buffer[USBUART_BUFFER_SIZE];
void USBUART_user_check_init(void);
void USBUART_user_echo(void);
uint16 USBUART_user_check_read(void);

uint16 read_count;
uint8 buffer_read[USBUART_BUFFER_SIZE];
uint8 serial_input = 0;

/* Laser/Shutter specific vars */
#define PWM_LASER_OFF (0)

enum shutter_laser_state {
    LSR_DISABLE = 0,
    LSR_ENABLE = 1
} laser_state;

enum shutter_active_state {
    NO_FRAME = 0,
    NEW_FRAME = 1
} shutter_state;

uint8_t frame_status = NO_FRAME;
uint8_t light_status = LSR_DISABLE;

// Testing Function
void print_imu_via_usbuart(void);

// System clock
uint32 sys_clock_cur_ms = 0;
float sys_clock_cur_us_in_ms = 0;
void sys_clock_ms_callback(void); // 1ms callback interrupt function

// Interrupt handlers for the ximea camera
void Isr_shutter_handler(void); // Shutter Active interrupt handler
void Isr_trigger_handler(void); // Trigger Timing interrupt handler

int main(void)
{
    uint8_t led_test = 0;
    buffer[0] = 1;
    
    /* Sets up the GPIO interrupt and enables it */
    isr_EXPOSURE_ACT_StartEx(Isr_shutter_handler);
    
    // USBUART Init
    init_usb_comm();
    USBUART_CDC_Init();
    
    // USBUART Protocol Init
    blaser_comms_register(&usb_get_char, &usb_num_available, &usb_print_block);
        
    // I2C Init
    I2C_1_Start();
    
    // IMU BMI160 Init
    imu_bmi160_init();
    imu_bmi160_config();
    imu_bmi160_enable_step_counter();
    
    // PWM Block Init
    PWM_LED_Start();
    PWM_LASER_Start();
   
    /* Turn off LEDs */
    Led_Red_Write(0);
    Led_Green_Write(0);
    Led_Blue_Write(0);
    Led_Key_Write(1);
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    CyDelay(10000);
    /* For initial testing, establish USB communication before attempting to send first trigger frame */
    while (0u == USBUART_CDCIsReady())
    {
    }
    // Start system 1ms tick
    CySysTickStart();
    CySysTickSetCallback(0, sys_clock_ms_callback);
    CySysTickEnableInterrupt();
    
    
    // Trigger first Ximea trigger pulse - Might want to link this to a button for manual triggering.
    Trig_Pulser_Start();
    Trigger_Reg_Write(1);
    

    for(;;)
    {
        bool reconfigured = false;
        reconfigured = usb_configuration_reinit();
        
        while (frame_status == NEW_FRAME) 
        {
            frame_status = NO_FRAME;
            send_comms_cmd(BLSR_POST_IMU_DATA);
        }

        if (USBUART_GetCount() > 0) 
        {
           read_comms_cmd();
        }
        
        /*
        // Retrigger each time USB communication is reconfigured - Alternatively, use a button to trigger an initial pulse.
        if (reconfigured) {
            Trigger_Reg_Write(1);
        }
        */
        
        
        led_test++;
        
        //Led_Red_Write((led_test >> 0) & 0x01);
        Led_Green_Write((led_test >> 1) & 0x01);
        Led_Blue_Write((led_test >> 2) & 0x01);   

        if (Led_Key_Read() == 0)
        {
            led_test = 0;
        }    
    }
}

int8_t imu_bmi160_init(void)
{
    /* IMU init */
    sensor.id = BMI160_I2C_ADDR;
    sensor.interface = BMI160_I2C_INTF;
    sensor.read = bmi160_psoc_i2c_read;
    sensor.write = bmi160_psoc_i2c_write;
    sensor.delay_ms = bmi160_psoc_delay_ms;
    
    int8_t rslt = BMI160_OK;
    rslt = bmi160_soft_reset(&sensor);
    
    rslt = BMI160_OK;
    rslt = bmi160_init(&sensor);
    /*
    // After the above function call, 
    // accel and gyro parameters in the device structure 
    // are set with default values, 
    // found in the datasheet of the sensor
    */

   return rslt;
}

int8_t imu_bmi160_config(void)
{
    int8_t rslt = BMI160_OK;

    /* Select the Output data rate, range of accelerometer sensor */
    // sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    // sensor.accel_cfg.bw = BMI160_ACCEL_BW_RES_AVG8;
    #define IMU_ACC_SCALE 4

    /* Select the power mode of accelerometer sensor */
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    //sensor.gyro_cfg.range = BMI160_GYRO_RANGE_1000_DPS;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_500_DPS;
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    // sensor.gyro_cfg.bw = BMI160_GYRO_BW_OSR2_MODE;
    #define IMU_GYO_SCALE 500

    /* Select the power mode of Gyroscope sensor */
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&sensor);    

    return rslt;
}

int8_t imu_bmi160_enable_step_counter(void)
{
    int8_t rslt = BMI160_OK;
    uint8_t step_enable = 1;//enable the step counter

    rslt = bmi160_set_step_counter(step_enable,  &sensor);

    return rslt;

}

// 1ms system tick callback interrupt function
void sys_clock_ms_callback(void){
    sys_clock_cur_ms ++; // increment ms counter by 1
}

/**
 * @brief Interrupt handler for Shutter Active pin
 */
void Isr_shutter_handler(void)
{
    /* Set interrupt flag */
	frame_status = NEW_FRAME;
    uint8_t pulse_state_val = get_pulse_state();
    /* If pulse mode enabled, alternate LED/LASER.
       Currently, hardware only support Lasers. 
       Ideally, could add extra logic to only write to
       PWM if state was changed vs. every frame. */ 
    switch (pulse_state_val)
    {
        case PULSE_STATE_ALL_OFF:
            PWM_LASER_WriteCompare(PWM_LASER_OFF);
            /* Disable LED if PWM was supported */
            break;
        case PULSE_STATE_LED:
            light_status = LSR_DISABLE;
            /* Set LED to its configured PWM val if supported */
            break;
        case PULSE_STATE_LASER:
            light_status = LSR_ENABLE;
            PWM_LASER_WriteCompare(get_laser_val());
        case PULSE_STATE_ALTERNATE:
           if (light_status == LSR_ENABLE) {
                light_status = LSR_DISABLE;
                PWM_LASER_WriteCompare(PWM_LASER_OFF);
            }
            else if (light_status == LSR_DISABLE) {
                light_status = LSR_ENABLE;
                PWM_LASER_WriteCompare(get_laser_val());
            }
            break;
        default:
            /* Error case */
            break;
    }
        
    /* Trigger a new camera frame */
    Trigger_Reg_Write(1);
    
    /* Clears the pin interrupt */
    Exposure_Active_ClearInterrupt();
    /* Clears the pending pin interrupt */
    isr_EXPOSURE_ACT_ClearPending();

}
/* [] END OF FILE */
