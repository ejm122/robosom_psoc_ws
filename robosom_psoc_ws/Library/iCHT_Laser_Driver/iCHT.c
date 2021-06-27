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

/* [] END OF FILE */
#include "iCHT_Defines.h"
#include "project.h"

#include "stdio.h"
#include "usb_comm.h"
#define USBFS_DEVICE    (0u)
#define USBUART_BUFFER_SIZE (256u)
uint8 buffer[USBUART_BUFFER_SIZE];

int8_t ICHT_read_register(uint8_t addr, uint8_t *out_bytes, uint8_t len, const struct ICHT_config *conf)
{
    int8_t status = ICHT_NO_ERR;


    if (len == 0 || out_bytes == NULL || conf == NULL || conf->read == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    status = conf->read(conf->device_number, addr, out_bytes, len);
    
    if (status != ICHT_NO_ERR)
    {
        return status;
    }

    return status;
}

int8_t ICHT_write(uint8_t addr, uint8_t *in_bytes, uint8_t len, const struct ICHT_config *conf)
{
    int8_t status = ICHT_NO_ERR;


    if (len == 0 || in_bytes == NULL || conf == NULL || conf->write == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    status = conf->write(conf->device_number, addr, in_bytes, len);
    
    if (status != ICHT_NO_ERR)
    {
        return ICHT_I2C_ERR;
    }

    return status;
}

/** @brief PSOC-specific implementation of I2C Read */
int8_t ICHT_psoc_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    
    int status;
    int i;

    /* Write control byte without stop */
    status = I2C_1_MasterSendStart(dev_addr, I2C_1_WRITE_XFER_MODE);

    if(status == I2C_1_MSTR_NO_ERROR) /* Check if transfer completed without errors */
    {
        /* Write control byte */
        status = I2C_1_MasterWriteByte(reg_addr);
    }

    /* Read data array */
    status = I2C_1_MasterSendRestart(dev_addr, I2C_1_READ_XFER_MODE);

    if(I2C_1_MSTR_NO_ERROR == status) /* Check if transfer completed without errors */
    {
        /* Read array of bytes */
        for(i=0; i<len; i++)
        {
            if(i < len-1)
            {
                data[i] = I2C_1_MasterReadByte(I2C_1_ACK_DATA); /* non-last byte send ACK */
            }
            else
            {
                data[i] = I2C_1_MasterReadByte(I2C_1_NAK_DATA); /* last byte send NACK */
            }
        }
    }

    I2C_1_MasterSendStop(); /* Send Stop */

    return status;
}

/** @brief PSOC-specific implementation of I2C Write */
int8_t ICHT_psoc_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int status;
    int i;

    status = I2C_1_MasterSendStart(dev_addr, I2C_1_WRITE_XFER_MODE);

    if(status == I2C_1_MSTR_NO_ERROR) /* Check if transfer completed without errors */
    {
        /* Write register byte */
        status = I2C_1_MasterWriteByte(reg_addr);
    }

    if(status == I2C_1_MSTR_NO_ERROR) /* Check if transfer completed without errors */
    {
        /* Send data array of len bytes */
        for(i=0; i<len; i++)
        {
            /* send data array */
            status = I2C_1_MasterWriteByte(data[i]);

            /* if has error then break the loop */
            if(status != I2C_1_MSTR_NO_ERROR) 
            {
                break;
            }
        }
    }

    I2C_1_MasterSendStop(); /* Send Stop */

    return status;
}

/* Local helper function declarations */
void ICHT_decode_ADSNF_RACC_Config(uint8_t config_reg,
                                   struct ICHT_ADSNF_RACC_Config *regs);
void ICHT_decode_status_reg_LSB(uint8_t status_reg,
                                struct ICHT_Status_Regs_R *regs);
void ICHT_decode_status_reg_MSB(uint8_t status_reg,
                                struct ICHT_Status_Regs_R *regs);
void ICHT_decode_temp(uint8_t temp_reg, uint8_t *temp);
void ICHT_decode_ADC_LSB(uint8_t adc_reg_LSB, struct ICHT_ADC_Val_R *regs);
void ICHT_decode_ADC_MSB(uint8_t adc_reg_MSB, struct ICHT_ADC_Val_R *regs);
void ICHT_decode_chip_rev(uint8_t rev_reg, uint8_t *rev);
void ICHT_decode_ADC_Config(uint8_t adc_config_reg, struct ICHT_ADC_Config *regs);

int8_t ICHT_init_test(struct ICHT_config *conf)
{
    int status = ICHT_NO_ERR;
    int status_2 = ICHT_NO_ERR;
    struct ICHT_Status_Regs_R regs;
    conf->device_number = ICHT_DEFAULT_SLAVE_ADDR;
    conf->read = &ICHT_psoc_i2c_read;
    conf->write = &ICHT_psoc_i2c_write;
    sprintf((char *)buffer, "Starting INIT!\n");
    CyDelay(10);
    usb_put_string((char8 *)buffer);
    status = ICHT_get_status_regs(conf, &regs);
    if (status != ICHT_NO_ERR)
    {
        sprintf((char *)buffer, "Failed status reg read %d!\n", status);
        usb_put_string((char8 *)buffer);
        CyDelay(10);
        //return status;   
    }
    
    uint8_t rev;
    status_2 = ICHT_get_chip_rev(conf, &rev);
    if (status_2 != ICHT_NO_ERR)
    {
        sprintf((char *)buffer, "Failed CHIPREV read! %d \n", status_2);
        usb_put_string((char8 *)buffer);
        CyDelay(10);
        //return status_2;   
    }
    else {
        sprintf((char *)buffer, "CHIPREV: %x\n", rev);
        usb_put_string((char8 *)buffer);
        CyDelay(10);
    }
    if (status != ICHT_NO_ERR) return status;
    else {    
        sprintf((char *)buffer, "Flags: CFGTIMO %x INITRAM %x LDKSAT1 %x LDKSAT2 %x MPAC1 %x MPAC2 %x MEMERR %x MONC1 %x MONC2 %x OSCERR %x OCV1 %x OCV2 %x OVT %x PDOVDD %x\n", 
        regs.CFGTIMO, regs.INITRAM, regs.LDKSAT1, regs.LDKSAT2, regs.MAPC1, regs.MAPC2, regs.MEMERR, regs.MONC1, regs.MONC2, regs.OSCERR, regs.OVC1, regs.OVC2, regs.OVT, regs.PDOVDD);
        usb_put_string((char8 *)buffer);      
        CyDelay(10);
    }
    return status;
}

/** @brief Configures the iCHT driver structs with default write settings */
void ICHT_init_structs(struct ICHT_config *conf) {
    conf->device_number = ICHT_DEFAULT_SLAVE_ADDR;
    conf->read = &ICHT_psoc_i2c_read;
    conf->write = &ICHT_psoc_i2c_write;
    // Zero the struct
    struct ICHT_reg_list reg_list = {0};

    struct ICHT_ADC_Val_R ADC_Val_R = {
        .channel = ICHT_CHANNEL_1,
        .ADC = 0
    };
    reg_list.ADC1 = ADC_Val_R;
    ADC_Val_R.channel = ICHT_CHANNEL_2;
    reg_list.ADC2 = ADC_Val_R;
    
    // Default configuration settings for W channels
    struct ICHT_ADC_Config ADC_Config = {
        .channel = ICHT_CHANNEL_1,
        .disable_PLR = true,
        .enable_external_capacitor = true,
        .enable_offset_compensation = true,
        .mode = ICHT_APC_ENABLE,
        .source = ICHT_ADCC_SRC_DISABLED
    };
     
    // Configure settings same for CHN 1 and 2
    reg_list.ADCCONFIG1 = ADC_Config;
    ADC_Config.channel = ICHT_CHANNEL_2;
    reg_list.ADCCONFIG2 = ADC_Config;
    
    reg_list.RMD1.channel = ICHT_CHANNEL_1;
    reg_list.RMD2.channel = ICHT_CHANNEL_2;
    
    /* TODO - DOUBLE CHECK Electrical Characteristic 108
       at a supply voltage of 5V.
     */
    /* 
       RLD63NPC8 N-Diode Laser has max of 80mA
       Documentation isn't consistent, assuming
       max of RACC_LO is 80mA, can set to MAX
       Max voltage of 2.6V
     */
    reg_list.ILIM2.channel = ICHT_CHANNEL_2;
    reg_list.ILIM2.n = 0xFF;
    reg_list.ADSNFRACC.range_2 = ICHT_RACC_CURRENT_LO; 
    // Not high enough to prevent dmg, 5V source - 1.2 = 3.8 > 2.6
    reg_list.REGCONFIG2.channel = ICHT_CHANNEL_2;
    reg_list.REGCONFIG2.sat_threshold = ICHT_RLDKS_VLDK_LT_1_2V;
    
    /* 
       D405-120 M diode has a max of 150mA
       Datasheet isn't consistent..
       Assuming typical shutdown resolution D_I(LDK)
       of 4 when RACC = HIGH, n = 38?
       Max voltage of 6V
     */
    reg_list.ILIM1.channel = ICHT_CHANNEL_1;
    reg_list.ILIM1.n = 38;
    reg_list.ADSNFRACC.range_1 = ICHT_RACC_CURRENT_HI;
    reg_list.REGCONFIG1.channel = ICHT_CHANNEL_1;
    reg_list.REGCONFIG1.sat_threshold = ICHT_RLDKS_VLDK_LT_0_5V;
    
    reg_list.mode = ICHT_MODE_SETTING_OP;
    conf->regs = reg_list;
}

/** @brief Reads all regs and updates them in the provided struct */
int8_t ICHT_read_all_regs(struct ICHT_config *conf, struct ICHT_reg_list *reg_list) {
   uint8_t status = ICHT_get_status_regs(conf, &(reg_list->STATUS));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_temp(conf, &(reg_list->TEMP_R));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_ADC(conf, &(reg_list->ADC1));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_ADC(conf, &(reg_list->ADC2));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_chip_rev(conf, &(reg_list->CHIP_REV_R));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_ADC_Config(conf, &(reg_list->ADCCONFIG1));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_overcurrent_thresh(conf, &(reg_list->ILIM1));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_monitor_resistance(conf, &(reg_list->RMD1));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_Regulator_Config(conf, &(reg_list->REGCONFIG1));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_ADC_Config(conf, &(reg_list->ADCCONFIG2));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_overcurrent_thresh(conf, &(reg_list->ILIM2));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_monitor_resistance(conf, &(reg_list->RMD2));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_Regulator_Config(conf, &(reg_list->REGCONFIG2));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_ADSNF_RACC_Config(conf, &(reg_list->ADSNFRACC));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_Merge_RDCO_Config(conf, &(reg_list->MERGERDCO));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_mode(conf, &(reg_list->mode));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_get_error_regs(conf, &(reg_list->ERROR));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }
   return status;
}

/** Performs the start-up sequence for the iCHT driver, clearing status regs and writing out the reg_list.
    The initial results of the read status registers will be placed in reg_list,
    and the final results of the read status registers and the full system status will be placed in conf's regs.
 */
int8_t ICHT_configure_driver(struct ICHT_config *conf, struct ICHT_reg_list *reg_list)
{
    reg_list->mode = ICHT_MODE_SETTING_CONFIG;
    uint8_t status = ICHT_set_mode(conf, &(reg_list->mode));
    if (status != ICHT_NO_ERR) return status;
    
    // Clears the status registers
    status = ICHT_get_status_regs(conf, &(reg_list->STATUS));
    if (status != ICHT_NO_ERR) return status;
    // Write out all registers
    status = ICHT_write_all_regs(conf, reg_list);
    if (status != ICHT_NO_ERR) return status;
    
    // Return to operation mode
    reg_list->mode = ICHT_MODE_SETTING_OP;
    status = ICHT_set_mode(conf, &(reg_list->mode));
    if (status != ICHT_NO_ERR) return status;
    
    // Get resulting driver status
    return ICHT_read_all_regs(conf, &(conf->regs));
}

/** @brief Writes all writeable regs from the provided struct */
int8_t ICHT_write_all_regs(struct ICHT_config *conf, struct ICHT_reg_list *reg_list)
{
   uint8_t status = ICHT_set_overcurrent_thresh(conf, &(reg_list->ILIM1));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_set_monitor_resistance(conf, &(reg_list->RMD1));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_set_Regulator_Config(conf, &(reg_list->REGCONFIG1));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_set_ADC_Config(conf, &(reg_list->ADCCONFIG2));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_set_overcurrent_thresh(conf, &(reg_list->ILIM2));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_set_monitor_resistance(conf, &(reg_list->RMD2));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_set_Regulator_Config(conf, &(reg_list->REGCONFIG2));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_set_ADSNF_RACC_Config(conf, &(reg_list->ADSNFRACC));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   status = ICHT_set_Merge_RDCO_Config(conf, &(reg_list->MERGERDCO));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }

   /* Mode isn't written due to nature of changing configuration modes
   status = ICHT_set_mode(conf, &(reg_list->mode));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }
   */

   status = ICHT_set_error_regs(conf, &(reg_list->ERROR));
   if (status != ICHT_NO_ERR)
   {
       return status;
   }
   return status;
}

/** @brief Reads the status registers, acknowledging any errors.
           Must be read after power-on.
 */
int8_t ICHT_get_status_regs(const struct ICHT_config *conf, 
                            struct ICHT_Status_Regs_R *regs)
{
    uint8_t status_reg_LSB, status_reg_MSB;
    int8_t status;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;   
    }
    
    /* Reads the status, LSB byte first */
    status = ICHT_read_register(ICHT_STATUS_ADDR_LSB, &status_reg_LSB, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;
    }
    
    status = ICHT_read_register(ICHT_STATUS_ADDR_MSB, &status_reg_MSB, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;
    }
    
    ICHT_decode_status_reg_LSB(status_reg_LSB, regs);
    ICHT_decode_status_reg_MSB(status_reg_MSB, regs);
    return status;
}

void ICHT_decode_status_reg_LSB(uint8_t status_reg,
                                struct ICHT_Status_Regs_R *regs)
{
    regs->CFGTIMO = ICHT_REG_GET(status_reg, ICHT_CFGTIMO);
    regs->OSCERR = ICHT_REG_GET(status_reg, ICHT_OSCERR);
    regs->OVC1 = ICHT_REG_GET(status_reg, ICHT_OVC1);
    regs->OVC2 = ICHT_REG_GET(status_reg, ICHT_OVC2);
    regs->OVT = ICHT_REG_GET(status_reg, ICHT_OVT);
    regs->MEMERR = ICHT_REG_GET(status_reg, ICHT_MEMERR);
    regs->PDOVDD = ICHT_REG_GET(status_reg, ICHT_PDOVDD);
    regs->INITRAM = ICHT_REG_GET(status_reg, ICHT_INITRAM);
}

void ICHT_decode_status_reg_MSB(uint8_t status_reg,
                                struct ICHT_Status_Regs_R *regs)
{
    regs->LDKSAT2 = ICHT_REG_GET(status_reg, ICHT_LDKSAT2);
    regs->MONC2 = ICHT_REG_GET(status_reg, ICHT_MONC2);    
    regs->MAPC2 = ICHT_REG_GET(status_reg, ICHT_MAPC2);   
    regs->LDKSAT1 = ICHT_REG_GET(status_reg, ICHT_LDKSAT1);
    regs->MONC1 = ICHT_REG_GET(status_reg, ICHT_MONC1);    
    regs->MAPC1 = ICHT_REG_GET(status_reg, ICHT_MAPC1);
}

/** @brief Reads the internal chip temp register
 */
int8_t ICHT_get_temp(const struct ICHT_config *conf, uint8_t *temp)
{
    uint8_t temp_reg;
    int8_t status;
    
    if (temp == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    /* Reads the status, LSB byte first */
    status = ICHT_read_register(ICHT_TEMP_ADDR, &temp_reg, 1, conf);
    
    if (status != ICHT_NO_ERR)
    {
        return status;
    }
    
    ICHT_decode_temp(temp_reg, temp);
    return status;
}

void ICHT_decode_temp(uint8_t temp_reg, uint8_t *temp)
{
    *temp = temp_reg;
}

/** @brief Reads the current value of the ADC for a given channel
 */
int8_t ICHT_get_ADC(const struct ICHT_config *conf,
                    struct ICHT_ADC_Val_R *regs)
{
    uint8_t adc_reg_LSB, adc_reg_MSB;
    uint8_t addr;
    int8_t status;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if (ICHT_INVALID_BOUNDS(regs->channel, ICHT_CHANNEL_NUM))
    {
        return ICHT_INVALID_VAR;   
    }
    
    addr = (regs->channel == ICHT_CHANNEL_1) ? ICHT_ADC1_BYTE_0_ADDR : 
            ICHT_ADC2_BYTE_0_ADDR;
    
    status = ICHT_read_register(addr, &adc_reg_LSB, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    
    addr++;
    
    status = ICHT_read_register(addr, &adc_reg_MSB, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    
    ICHT_decode_ADC_LSB(adc_reg_LSB, regs);
    ICHT_decode_ADC_MSB(adc_reg_MSB, regs);
    
    return status;
}

void ICHT_decode_ADC_LSB(uint8_t adc_reg_LSB, struct ICHT_ADC_Val_R *regs)
{
    regs->ADC = adc_reg_LSB;
}

void ICHT_decode_ADC_MSB(uint8_t adc_reg_MSB, struct ICHT_ADC_Val_R *regs)
{
    // Assumes LSB was already set
    regs->ADC = (regs->ADC & 0xFF) | (adc_reg_MSB << 8);
}

/** @brief Reads the chip revision register
 */
int8_t ICHT_get_chip_rev(const struct ICHT_config *conf, uint8_t *rev)
{
    uint8_t rev_reg;
    int8_t status;
    
    if (rev == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    /* Reads the status, LSB byte first */
    status = ICHT_read_register(ICHT_CHIPREV_ADDR, &rev_reg, 1, conf);
    
    if (status != ICHT_NO_ERR)
    {
        return status;
    }
    
    ICHT_decode_chip_rev(rev_reg, rev);

    return status;
}

void ICHT_decode_chip_rev(uint8_t rev_reg, uint8_t *rev)
{
    *rev = rev_reg;
}

/** @brief Reads various ADC variables for a given channel
 */
int8_t ICHT_get_ADC_Config(const struct ICHT_config *conf,
                           struct ICHT_ADC_Config *regs)
{
    uint8_t adc_config_reg;
    uint8_t addr;
    int8_t status;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if (ICHT_INVALID_BOUNDS(regs->channel, ICHT_CHANNEL_NUM))
    {
        return ICHT_INVALID_VAR;   
    }
    
    addr = (regs->channel == ICHT_CHANNEL_1) ? ICHT_ADC_1_CONFIG_ADDR : 
            ICHT_ADC_2_CONFIG_ADDR;
    
    status = ICHT_read_register(addr, &adc_config_reg, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    
    ICHT_decode_ADC_Config(adc_config_reg, regs);
    
    return status;
}

void ICHT_decode_ADC_Config(uint8_t adc_config_reg, struct ICHT_ADC_Config *regs)
{
    regs->source = ICHT_REG_GET(adc_config_reg, ICHT_ADCC);
    regs->enable_offset_compensation = ICHT_REG_GET(adc_config_reg, ICHT_EOC);
    regs->disable_channel = ICHT_REG_GET(adc_config_reg, ICHT_DISC);
    regs->disable_PLR = ICHT_REG_GET(adc_config_reg, ICHT_DISP);
    regs->enable_external_capacitor = ICHT_REG_GET(adc_config_reg, ICHT_ECIE);
    regs->mode = ICHT_REG_GET(adc_config_reg, ICHT_EACC);
}

/** @brief Writes various ADC variables for a given channel
 */
int8_t ICHT_set_ADC_Config(const struct ICHT_config *conf,
                           struct ICHT_ADC_Config *regs)
{
    uint8_t adc_config_reg = 0;
    uint8_t addr;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if ((ICHT_INVALID_BOUNDS(regs->channel, ICHT_CHANNEL_NUM)) ||
        (ICHT_INVALID_BOUNDS(regs->source, ICHT_ADCC_SRC)) ||
        (ICHT_INVALID_BOUNDS(regs->mode, ICHT_EACC_MODE)))
    {
        return ICHT_INVALID_VAR;   
    }
    
    addr = (regs->channel == ICHT_CHANNEL_1) ? ICHT_ADC_1_CONFIG_ADDR : 
            ICHT_ADC_2_CONFIG_ADDR;
    
    adc_config_reg = ICHT_REG_SET(adc_config_reg, ICHT_ADCC, regs->source);
    adc_config_reg = ICHT_REG_SET(adc_config_reg, ICHT_EOC, 
                                  regs->enable_offset_compensation);
    adc_config_reg = ICHT_REG_SET(adc_config_reg, ICHT_DISC, 
                                  regs->disable_channel);
    adc_config_reg = ICHT_REG_SET(adc_config_reg, ICHT_DISP, 
                                  regs->disable_PLR);
    adc_config_reg = ICHT_REG_SET(adc_config_reg, ICHT_ECIE, 
                                  regs->enable_external_capacitor);
    adc_config_reg = ICHT_REG_SET(adc_config_reg, ICHT_EACC, regs->mode);
    
    return ICHT_write(addr, &adc_config_reg, 1, conf);
}


/** @brief Reads the overlimit threshold (ILIM) of a given channel
 */
int8_t ICHT_get_overcurrent_thresh(const struct ICHT_config *conf,
                                   struct ICHT_Overcurrent_Threshold *regs)
{
    uint8_t ilim_reg;
    uint8_t addr;
    int8_t status;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if (ICHT_INVALID_BOUNDS(regs->channel, ICHT_CHANNEL_NUM))
    {
        return ICHT_INVALID_VAR;   
    }
    
    addr = (regs->channel == ICHT_CHANNEL_1) ? ICHT_ILIM1_ADDR : 
            ICHT_ILIM2_ADDR;
    
    status = ICHT_read_register(addr, &ilim_reg, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    
    regs->n = ilim_reg;
    
    return status;
}

/** @brief Writes the overlimit threshold (ILIM) of a given channel
 */
int8_t ICHT_set_overcurrent_thresh(const struct ICHT_config *conf,
                                   struct ICHT_Overcurrent_Threshold *regs)
{
    uint8_t ilim_reg = 0;
    uint8_t addr;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if ((ICHT_INVALID_BOUNDS(regs->channel, ICHT_CHANNEL_NUM)) ||
        (ICHT_INVALID_BOUNDS(regs->n, ICHT_ILIM_RANGE)))
    {
        return ICHT_INVALID_VAR;   
    }
    
    addr = (regs->channel == ICHT_CHANNEL_1) ? ICHT_ILIM1_ADDR : 
            ICHT_ILIM2_ADDR;
    
    ilim_reg = regs->n;
    
    return ICHT_write(addr, &ilim_reg, 1, conf);
}

/** @brief Reads the internal monitor resistance (RMD) of a given channel
 */
int8_t ICHT_get_monitor_resistance(const struct ICHT_config *conf,
                                   struct ICHT_Internal_Monitor_Resistance *regs)
{
    uint8_t rmd_reg;
    uint8_t addr;
    int8_t status;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if (ICHT_INVALID_BOUNDS(regs->channel, ICHT_CHANNEL_NUM))
    {
        return ICHT_INVALID_VAR;   
    }
    
    addr = (regs->channel == ICHT_CHANNEL_1) ? ICHT_RMD1_ADDR : 
            ICHT_RMD2_ADDR;
    
    status = ICHT_read_register(addr, &rmd_reg, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    
    regs->n = rmd_reg;
    
    return status;
}

/** @brief Writes the internal monitor resistance (RMD) of a given channel
 */
int8_t ICHT_set_monitor_resistance(const struct ICHT_config *conf,
                                   struct ICHT_Internal_Monitor_Resistance *regs)
{
    uint8_t rmd_reg = 0;
    uint8_t addr;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if (ICHT_INVALID_BOUNDS(regs->channel, ICHT_CHANNEL_NUM))
    {
        return ICHT_INVALID_VAR;   
    }
    
    addr = (regs->channel == ICHT_CHANNEL_1) ? ICHT_RMD1_ADDR : 
            ICHT_RMD2_ADDR;
    
    rmd_reg = regs->n;
    
    return ICHT_write(addr, &rmd_reg, 1, conf);
}


/** @brief Reads various regulator variables for a given channel
 */
int8_t ICHT_get_Regulator_Config(const struct ICHT_config *conf,
                                 struct ICHT_Regulator_Config *regs)
{
    uint8_t reg_config_reg_LSB, reg_config_reg_MSB;
    uint8_t addr;
    int8_t status;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if (ICHT_INVALID_BOUNDS(regs->channel, ICHT_CHANNEL_NUM))
    {
        return ICHT_INVALID_VAR;   
    }
    
    addr = (regs->channel == ICHT_CHANNEL_1) ? ICHT_REG_1_CONFIG : 
            ICHT_REG_2_CONFIG;
    
    status = ICHT_read_register(addr, &reg_config_reg_LSB, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    addr++;
    status = ICHT_read_register(addr, &reg_config_reg_MSB, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    
    regs->compensation = ICHT_REG_GET(reg_config_reg_LSB, ICHT_COMP);
    regs->sat_threshold = ICHT_REG_GET(reg_config_reg_LSB, ICHT_RLDKS);
    regs->Vref = (ICHT_REG_GET(reg_config_reg_LSB, ICHT_REF_BYTE_1) << 8);
    regs->Vref |= reg_config_reg_MSB;
    
    return status;
}

/** @brief Writes various regulator variables for a given channel
 */
int8_t ICHT_set_Regulator_Config(const struct ICHT_config *conf,
                                 struct ICHT_Regulator_Config *regs)
{
    uint8_t reg_config_reg_LSB, reg_config_reg_MSB;
    uint8_t addr, ref_byte_0;
    uint8_t status;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if ((ICHT_INVALID_BOUNDS(regs->channel, ICHT_CHANNEL_NUM)) ||
        (ICHT_INVALID_BOUNDS(regs->compensation, ICHT_COMP_RANGE)) ||
        (ICHT_INVALID_BOUNDS(regs->sat_threshold, ICHT_RLDKS_SAT)) ||
        (ICHT_INVALID_BOUNDS(regs->Vref, ICHT_REF_RANGE)))
    {
        return ICHT_INVALID_VAR;   
    }
    
    addr = (regs->channel == ICHT_CHANNEL_1) ? ICHT_REG_1_CONFIG : 
            ICHT_REG_2_CONFIG;
    
    reg_config_reg_LSB = 0;
    reg_config_reg_LSB = ICHT_REG_SET(reg_config_reg_LSB, ICHT_COMP, 
                                     regs->compensation);
    reg_config_reg_LSB = ICHT_REG_SET(reg_config_reg_LSB, ICHT_RLDKS, 
                                     regs->sat_threshold);
    
    ref_byte_0 = ((regs->Vref >> 8) & ICHT_REF_BYTE_1_MASK); 
    reg_config_reg_LSB = ICHT_REG_SET(reg_config_reg_LSB, ICHT_REF_BYTE_1, 
                                     ref_byte_0);
    
    reg_config_reg_MSB = (regs->Vref & ICHT_REF_BYTE_0_MASK);
    
    status = ICHT_write(addr, &reg_config_reg_LSB, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    
    addr++;
    
    return ICHT_write(addr, &reg_config_reg_MSB, 1, conf);
}

/** @brief Reads various the ADSNF and RACC variables for both channels
 */
int8_t ICHT_get_ADSNF_RACC_Config(const struct ICHT_config *conf,
                                  struct ICHT_ADSNF_RACC_Config *regs)
{
    uint8_t config_reg;
    int8_t status;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    status = ICHT_read_register(ICHT_ADSNF_RACC_ADDR, &config_reg, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    
    ICHT_decode_ADSNF_RACC_Config(config_reg, regs);
    return status;
}
    
void ICHT_decode_ADSNF_RACC_Config(uint8_t config_reg,
                                   struct ICHT_ADSNF_RACC_Config *regs)
{
    regs->range_1 = ICHT_REG_GET(config_reg, ICHT_RACC1);
    regs->range_2 = ICHT_REG_GET(config_reg, ICHT_RACC2);
    regs->source_1 = ICHT_REG_GET(config_reg, ICHT_ADSNF1);
    regs->source_2 = ICHT_REG_GET(config_reg, ICHT_ADSNF2);
}

/** @brief Writes various the ADSNF and RACC variables for both channels
 */
int8_t ICHT_set_ADSNF_RACC_Config(const struct ICHT_config *conf,
                                  struct ICHT_ADSNF_RACC_Config *regs)
{
    uint8_t config_reg = 0;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if ((ICHT_INVALID_BOUNDS(regs->range_1, ICHT_RACC_RANGE)) ||
        (ICHT_INVALID_BOUNDS(regs->range_2, ICHT_RACC_RANGE)) ||
        (ICHT_INVALID_BOUNDS(regs->source_1, ICHT_ADSNF_SRC)) ||
        (ICHT_INVALID_BOUNDS(regs->source_1, ICHT_ADSNF_SRC)))
    {
        return ICHT_INVALID_VAR;   
    }

    
    config_reg = ICHT_REG_SET(config_reg, ICHT_RACC1, regs->range_1);
    config_reg = ICHT_REG_SET(config_reg, ICHT_RACC2, regs->range_2);
    config_reg = ICHT_REG_SET(config_reg, ICHT_ADSNF1, regs->source_1);
    config_reg = ICHT_REG_SET(config_reg, ICHT_ADSNF2, regs->source_2);

    return ICHT_write(ICHT_ADSNF_RACC_ADDR, &config_reg, 1, conf);
}

/** @brief Reads the merge and DCO current configuration
 */
int8_t ICHT_get_Merge_RDCO_Config(const struct ICHT_config *conf,
                                  struct ICHT_Merge_RDCO_Config *regs)
{
    uint8_t config_reg;
    int8_t status;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    status = ICHT_read_register(ICHT_MERGE_RDCO_ADDR, &config_reg, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    
    regs->en_merge = ICHT_REG_GET(config_reg, ICHT_MERGE);
    regs->DCO_current_config = ICHT_REG_GET(config_reg, ICHT_RDCO);
    
    return status;
}

/** @brief Writes the merge and DCO current configuration
 */
int8_t ICHT_set_Merge_RDCO_Config(const struct ICHT_config *conf,
                                  struct ICHT_Merge_RDCO_Config *regs)
{
    uint8_t config_reg = 0;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    if (ICHT_INVALID_BOUNDS(regs->DCO_current_config, ICHT_RDCO_RANGE))
    {
        return ICHT_INVALID_VAR;   
    }

    
    config_reg = ICHT_REG_SET(config_reg, ICHT_RDCO, regs->DCO_current_config);
    config_reg = ICHT_REG_SET(config_reg, ICHT_MERGE, regs->en_merge);

    return ICHT_write(ICHT_MERGE_RDCO_ADDR, &config_reg, 1, conf);
}

/** @brief Reads the mode of the driver
 */
int8_t ICHT_get_mode(const struct ICHT_config *conf,
                     ICHT_MODE_SETTING *mode)
{
    uint8_t mode_reg;
    int8_t status;

    if (mode == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    status = ICHT_read_register(ICHT_MODE_ADDR, &mode_reg, 1, conf);
    if (status != ICHT_NO_ERR)
    {
        return status;   
    }
    
    *mode = mode_reg;
    
    return status;
}

/** @brief Reads the mode of the driver
 */
int8_t ICHT_set_mode(const struct ICHT_config *conf,
                     ICHT_MODE_SETTING *mode)
{
    uint8_t mode_reg = 0;
    
    if (mode == NULL)
    {
        return ICHT_NULL_PTR;
    }
    
    mode_reg = *mode;
    
    if (ICHT_INVALID_BOUNDS(mode_reg, ICHT_MODE_SETTING))
    {
        return ICHT_INVALID_VAR;   
    }
    
    return ICHT_write(ICHT_MODE_ADDR, &mode_reg, 1, conf);
}

/** @brief Reads the misc error registers.
 */
int8_t ICHT_get_error_regs(const struct ICHT_config *conf, 
                            struct ICHT_Error_Regs *regs)
{
    uint8_t status_reg;
    int8_t status;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;   
    }
    
    /* Reads the status, LSB byte first */
    status = ICHT_read_register(ICHT_ERROR_REGS_ADDR, &status_reg, 1, conf);
    
    if (status != ICHT_NO_ERR)
    {
        return status;
    }
    
    regs->SOSCERR = ICHT_REG_GET(status_reg, ICHT_SOSCERR);
    regs->SOVC2 = ICHT_REG_GET(status_reg, ICHT_SOVC2);
    regs->SOVC1 = ICHT_REG_GET(status_reg, ICHT_SOVC1);
    regs->SOVT = ICHT_REG_GET(status_reg, ICHT_SOVT);
    regs->MLDKSAT2 = ICHT_REG_GET(status_reg, ICHT_MLDKSAT2);
    regs->MLDKSAT1 = ICHT_REG_GET(status_reg, ICHT_MLDKSAT1);
    regs->MMONC = ICHT_REG_GET(status_reg, ICHT_MMONC);
    regs->MOSCERR = ICHT_REG_GET(status_reg, ICHT_MOSCERR);

    return status;
}


/** @brief Writes the misc error registers.
 */
int8_t ICHT_set_error_regs(const struct ICHT_config *conf, 
                            struct ICHT_Error_Regs *regs)
{
    uint8_t status_reg = 0;
    
    if (regs == NULL)
    {
        return ICHT_NULL_PTR;   
    }
    
    status_reg = ICHT_REG_SET(status_reg, ICHT_SOSCERR, regs->SOSCERR);
    status_reg = ICHT_REG_SET(status_reg, ICHT_SOVC2, regs->SOVC2);
    status_reg = ICHT_REG_SET(status_reg, ICHT_SOVC1, regs->SOVC1);
    status_reg = ICHT_REG_SET(status_reg, ICHT_SOVT, regs->SOVT);
    status_reg = ICHT_REG_SET(status_reg, ICHT_MLDKSAT2, regs->MLDKSAT2);
    status_reg = ICHT_REG_SET(status_reg, ICHT_MLDKSAT1, regs->MLDKSAT1);
    status_reg = ICHT_REG_SET(status_reg, ICHT_MMONC, regs->MMONC);
    status_reg = ICHT_REG_SET(status_reg, ICHT_MOSCERR, regs->MOSCERR);

    return ICHT_write(ICHT_ERROR_REGS_ADDR, &status_reg, 1, conf);
}
