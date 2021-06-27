/* ========================================
 * iCHT Dual CW Laser Diode Driver Defines
 *
 * Created 5/13/2021 by ebcohen@andrew.cmu.edu
 * ========================================
 */

#ifndef ICHT_DEFINES_H_
#define ICHT_DEFINES_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifndef NULL
#define NULL ((void *) 0)
#endif /* NULL */


/** -------- iCHT Register Defines ------------- */

/** Parameter Defines */

/** @brief Configuration for ADC from Channel 1 */
#define ICHT_ADC_1_CONFIG_ADDR              UINT8_C(0x10)
/** @brief Configuration for ADC from Channel 2 */
#define ICHT_ADC_2_CONFIG_ADDR              UINT8_C(0x15)
#define ICHT_ADCC_SHIFT                     UINT8_C(5)
#define ICHT_ADCC_MASK                      UINT8_C(0xE0) // 7:5
/** @brief Enable offset compensation for channel  1/2 */
#define ICHT_EOC_SHIFT                      UINT8_C(4)
#define ICHT_EOC_MASK                       UINT8_C(0x10) // 4
/** @brief Software disable for channel 1/2 */
#define ICHT_DISC_SHIFT                     UINT8_C(3)
#define ICHT_DISC_MASK                      UINT8_C(0x08) // 3
/** @brief Disable PLR for channel 1/2 */
#define ICHT_DISP_SHIFT                     UINT8_C(2)
#define ICHT_DISP_MASK                      UINT8_C(0x04) // 2
/** @brief Enable external CI capacitor for channel 1/2 */
#define ICHT_ECIE_SHIFT                     UINT8_C(1)
#define ICHT_ECIE_MASK                      UINT8_C(0x02) // 1
/** @brief Enable ACC mode for channel 1/2 */
#define ICHT_EACC_SHIFT                     UINT8_C(0)
#define ICHT_EACC_MASK                      UINT8_C(0x01) // 0

/** @brief Current limit at channel 1 */
#define ICHT_ILIM1_ADDR                     UINT8_C(0x11)
/** @brief Current limit at channel 2 */
#define ICHT_ILIM2_ADDR                     UINT8_C(0x16)
#define ICHT_ILIM_SHIFT                     UINT8_C(0)
#define ICHT_ILIM_MASK                      UINT8_C(0xFF) // 7:0

/** @brief Resistor at channel 1 */
#define ICHT_RMD1_ADDR                      UINT8_C(0x12)
/** @brief Resistor at channel 2 */
#define ICHT_RMD2_ADDR                      UINT8_C(0x17)
#define ICHT_RMD_SHIFT                      UINT8_C(0)
#define ICHT_RMD_MASK                       UINT8_C(0xFF) // 7:0

/** @brief Channel 1 regulator delay compensation */
#define ICHT_REG_1_CONFIG                   UINT8_C(0x13)
/** @brief Channel 2 regulator delay compensation */
#define ICHT_REG_2_CONFIG                   UINT8_C(0x18)
#define ICHT_COMP_SHIFT                     UINT8_C(4)
#define ICHT_COMP_MASK                      UINT8_C(0x70) // 6:4
/** @brief Channel 1/2 LDK saturation detector threshold */
#define ICHT_RLDKS_SHIFT                    UINT8_C(2)
#define ICHT_RLDKS_MASK                     UINT8_C(0x0C) // 3:2
/** @brief Voltage reference at channel 1/2 */
#define ICHT_REF_BYTE_1_SHIFT               UINT8_C(0)
#define ICHT_REF_BYTE_1_MASK                UINT8_C(0x03) // 1:0

/** @brief Voltage reference at channel 1 */
#define ICHT_REF1_BYTE_0_ADDR               UINT8_C(0x14) // 9:0
/** @brief Voltage reference at channel 2 */
#define ICHT_REF2_BYTE_0_ADDR               UINT8_C(0x19) // 9:0
#define ICHT_REF_BYTE_0_SHIFT               UINT8_C(0)
#define ICHT_REF_BYTE_0_MASK                UINT8_C(0xFF) // 7:0

/** @brief MDA force/sense for ADC measurement in channel 1 */
#define ICHT_ADSNF_RACC_ADDR                UINT8_C(0x1A)
#define ICHT_ADSNF1_SHIFT                   UINT8_C(2)
#define ICHT_ADSNF1_MASK                    UINT8_C(0x04) // 2
/** @brief MDA force/sense for ADC measurement in channel 2 */
#define ICHT_ADSNF2_SHIFT                   UINT8_C(6)
#define ICHT_ADSNF2_MASK                    UINT8_C(0x40) // 6
/** @brief Channel 1 ACC resistor mirror factor */
#define ICHT_RACC1_SHIFT                    UINT8_C(0)
#define ICHT_RACC1_MASK                     UINT8_C(0x01) // 0
/** @brief Channel 2 ACC resistor mirror factor */
#define ICHT_RACC2_SHIFT                    UINT8_C(4)
#define ICHT_RACC2_MASK                     UINT8_C(0x10) // 4

/** @brief MERGE channels 1 and 2, controlled by channel 1 */
#define ICHT_MERGE_RDCO_ADDR                UINT8_C(0x1B)
#define ICHT_MERGE_SHIFT                    UINT8_C(6)
#define ICHT_MERGE_MASK                     UINT8_C(0x40) // 6
/** @brief DC converter set point */
#define ICHT_RDCO_SHIFT                     UINT8_C(0)
#define ICHT_RDCO_MASK                      UINT8_C(0x3F) // 5:0

/** @brief Configuration / Operation mode selection */
#define ICHT_MODE_ADDR                      UINT8_C(0x1C)
#define ICHT_MODE_SHIFT                     UINT8_C(0)
#define ICHT_MODE_MASK                      UINT8_C(0x03) // 1:0

/** @brief LDKSAT1 error mask */
#define ICHT_ERROR_REGS_ADDR                UINT8_C(0x1D)
#define ICHT_MLDKSAT1_SHIFT                 UINT8_C(2)
#define ICHT_MLDKSAT1_MASK                  UINT8_C(0x04) // 2
/** @brief LDKSAT2 error mask */
#define ICHT_MLDKSAT2_SHIFT                 UINT8_C(3)
#define ICHT_MLDKSAT2_MASK                  UINT8_C(0x08) // 3
/** @brief MONC error mask */
#define ICHT_MMONC_SHIFT                    UINT8_C(1)
#define ICHT_MMONC_MASK                     UINT8_C(0x02) // 1
/** @brief OSCERR error mask */
#define ICHT_MOSCERR_SHIFT                  UINT8_C(0)
#define ICHT_MOSCERR_MASK                   UINT8_C(0x01) // 0
/** @brief Oscillator error simulation (watchdog timeout) */
#define ICHT_SOSCERR_SHIFT                  UINT8_C(7)
#define ICHT_SOSCERR_MASK                   UINT8_C(0x80) // 7
/** @brief Overcurrent event at channel 1 simulation */
#define ICHT_SOVC1_SHIFT                    UINT8_C(5)
#define ICHT_SOVC1_MASK                     UINT8_C(0x20) // 5
/** @brief Overcurrent event at channel 2 simulation */
#define ICHT_SOVC2_SHIFT                    UINT8_C(6)
#define ICHT_SOVC2_MASK                     UINT8_C(0x40) // 6
/** @brief Overtemperature event simulation */
#define ICHT_SOVT_SHIFT                     UINT8_C(4)
#define ICHT_SOVT_MASK                      UINT8_C(0x10) // 4

/** Status register addresses - R/O */

/** @brief RAM initialized. */
#define ICHT_STATUS_ADDR_LSB                UINT8_C(0x00)
#define ICHT_INITRAM_SHIFT                  UINT8_C(0)
#define ICHT_INITRAM_MASK                   UINT8_C(0x01) // 0
/** @brief Power-down event at VDD */
#define ICHT_PDOVDD_SHIFT                   UINT8_C(1)
#define ICHT_PDOVDD_MASK                    UINT8_C(0x02) // 1
/** @brief RAM memory validation error */
#define ICHT_MEMERR_SHIFT                   UINT8_C(2)
#define ICHT_MEMERR_MASK                    UINT8_C(0x04) // 2
/** @brief Overtemperature event */
#define ICHT_OVT_SHIFT                      UINT8_C(3)
#define ICHT_OVT_MASK                       UINT8_C(0x08) // 3
/** @brief Overcurrent at channel 2 */
#define ICHT_OVC2_SHIFT                     UINT8_C(4)
#define ICHT_OVC2_MASK                      UINT8_C(0x10) // 4
/** @brief Overcurrent at channel 1 */
#define ICHT_OVC1_SHIFT                     UINT8_C(5)
#define ICHT_OVC1_MASK                      UINT8_C(0x20) // 5
/** @brief Oscillator error (watchdog set) */
#define ICHT_OSCERR_SHIFT                   UINT8_C(6)
#define ICHT_OSCERR_MASK                    UINT8_C(0x40) // 6
/** @brief Configuration mode timeout event */
#define ICHT_CFGTIMO_SHIFT                  UINT8_C(7)
#define ICHT_CFGTIMO_MASK                   UINT8_C(0x80) // 8

/** @brief Channel 1 current state */
#define ICHT_STATUS_ADDR_MSB                UINT8_C(0x01)
#define ICHT_MAPC1_SHIFT                    UINT8_C(0)
#define ICHT_MAPC1_MASK                     UINT8_C(0x01) // 0
/** @brief Monitor channel 1 enabled at least once (latched) */
#define ICHT_MONC1_SHIFT                    UINT8_C(1)
#define ICHT_MONC1_MASK                     UINT8_C(0x02) // 1
/** @brief Channel 1 LDK saturation event */
#define ICHT_LDKSAT1_SHIFT                  UINT8_C(2)
#define ICHT_LDKSAT1_MASK                   UINT8_C(0x04) // 2
/** @brief Channel 2 current state */
#define ICHT_MAPC2_SHIFT                    UINT8_C(4)
#define ICHT_MAPC2_MASK                     UINT8_C(0x10) // 4
/** @brief Monitor channel 2 enabled at least once (latched) */
#define ICHT_MONC2_SHIFT                    UINT8_C(5)
#define ICHT_MONC2_MASK                     UINT8_C(0x20) // 5
/** @brief Channel 2 LDK saturation event */
#define ICHT_LDKSAT2_SHIFT                  UINT8_C(6)
#define ICHT_LDKSAT2_MASK                   UINT8_C(0x40) // 6

/** Measurement addresses - R/O */

/** @brief Chip temperature measurement */
#define ICHT_TEMP_ADDR                      UINT8_C(0x02)
#define ICHT_TEMP_SHIFT                     UINT8_C(0)
#define ICHT_TEMP_MASK                      UINT8_C(0xFF) // 7:0

/** @brief Channel 1 ADC 7:0 readout */
#define ICHT_ADC1_BYTE_0_ADDR               UINT8_C(0x04) // 9:0
/** @brief Channel 2 ADC 7:0 readout */
#define ICHT_ADC2_BYTE_0_ADDR               UINT8_C(0x06) // 9:0
#define ICHT_ADC_BYTE_0_SHIFT               UINT8_C(0)
#define ICHT_ADC_BYTE_0_MASK                UINT8_C(0xFF) // 7:0
/** @brief Channel 2 ADC 9:8 readout */
#define ICHT_ADC2_BYTE_1_ADDR               UINT8_C(0x05)
/** @brief Channel 1 ADC 9:8 readout */
#define ICHT_ADC1_BYTE_1_ADDR               UINT8_C(0x03)
#define ICHT_ADC_BYTE_1_SHIFT               UINT8_C(0)
#define ICHT_ADC_BYTE_1_MASK                UINT8_C(0x03) // 1:0

/** @brief Chip revision identification */
#define ICHT_CHIPREV_ADDR                   UINT8_C(0x0F)
#define ICHT_CHIPREV_SHIFT                  UINT8_C(0)
#define ICHT_CHIPREV_MASK                   UINT8_C(0xFF) // 7:0

#define ICHT_DEFAULT_SLAVE_ADDR             UINT8_C(0x60)

/** Error code definitions */
#define ICHT_NO_ERR                         INT8_C(0)
#define ICHT_NULL_PTR                       INT8_C(-1)
#define ICHT_I2C_ERR                        INT8_C(-2)
#define ICHT_INVALID_VAR                    INT8_C(-3)

/* Set and Get register macros */
#define ICHT_REG_GET(value, fieldname) \
    (((value) & (fieldname##_MASK)) >> (fieldname##_SHIFT))
#define ICHT_REG_SET(value, fieldname, setvalue) \
    (((value) & ~(fieldname##_MASK)) | \
     (((setvalue) << (fieldname##_SHIFT)) & (fieldname##_MASK)))

#define ICHT_INVALID_BOUNDS(var, fieldname) \
    (((var) < (fieldname##_MIN)) || ((var) > (fieldname##_MAX)))

/*************************** Public definitions ******************************/

typedef int8_t (*ICHT_i2c_read_fcn) (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef int8_t (*ICHT_i2c_write_fcn)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

/*******************************   Enums  ************************************/
typedef enum {
    ICHT_CHANNEL_1       = 1,
    ICHT_CHANNEL_2       = 2,
    ICHT_CHANNEL_NUM_MIN = ICHT_CHANNEL_1,
    ICHT_CHANNEL_NUM_MAX = ICHT_CHANNEL_2
} ICHT_CHANNEL_NUM;

typedef enum {
    ICHT_ADCC_SRC_DISABLED  = 0b000, /* Disabled for all 0b0xx */
    ICHT_ADCC_SRC_MDA_PLR   = 0b100, /* MDA when ADSNF1 = 0, PLR otherwise */
    ICHT_ADCC_SRC_VVB       = 0b101,
    ICHT_ADCC_SRC_VDD       = 0b110,
    ICHT_ADCC_SRC_LDK       = 0b111,
    ICHT_ADCC_SRC_MIN       = ICHT_ADCC_SRC_DISABLED,
    ICHT_ADCC_SRC_MAX       = ICHT_ADCC_SRC_LDK
} ICHT_ADCC_SRC;

typedef enum {
    ICHT_ADSNF_MDA       = 0,
    ICHT_ADSNF_PLR       = 1,
    ICHT_ADSNF_SRC_MIN   = ICHT_ADSNF_MDA,
    ICHT_ADSNF_SRC_MAX   = ICHT_ADSNF_PLR
} ICHT_ADSNF_SRC;


typedef enum {
    ICHT_RACC_CURRENT_HI = 0, /* Overcurrent threshold up to 750mA in APC mode,
                                 Threshold up to 650mA in ACC mode. */
    ICHT_RACC_CURRENT_LO = 1, /* Overcurrent threshold up to 90mA in APC mode,
                                 Threshold up to 75mA in ACC mode. */
    ICHT_RACC_RANGE_MIN  = ICHT_RACC_CURRENT_HI,
    ICHT_RACC_RANGE_MAX  = ICHT_RACC_CURRENT_LO
} ICHT_RACC_RANGE;

/** @brief Valid range for ILIMx inputs. ILIM = delta_I(LDK) * n, input = n */
typedef enum {
    ICHT_ILIM_RANGE_MIN  = 0x0A, /* When merged, min threshold = 80mA */
    ICHT_ILIM_RANGE_MAX  = 0xFF  /* When merged, max threshold = 2040mA */
} ICHT_ILIM_RANGE;

typedef enum {
    ICHT_APC_ENABLE     = 0,
    ICHT_ACC_ENABLE     = 1,
    ICHT_EACC_MODE_MIN  = ICHT_APC_ENABLE,
    ICHT_EACC_MODE_MAX  = ICHT_ACC_ENABLE
} ICHT_EACC_MODE;

/** @brief Valid range for COMPx inputs. 0 = Slowest response, 0x7 = Fastest */
typedef enum {
    ICHT_COMP_RANGE_MIN  = 0b000,
    ICHT_COMP_RANGE_MAX  = 0b111
} ICHT_COMP_RANGE;

/** @brief Valid range for REFx inputs. 0 = 0.1V, 1023 = 1.1V */
typedef enum {
    ICHT_REF_RANGE_MIN  = 0x000,
    ICHT_REF_RANGE_MAX  = 0x3FF
} ICHT_REF_RANGE;

/** @brief Valid range for RDCO inputs. 0 = no current, 0x3F = 130uA */
typedef enum {
    ICHT_RDCO_RANGE_MIN  = 0x00,
    ICHT_RDCO_RANGE_MAX  = 0x3F
} ICHT_RDCO_RANGE;

/** @brief Sets the mode of the digital memory integrity monitor interface */
typedef enum {
    ICHT_MODE_SETTING_OP = 1,       // Enables R-only operational mode
    ICHT_MODE_SETTING_CONFIG = 2,   // Enables R/W Configuration mode
    ICHT_MODE_SETTING_MIN  = ICHT_MODE_SETTING_OP,
    ICHT_MODE_SETTING_MAX  = ICHT_MODE_SETTING_CONFIG
} ICHT_MODE_SETTING;


typedef enum {
    ICHT_RLDKS_VLDK_LT_0_5V     = 0b00, /* Triggers LDKSAT when VLDK1 < 0.5V */
    ICHT_RLDKS_VLDK_LT_0_8V     = 0b01, /* Triggers LDKSAT when VLDK1 < 0.8V */
    ICHT_RLDKS_VLDK_LT_1_0V     = 0b10, /* Triggers LDKSAT when VLDK1 < 1.0V */
    ICHT_RLDKS_VLDK_LT_1_2V     = 0b11, /* Triggers LDKSAT when VLDK1 < 1.2V */
    ICHT_RLDKS_SAT_MIN  = ICHT_RLDKS_VLDK_LT_0_5V,
    ICHT_RLDKS_SAT_MAX  = ICHT_RLDKS_VLDK_LT_1_2V
} ICHT_RLDKS_SAT;

/*************************** Data structures *********************************/



struct ICHT_Status_Regs_R
{
    /** @brief RAM initialized */
    bool INITRAM;
    /** @brief Power down event at VDD */
    bool PDOVDD;
    /** @brief RAM memory validation error */
    bool MEMERR;
    /** @brief Overtemperature event */
    bool OVT;
    /** @brief Overcurrent at channel 2 */
    bool OVC2;
    /** @brief Overcurrent at channel 1 */
    bool OVC1;
    /** @brief Oscillator error (watchdog set) */
    bool OSCERR;
    /** @brief Configuration mode timeout event */
    bool CFGTIMO;
    /** @brief Channel 1 current state (on or off) */
    bool MAPC1;
    /** @brief Monitor channel 1 enabled at least once (latched) */
    bool MONC1;
    /** @brief Channel 1 LDK saturation event */
    bool LDKSAT1;
    /** @brief Channel 2 current state (on or off) */
    bool MAPC2;
    /** @brief Monitor channel 2 enabled at least once (latched) */
    bool MONC2;
    /** @brief Channel 2 LDK saturation event */
    bool LDKSAT2;
};

struct ICHT_ADC_Val_R
{
    /** @brief INPUT Channel number, either 1 or 2 */
    ICHT_CHANNEL_NUM channel;

    /** @brief OUTPUT Value of the ADC of the selected channel*/
    uint16_t ADC;                       /* ADCx */
};


struct ICHT_ADC_Config
{
    /** @brief INPUT Channel number, either 1 or 2 */
    ICHT_CHANNEL_NUM channel;
    
    /** @brief Source of the ADC for the channel 
        If the MSB is set to 0, the ADC is disabled.
        LDKx can be measure up to 8V with 8.6mV resolution
        VDD can be measured up to 8V with 8.6mV resolution
        VB can be measured up to 8V with 8.6mV resolution
        MDA can be measured up to 1V with 1.075mV resolution
        PLR can be measured up to 1V with 1.075mV resolution
    
        MDA or PLR is selected by the ADSNFx register. 
    */
    ICHT_ADCC_SRC source;               /* ADCCx */
    
    /** @brief Enables offset compensation to prevent optical power drifts */
    bool enable_offset_compensation;    /* EOCx */
    
    /** @brief If set to 1, the channel cannot be enabled by pin ECx*/
    bool disable_channel;               /* DISCx */
    
    /** @brief If set to 1, disables the Programmable Logarithmic monitor
               resistor (PLR), and an external monitor resistor must be used.
     */
    bool disable_PLR;                   /* DISPx */
    
    /** @brief Enables use of external capacitor, COMP should be set to 0b111 */
    bool enable_external_capacitor;     /* ECIEx */
    
    /** @brief If set to 0, enables automatic power control mode,
               else if set to 1, enables the automatic current mode
     */
    ICHT_EACC_MODE mode;                /* EACCx */
};

struct ICHT_Overcurrent_Threshold
{
    /** @brief INPUT Channel number, either 1 or 2 */
    ICHT_CHANNEL_NUM channel;
    
    /** @brief Sets the specific overcurrent threshold.
        The threshold Ilim = (d_I(LDK) * n), where n = 10 to 255.
        In merged mode, this formula becomes Ilim = (2*d_I(LDK) * n)
        d_I(LDK) represents the resolution set by RACC
     */
    uint8_t n;                          /* ILIMx */
};

struct ICHT_Internal_Monitor_Resistance
{
    /** @brief INPUT Channel number, either 1 or 2 */
    ICHT_CHANNEL_NUM channel;
    
    /** @brief Sets the resistance of the PLR for a given channel
        Resistor can be selected from 100ohms to 500kOhms, following
        logarithmic increments with step width of a typical 3.3%. 
        Rmd_0 is typically 100ohms, d_Rmd(%) is typically 3.3%
        PLR1 set to Rmd = Rmd_0(1 + (d_Rmd(%)/100))^(n+1), where n = 0 to 255.
     */
    uint8_t n;                          /* RMDx */
};


struct ICHT_Regulator_Config
{
    /** @brief INPUT Channel number, either 1 or 2 */
    ICHT_CHANNEL_NUM channel;
    
    /** @brief Regulator compensation, slowest (0) to fastest (7) */
    uint8_t compensation;               /* COMPx */
    
    /** @brief Configures threshold voltage for setting the LDKSATx alarm */
    ICHT_RLDKS_SAT sat_threshold;       /* RLDKx */
    
    /** @brief Regulator reference voltage ranging from 0.1V (Vref_0) to 1.1V
               Step width (d_Vref(%)) is typically 0.235%.
               Vref = Vref_0(1 + (d_Vref(%)/100))^(n+1) from n = 0 to 1023
     */
    uint16_t Vref;                      /* REFx */
};

struct ICHT_ADSNF_RACC_Config
{
    /** @brief If 0, source is MDA, else if 1, source is PLR 
        The ADCC register must also be configured to support
        this ADC source.
    */
    ICHT_ADSNF_SRC source_1;            /* ADNSF1 */
    
    /** @brief Sets the overcurrent range */
    ICHT_RACC_RANGE range_1;            /* RACC1 */
    
    /** @brief If 0, source is MDA, else if 1, source is PLR 
        The ADCC register must also be configured to support
        this ADC source.
    */
    ICHT_ADSNF_SRC source_2;            /* ADNSF2 */
    
        /** @brief Sets the overcurrent range */
    ICHT_RACC_RANGE range_2;            /* RACC2 */
};

struct ICHT_Merge_RDCO_Config
{
    /** @brief If set to 1, Powers transistor from channel 2 in
               parallel with channel 1, controlled by channel 1
     */
    bool en_merge;
    
    /** @brief 6-bit configurable current pin at DCO,
               can be used to trim output voltage of a DC/DC converter.
     */
    uint8_t DCO_current_config;         /* RDCO */
};

struct ICHT_Error_Regs
{
    /** @brief 1 Triggers an Oscillation error (watchdog timeout) */
    bool SOSCERR;
    /** @brief 1 Simulates an overcurrent event at channel 2 */
    bool SOVC2;
    /** @brief 1 Simulates an overcurrent event at channel 1 */
    bool SOVC1;
    /** @brief 1 Simulates an overtemperature event, setting OVT */
    bool SOVT;
    /** @brief If 0 LDKSAT2 event will signal at NCHK, else not signaled */
    bool MLDKSAT2;
    /** @brief If 0 LDKSAT1 event will signal at NCHK, else not signaled */
    bool MLDKSAT1;
    /** @brief If 0 MONC1 and MONC2 event will signal at NCHK, else not signaled */
    bool MMONC;
    /** @brief If 0, Oscillator error signaled at NCHK, else not signaled */
    bool MOSCERR;
};

/** Struct that contains full list of iCHT registers */
struct ICHT_reg_list
{
    struct ICHT_Status_Regs_R STATUS;
    /** @brief Chip Temperature */
    uint8_t TEMP_R;
    struct ICHT_ADC_Val_R ADC1;
    struct ICHT_ADC_Val_R ADC2;
    /** @brief Chip revision mark */
    uint8_t CHIP_REV_R;
    struct ICHT_ADC_Config ADCCONFIG1;
    struct ICHT_Overcurrent_Threshold ILIM1;
    struct ICHT_Internal_Monitor_Resistance RMD1;
    struct ICHT_Regulator_Config REGCONFIG1;
    struct ICHT_ADC_Config ADCCONFIG2;
    struct ICHT_Overcurrent_Threshold ILIM2;
    struct ICHT_Internal_Monitor_Resistance RMD2;
    struct ICHT_Regulator_Config REGCONFIG2;
    struct ICHT_ADSNF_RACC_Config ADSNFRACC;
    struct ICHT_Merge_RDCO_Config MERGERDCO;
    /** @brief Sets the memory in either R-only or R/W mode. 
           Configuration mode allows setting address 0x10-x1F
           without changing present configured operational state.
           
           Time in configuration mode must be less than 40ms
           to avoid timeout.
            
           Switching to operational mode sets the configuration
           and changes the registers to read-only mode.
    */
    ICHT_MODE_SETTING mode;
    struct ICHT_Error_Regs ERROR;
};

struct ICHT_config
{
    /** @brief Device ID */
    uint8_t device_number;

    /** @brief Generic read function pointer */
    ICHT_i2c_read_fcn read;

    /** @brief Generic write function pointer */
    ICHT_i2c_write_fcn write;
    
    /** @brief Current state of the sensor, for W/R */
    struct ICHT_reg_list regs;
};

/** Function defines */
void ICHT_init_structs(struct ICHT_config *conf);
int8_t ICHT_init_test(struct ICHT_config *conf);
int8_t ICHT_get_status_regs(const struct ICHT_config *conf, 
                            struct ICHT_Status_Regs_R *regs);
int8_t ICHT_get_temp(const struct ICHT_config *conf, uint8_t *temp);
int8_t ICHT_get_ADC(const struct ICHT_config *conf,
                    struct ICHT_ADC_Val_R *regs);
int8_t ICHT_get_chip_rev(const struct ICHT_config *conf, uint8_t *rev);
int8_t ICHT_get_ADC_Config(const struct ICHT_config *conf,
                           struct ICHT_ADC_Config *regs);
int8_t ICHT_set_ADC_Config(const struct ICHT_config *conf,
                           struct ICHT_ADC_Config *regs);
int8_t ICHT_get_overcurrent_thresh(const struct ICHT_config *conf,
                                   struct ICHT_Overcurrent_Threshold *regs);
int8_t ICHT_set_overcurrent_thresh(const struct ICHT_config *conf,
                                   struct ICHT_Overcurrent_Threshold *regs);
int8_t ICHT_get_monitor_resistance(const struct ICHT_config *conf,
                                   struct ICHT_Internal_Monitor_Resistance *regs);
int8_t ICHT_set_monitor_resistance(const struct ICHT_config *conf,
                                   struct ICHT_Internal_Monitor_Resistance *regs);
int8_t ICHT_get_Regulator_Config(const struct ICHT_config *conf,
                                 struct ICHT_Regulator_Config *regs);
int8_t ICHT_set_Regulator_Config(const struct ICHT_config *conf,
                                 struct ICHT_Regulator_Config *regs);
int8_t ICHT_get_ADSNF_RACC_Config(const struct ICHT_config *conf,
                                  struct ICHT_ADSNF_RACC_Config *regs);
int8_t ICHT_set_ADSNF_RACC_Config(const struct ICHT_config *conf,
                                  struct ICHT_ADSNF_RACC_Config *regs);
int8_t ICHT_get_Merge_RDCO_Config(const struct ICHT_config *conf,
                                  struct ICHT_Merge_RDCO_Config *regs);
int8_t ICHT_set_Merge_RDCO_Config(const struct ICHT_config *conf,
                                  struct ICHT_Merge_RDCO_Config *regs);
int8_t ICHT_get_mode(const struct ICHT_config *conf,
                     ICHT_MODE_SETTING *mode);
int8_t ICHT_set_mode(const struct ICHT_config *conf,
                     ICHT_MODE_SETTING *mode);
int8_t ICHT_get_error_regs(const struct ICHT_config *conf, 
                            struct ICHT_Error_Regs *regs);
int8_t ICHT_set_error_regs(const struct ICHT_config *conf, 
                            struct ICHT_Error_Regs *regs);

int8_t ICHT_write_all_regs(struct ICHT_config *conf, struct ICHT_reg_list *reg_list);
int8_t ICHT_read_all_regs(struct ICHT_config *conf, struct ICHT_reg_list *reg_list);
int8_t ICHT_configure_driver(struct ICHT_config *conf, struct ICHT_reg_list *reg_list);


#endif /* ICHT_DEFINES_H_ */
