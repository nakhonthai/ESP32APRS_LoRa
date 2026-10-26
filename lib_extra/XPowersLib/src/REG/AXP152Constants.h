#pragma once

#define AXP152_SLAVE_ADDRESS                            (0x30)

#define XPOWERS_AXP152_CHIP_ID                          (0x05)

#define XPOWERS_AXP152_STATUS1                          (0x00)
#define XPOWERS_AXP152_STATUS2                          (0x01)
#define XPOWERS_AXP152_IC_TYPE                          (0x03)

#define XPOWERS_AXP152_COMMON_CONFIG                    (0x10)
#define XPOWERS_AXP152_BATFET_CTRL                      (0x12)
#define XPOWERS_AXP152_DIE_TEMP_CTRL                    (0x13)
#define XPOWERS_AXP152_MIN_SYS_VOL_CTRL                 (0x14)
#define XPOWERS_AXP152_INPUT_VOL_LIMIT_CTRL             (0x15)
#define XPOWERS_AXP152_INPUT_CUR_LIMIT_CTRL             (0x16)
#define XPOWERS_AXP152_RESET_FUEL_GAUGE                 (0x17)
#define XPOWERS_AXP152_CHARGE_GAUGE_WDT_CTRL            (0x18)


#define XPOWERS_AXP152_WDT_CTRL                         (0x19)
#define XPOWERS_AXP152_LOW_BAT_WARN_SET                 (0x1A)


#define XPOWERS_AXP152_PWRON_STATUS                     (0x20)
#define XPOWERS_AXP152_PWROFF_STATUS                    (0x21)
#define XPOWERS_AXP152_PWROFF_EN                        (0x22)
#define XPOWERS_AXP152_DC_OVP_UVP_CTRL                  (0x23)
// DCDC2 output voltage control (0.5~1.2V @10mV/step, 1.22~1.54V @20mV/step)
#define XPOWERS_AXP152_DC_VOL2_CTRL                     (0x23) //DC-DC2
// DCDC2 voltage slew rate control
#define XPOWERS_AXP152_DCDC2_V_RAMP                     (0x25)
// DCDC1 output voltage control (1.7~3.5V @100mV/step)
#define XPOWERS_AXP152_DC_VOL1_CTRL                     (0x26) //DC-DC1
// DCDC3 output voltage control (0.7~3.5V @25mV/step)
#define XPOWERS_AXP152_DC_VOL3_CTRL                     (0x27) //DC-DC3
// ALDO1/ALDO2 output voltage control (0.5~3.5V @100mV/step)
#define XPOWERS_AXP152_LDO_VOL0_CTRL                    (0x28) //ALDO1 (high nibble) / ALDO2 (low nibble)
// DLDO1 output voltage control (0.7~3.5V @100mV/step)
#define XPOWERS_AXP152_LDO_VOL7_CTRL                    (0x29) //DLDO1
// DLDO2 output voltage control (0.7~3.5V @100mV/step)
#define XPOWERS_AXP152_LDO_VOL8_CTRL                    (0x2A) //DLDO2
// DCDC4 output voltage control (0.7~3.5V @25mV/step)
#define XPOWERS_AXP152_DC_VOL4_CTRL                     (0x2B) //DC-DC4

// Fast power-on start sequence registers (match AXP2101 map)
#define XPOWERS_AXP152_FAST_PWRON_SET0                  (0x28) // DCDC1~DCDC4 start sequence
#define XPOWERS_AXP152_FAST_PWRON_SET1                  (0x29) // DCDC5 / ALDO1~ALDO3 start sequence
#define XPOWERS_AXP152_FAST_PWRON_SET2                  (0x2A) // ALDO4 / BLDO1~BLDO2 / CPULDO start sequence
#define XPOWERS_AXP152_FAST_PWRON_CTRL                  (0x2B) // DLDO1~DLDO2 start sequence / enable

// Register 0x31: VOFF - minimum system voltage before PMU shutdown
#define XPOWERS_AXP152_VOFF_SET                         (0x31)
// Register 0x32: OFF control (PWROK sequencing / shutdown control)
#define XPOWERS_AXP152_OFF_CTRL                         (0x32)
// Register 0x37: DCDC switching frequency control
#define XPOWERS_AXP152_DCDC_FREQ                        (0x37)
// Register 0x80: DCDC forced-PWM / power-down mode control
#define XPOWERS_AXP152_DCDC_MODE                        (0x80)

// Register 0x36 PEK (power-enable key) level control
#define XPOWERS_AXP152_IRQ_OFF_ON_LEVEL_CTRL            (0x36)

#define XPOWERS_AXP152_ADC_CHANNEL_CTRL                 (0x30)
#define XPOWERS_AXP152_ADC_DATA_RELUST0                 (0x34)
#define XPOWERS_AXP152_ADC_DATA_RELUST1                 (0x35)
#define XPOWERS_AXP152_ADC_DATA_RELUST2                 (0x36)
#define XPOWERS_AXP152_ADC_DATA_RELUST3                 (0x37)
#define XPOWERS_AXP152_ADC_DATA_RELUST4                 (0x38)
#define XPOWERS_AXP152_ADC_DATA_RELUST5                 (0x39)
#define XPOWERS_AXP152_ADC_DATA_RELUST6                 (0x3A)
#define XPOWERS_AXP152_ADC_DATA_RELUST7                 (0x3B)
#define XPOWERS_AXP152_ADC_DATA_RELUST8                 (0x3C)
#define XPOWERS_AXP152_ADC_DATA_RELUST9                 (0x3D)


//XPOWERS INTERRUPT REGISTER
#define XPOWERS_AXP152_INTEN1                           (0x40)
#define XPOWERS_AXP152_INTEN2                           (0x41)
#define XPOWERS_AXP152_INTEN3                           (0x42)


//XPOWERS INTERRUPT STATUS REGISTER
#define XPOWERS_AXP152_INTSTS1                          (0x48)
#define XPOWERS_AXP152_INTSTS2                          (0x49)
#define XPOWERS_AXP152_INTSTS3                          (0x4A)
#define XPOWERS_AXP152_INTSTS_CNT                       (3)

#define XPOWERS_AXP152_TS_PIN_CTRL                      (0x50)
#define XPOWERS_AXP152_TS_HYSL2H_SET                    (0x52)
#define XPOWERS_AXP152_TS_LYSL2H_SET                    (0x53)


#define XPOWERS_AXP152_VLTF_CHG_SET                     (0x54)
#define XPOWERS_AXP152_VHLTF_CHG_SET                    (0x55)
#define XPOWERS_AXP152_VLTF_WORK_SET                    (0x56)
#define XPOWERS_AXP152_VHLTF_WORK_SET                   (0x57)


#define XPOWERS_AXP152_JIETA_EN_CTRL                    (0x58)
#define XPOWERS_AXP152_JIETA_SET0                       (0x59)
#define XPOWERS_AXP152_JIETA_SET1                       (0x5A)
#define XPOWERS_AXP152_JIETA_SET2                       (0x5B)


#define XPOWERS_AXP152_IPRECHG_SET                      (0x61)
#define XPOWERS_AXP152_ICC_CHG_SET                      (0x62)
#define XPOWERS_AXP152_ITERM_CHG_SET_CTRL               (0x63)

#define XPOWERS_AXP152_CV_CHG_VOL_SET                   (0x64)

#define XPOWERS_AXP152_THE_REGU_THRES_SET               (0x65)
#define XPOWERS_AXP152_CHG_TIMEOUT_SET_CTRL             (0x67)

#define XPOWERS_AXP152_BAT_DET_CTRL                     (0x68)
#define XPOWERS_AXP152_CHGLED_SET_CTRL                  (0x69)

#define XPOWERS_AXP152_BTN_VOL_MIN                      (2600)
#define XPOWERS_AXP152_BTN_VOL_MAX                      (3300)
#define XPOWERS_AXP152_BTN_VOL_STEPS                    (100)


#define XPOWERS_AXP152_BTN_BAT_CHG_VOL_SET              (0x6A)

// LDO0 (LDOIO0) output voltage control - also used for LDO0 enable
// Reg 0x15 : bit7 = LDO0 enable, bits[4:0] = output voltage code
#define XPOWERS_AXP152_LDO0_CTRL                        (0x15)

#define XPOWERS_AXP152_LDO0_VOL_MIN                     (1800)
#define XPOWERS_AXP152_LDO0_VOL_MAX                     (3300)
#define XPOWERS_AXP152_LDO0_VOL_STEPS                   (100u)

// GPIO/LDO voltage control registers (upstream AXP152 register map 0x90-0x9D)
#define XPOWERS_AXP152_GPIO0_CTRL                       (0x90)
#define XPOWERS_AXP152_GPIO1_CTRL                       (0x91)
#define XPOWERS_AXP152_GPIO2_CTRL                       (0x92)
#define XPOWERS_AXP152_GPIO3_CTRL                       (0x93)
#define XPOWERS_AXP152_LDOGPIO2_V_OUT                   (0x96)
#define XPOWERS_AXP152_GPIO_INPUT                       (0x97)
#define XPOWERS_AXP152_PWM0_FREQ_X                      (0x98)
#define XPOWERS_AXP152_PWM0_FREQ_Y                      (0x99)
#define XPOWERS_AXP152_PWM0_DUTY_CYCLE                  (0x9A)
#define XPOWERS_AXP152_PWM1_FREQ_X                      (0x9B)
#define XPOWERS_AXP152_PWM1_FREQ_Y                      (0x9C)
#define XPOWERS_AXP152_PWM1_DUTY_CYCLE                  (0x9D)

// Board-specific voltage control registers (kept for API compatibility with .tpp)
#define XPOWERS_AXP152_LDO_VOL2_CTRL                    (0x94) //ALDO3
#define XPOWERS_AXP152_LDO_VOL3_CTRL                    (0x95) //ALDO4
#define XPOWERS_AXP152_LDO_VOL4_CTRL                    (0x96) //BLDO1
#define XPOWERS_AXP152_LDO_VOL5_CTRL                    (0x97) //BLDO2
#define XPOWERS_AXP152_LDO_VOL6_CTRL                    (0x98) //CPUSLDO

// Internal register mappings used by the module enable/disable functions
// (per datasheet chapter "Internal Registers")
#define XPOWERS_AXP152_DC_ONOFF_DVM_CTRL                (0x12)
#define XPOWERS_AXP152_DC_FORCE_PWM_CTRL                (0x80)
#define XPOWERS_AXP152_LDO_ONOFF_CTRL0                  (0x12)
#define XPOWERS_AXP152_LDO_ONOFF_CTRL1                  (0x12)

#define XPOWERS_AXP152_BAT_PARAME                       (0xA1)
#define XPOWERS_AXP152_FUEL_GAUGE_CTRL                  (0xA2)
#define XPOWERS_AXP152_BAT_PERCENT_DATA                 (0xA4)

// DCDC 1~4
#define XPOWERS_AXP152_DCDC1_VOL_MIN                    (1700)
#define XPOWERS_AXP152_DCDC1_VOL_MAX                    (3500)
#define XPOWERS_AXP152_DCDC1_VOL_STEPS                  (100u)

#define XPOWERS_AXP152_DCDC2_VOL1_MIN                  (500u)
#define XPOWERS_AXP152_DCDC2_VOL1_MAX                  (1200u)
#define XPOWERS_AXP152_DCDC2_VOL2_MIN                  (1220u)
#define XPOWERS_AXP152_DCDC2_VOL2_MAX                  (1540u)

#define XPOWERS_AXP152_DCDC2_VOL_STEPS1                 (10u)
#define XPOWERS_AXP152_DCDC2_VOL_STEPS2                 (20u)

#define XPOWERS_AXP152_DCDC2_VOL_STEPS1_BASE            (0u)
#define XPOWERS_AXP152_DCDC2_VOL_STEPS2_BASE            (71)


#define XPOWERS_AXP152_DCDC3_VOL1_MIN                  (500u)
#define XPOWERS_AXP152_DCDC3_VOL1_MAX                  (1200u)
#define XPOWERS_AXP152_DCDC3_VOL2_MIN                  (1220u)
#define XPOWERS_AXP152_DCDC3_VOL2_MAX                  (1540u)
#define XPOWERS_AXP152_DCDC3_VOL3_MIN                  (1600u)
#define XPOWERS_AXP152_DCDC3_VOL3_MAX                  (3400u)

#define XPOWERS_AXP152_DCDC3_VOL_MIN                    (700)
#define XPOWERS_AXP152_DCDC3_VOL_MAX                    (3500)
#define XPOWERS_AXP152_DCDC3_VOL_STEPS                 (50u)

#define XPOWERS_AXP152_DCDC3_VOL_STEPS1                 (10u)
#define XPOWERS_AXP152_DCDC3_VOL_STEPS2                 (20u)
#define XPOWERS_AXP152_DCDC3_VOL_STEPS3                 (100u)

#define XPOWERS_AXP152_DCDC3_VOL_STEPS1_BASE            (0u)
#define XPOWERS_AXP152_DCDC3_VOL_STEPS2_BASE            (71)
#define XPOWERS_AXP152_DCDC3_VOL_STEPS3_BASE            (88)


#define XPOWERS_AXP152_DCDC4_VOL_MIN                    (700)
#define XPOWERS_AXP152_DCDC4_VOL_MAX                    (3500)
#define XPOWERS_AXP152_DCDC4_VOL_STEPS                 (25u)

#define XPOWERS_AXP152_DCDC4_VOL1_MIN                  (700u)
#define XPOWERS_AXP152_DCDC4_VOL1_MAX                  (3500u)
#define XPOWERS_AXP152_DCDC4_VOL2_MIN                  (1220u)
#define XPOWERS_AXP152_DCDC4_VOL2_MAX                  (1840u)

#define XPOWERS_AXP152_DCDC4_VOL_STEPS1                 (10u)
#define XPOWERS_AXP152_DCDC4_VOL_STEPS2                 (20u)

#define XPOWERS_AXP152_DCDC4_VOL_STEPS1_BASE            (0u)
#define XPOWERS_AXP152_DCDC4_VOL_STEPS2_BASE            (71)



// DCDC5 (LDOIO0) – no DCDC5 on AXP152, kept only for API compatibility
#define XPOWERS_AXP152_DCDC5_VOL_1200MV                 (1200)
#define XPOWERS_AXP152_DCDC5_VOL_VAL                    (0x19)
#define XPOWERS_AXP152_DCDC5_VOL_MIN                    (1400)
#define XPOWERS_AXP152_DCDC5_VOL_MAX                    (3700)
#define XPOWERS_AXP152_DCDC5_VOL_STEPS                  (100u)

#define XPOWERS_AXP152_VSYS_VOL_THRESHOLD_MIN          (2600)
#define XPOWERS_AXP152_VSYS_VOL_THRESHOLD_MAX          (3300)
#define XPOWERS_AXP152_VSYS_VOL_THRESHOLD_STEPS        (100)

// ALDO 1~4

#define XPOWERS_AXP152_ALDO1_VOL_MIN                    (1200)
#define XPOWERS_AXP152_ALDO1_VOL_MAX                    (3300)
#define XPOWERS_AXP152_ALDO1_VOL_STEPS                  (100u)

#define XPOWERS_AXP152_ALDO2_VOL_MIN                    (1200)
#define XPOWERS_AXP152_ALDO2_VOL_MAX                    (3300)
#define XPOWERS_AXP152_ALDO2_VOL_STEPS                  (100u)


#define XPOWERS_AXP152_ALDO3_VOL_MIN                    (500)
#define XPOWERS_AXP152_ALDO3_VOL_MAX                    (3500)
#define XPOWERS_AXP152_ALDO3_VOL_STEPS                  (100u)


#define XPOWERS_AXP152_ALDO4_VOL_MIN                    (500)
#define XPOWERS_AXP152_ALDO4_VOL_MAX                    (3500)
#define XPOWERS_AXP152_ALDO4_VOL_STEPS                  (100u)

// BLDO 1~2

#define XPOWERS_AXP152_BLDO1_VOL_MIN                    (500)
#define XPOWERS_AXP152_BLDO1_VOL_MAX                    (3500)
#define XPOWERS_AXP152_BLDO1_VOL_STEPS                  (100u)

#define XPOWERS_AXP152_BLDO2_VOL_MIN                    (500)
#define XPOWERS_AXP152_BLDO2_VOL_MAX                    (3500)
#define XPOWERS_AXP152_BLDO2_VOL_STEPS                  (100u)

// CPUSLDO

#define XPOWERS_AXP152_CPUSLDO_VOL_MIN                  (500)
#define XPOWERS_AXP152_CPUSLDO_VOL_MAX                  (1400)
#define XPOWERS_AXP152_CPUSLDO_VOL_STEPS                (50)


// DLDO 1~2
#define XPOWERS_AXP152_DLDO1_VOL_MIN                  (700)
#define XPOWERS_AXP152_DLDO1_VOL_MAX                  (3500)
#define XPOWERS_AXP152_DLDO1_VOL_STEPS                (100u)

#define XPOWERS_AXP152_DLDO2_VOL_MIN                  (700)
#define XPOWERS_AXP152_DLDO2_VOL_MAX                  (3500)
#define XPOWERS_AXP152_DLDO2_VOL_STEPS                (100u)


#define XPOWERS_AXP152_CONVERSION(raw)                 (22.0 + (7274 - raw) / 20.0)

