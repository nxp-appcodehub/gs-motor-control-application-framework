/*******************************************************************************
 * Export Registers - Modified Values 
 ******************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v14.0
processor: LPC55S36
package_id: LPC55S36JBD100
mcu_data: ksdk2_0
processor_version: 14.0.2
board: LPCXpresso55S36
external_user_signals: {}
pin_labels:
- {pin_num: '11', pin_signal: PIO1_20/FC7_RTS_SCL_SSEL1/CT_INP14/FC4_TXD_SCL_MISO_WS/PWM0_A0/AOI0_OUT1/ADC1_8A, label: 'J10[15]/P1_20-ADC1_8A-PWM0_A0', identifier: PWM_A0}
- {pin_num: '4', pin_signal: PIO1_7/FC0_RTS_SCL_SSEL1/CTIMER2_MAT2/SCT_GPI4/AOI1_OUT3/ADC1_3B, label: CUR_C}
- {pin_num: '7', pin_signal: ADC1IN1B, label: VOLT_DCB}
- {pin_num: '9', pin_signal: PIO0_11/FC6_RXD_SDA_MOSI_DATA/CTIMER2_MAT2/FREQME_GPIO_CLK_A/SECURE_GPIO0_11/AOI1_OUT2/ADC1_2A, label: CUR_B}
- {pin_num: '10', pin_signal: PIO0_12/FC3_TXD_SCL_MISO_WS/FREQME_GPIO_CLK_B/SCT_GPI7/FC6_TXD_SCL_MISO_WS/SECURE_GPIO0_12/AOI1_OUT1/ADC1_3A, label: CUR_A}
- {pin_num: '19', pin_signal: PIO0_23/MCLK/CTIMER1_MAT2/CTIMER3_MAT3/SCT0_OUT4/FC0_CTS_SDA_SSEL0/SECURE_GPIO0_23/ADC0_8B, label: CUR_B, identifier: ADC_CUR_B}
- {pin_num: '20', pin_signal: PIO0_16/FC4_TXD_SCL_MISO_WS/CLKOUT/CT_INP4/SECURE_GPIO0_16/AOI0_OUT3/ADC0_3B, label: CUR_C, identifier: ADC_CUR_C}
- {pin_num: '21', pin_signal: PIO0_10/FC6_SCK/CT_INP10/CTIMER2_MAT0/FC1_TXD_SCL_MISO_WS/SCT0_OUT2/SECURE_GPIO0_10/ADC0_1A, label: VOLT_DCB, identifier: ADC_VDCB}
- {pin_num: '22', pin_signal: PIO0_15/FC6_CTS_SDA_SSEL0/UTICK_CAP2/CT_INP16/SCT0_OUT2/SECURE_GPIO0_15/ADC0_3A, label: CUR_A, identifier: ADC_CUR_A}
- {pin_num: '23', pin_signal: PIO0_31/FC0_CTS_SDA_SSEL0/CTIMER0_MAT1/SCT0_OUT3/SECURE_GPIO0_31/AOI0_OUT0/I3C0_SCL/ADC0_8A, label: P0_31-ADC0_8A, identifier: ADC0_8A}
- {pin_num: '35', pin_signal: PIO1_5/FC0_RXD_SDA_MOSI_DATA/CTIMER2_MAT0/SCT_GPI0/PWM1_A3/TRIGOUT_0/HSCMP0_IN3, label: P1_5-HSCMP0_IN3, identifier: HSCMP0_IN3}
- {pin_num: '36', pin_signal: PIO1_8/FC0_CTS_SDA_SSEL0/SCT0_OUT1/FC4_SSEL2/FC1_SCK/PWM0_A2/AOI1_OUT2/TRIGOUT_6, label: 'J8[1]/P1_8-FC0_CTS-PWM0_A2', identifier: PWM_A2}
- {pin_num: '40', pin_signal: PIO1_22/CTIMER2_MAT3/SCT_GPI5/FC4_SSEL3/CAN0_RD/QSPI_DIN3/PWM0_B1/TRIGOUT_2/HSCMP1_IN1/DAC0_OUT, label: 'JP48[3]/J10[9]/P1_22-HSCMP1_IN1-DAC0_OUT-PWM0_B1_CAN0_RD',
  identifier: PWM_B1}
- {pin_num: '50', pin_signal: PIO1_6/FC0_TXD_SCL_MISO_WS/CTIMER2_MAT1/SCT_GPI3/PWM0_A1/TRIGOUT_5/HSCMP0_OUT, label: 'J10[11]/P1_6-PWM0_A1', identifier: PWM_A1}
- {pin_num: '69', pin_signal: PIO0_24/FC0_RXD_SDA_MOSI_DATA/CT_INP8/SCT_GPI0/I3C0_SDA/TRACEDATA0/SECURE_GPIO0_24/PWM0_A1/SPI_CS0_DIS/PWM0_X0/HSCMP0_IN0, label: HSCMP0IN0,
  identifier: HSCMP0IN0}
- {pin_num: '70', pin_signal: PIO0_13/FC1_CTS_SDA_SSEL0/UTICK_CAP0/CT_INP0/SCT_GPI0/FC1_RXD_SDA_MOSI_DATA/SECURE_GPIO0_13/EXTTRIG_IN3, label: 'J10[3]/U27[1]/P0_13-FC1_SDA-SCT0_GPI0-EXTTRIG_IN3',
  identifier: ENC0_PHA}
- {pin_num: '71', pin_signal: PIO0_14/FC1_RTS_SCL_SSEL1/UTICK_CAP1/CT_INP1/SCT_GPI1/FC1_TXD_SCL_MISO_WS/SECURE_GPIO0_14/EXTTRIG_IN2, label: 'J10[1]/U27[8]/P0_14-FC1_SCL-SCT0_GPI1-EXTTRIG_IN2',
  identifier: ENC0_PHB}
- {pin_num: '75', pin_signal: PIO1_4/FC0_SCK/CTIMER2_MAT1/SCT0_OUT0/FREQME_GPIO_CLK_A/FC4_TXD_SCL_MISO_WS/SPI_DIN/PWM0_B2/TRIGOUT_7/EXTTRIG_IN8, label: 'J10[5]/P1_4-PWM0_B2',
  identifier: PWM_B2}
- {pin_num: '91', pin_signal: PIO1_17/FC6_RTS_SCL_SSEL1/SCT0_OUT4/PWM0_B0/AOI1_OUT3, label: 'J10[13]/P1_17-PWM0_B0', identifier: PWM_B0}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_Init_Pins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/

#define INIT_PINS_GPIO_DIR0_VALUE                                               0x00400000   /*!< Register name: GPIO_DIR0 */
#define INIT_PINS_GPIO_DIR1_VALUE                                               0x10000000   /*!< Register name: GPIO_DIR1 */
#define INIT_PINS_GPIO_PIN0_VALUE                                               0x00400000   /*!< Register name: GPIO_PIN0 */
#define INIT_PINS_GPIO_PIN1_VALUE                                               0x10000000   /*!< Register name: GPIO_PIN1 */
#define INIT_PINS_INPUTMUX_ADC0_TRIG0_VALUE                                     0x00000018   /*!< Register name: INPUTMUX_ADC0_TRIG0 */
#define INIT_PINS_SYSCON_AHBCLKCTRL0_VALUE                                      0x00002980   /*!< Register name: SYSCON_AHBCLKCTRL0 */
#define INIT_PINS_INPUTMUX_ENC0_PHASEA_VALUE                                    0x0000002D   /*!< Register name: INPUTMUX_ENC0_PHASEA */
#define INIT_PINS_INPUTMUX_ENC0_PHASEB_VALUE                                    0x0000002C   /*!< Register name: INPUTMUX_ENC0_PHASEB */
#define INIT_PINS_INPUTMUX_PINTSEL0_VALUE                                       0x00000011   /*!< Register name: INPUTMUX_PINTSEL0 */
#define INIT_PINS_INPUTMUX_PWM0_FAULT0_VALUE                                    0x00000015   /*!< Register name: INPUTMUX_PWM0_FAULT0 */
#define INIT_PINS_IOCON_PIO0_10_VALUE                                           0x00000000   /*!< Register name: IOCON_PIO0_10 */
#define INIT_PINS_IOCON_PIO0_13_VALUE                                           0x0000510D   /*!< Register name: IOCON_PIO0_13 */
#define INIT_PINS_IOCON_PIO0_14_VALUE                                           0x0000510D   /*!< Register name: IOCON_PIO0_14 */
#define INIT_PINS_IOCON_PIO0_15_VALUE                                           0x00000000   /*!< Register name: IOCON_PIO0_15 */
#define INIT_PINS_IOCON_PIO0_16_VALUE                                           0x00000000   /*!< Register name: IOCON_PIO0_16 */
#define INIT_PINS_IOCON_PIO0_17_VALUE                                           0x00000100   /*!< Register name: IOCON_PIO0_17 */
#define INIT_PINS_IOCON_PIO0_22_VALUE                                           0x00000100   /*!< Register name: IOCON_PIO0_22 */
#define INIT_PINS_IOCON_PIO0_23_VALUE                                           0x00000400   /*!< Register name: IOCON_PIO0_23 */
#define INIT_PINS_IOCON_PIO0_29_VALUE                                           0x00000101   /*!< Register name: IOCON_PIO0_29 */
#define INIT_PINS_IOCON_PIO0_30_VALUE                                           0x00000101   /*!< Register name: IOCON_PIO0_30 */
#define INIT_PINS_IOCON_PIO0_31_VALUE                                           0x00000400   /*!< Register name: IOCON_PIO0_31 */
#define INIT_PINS_IOCON_PIO1_17_VALUE                                           0x0000010B   /*!< Register name: IOCON_PIO1_17 */
#define INIT_PINS_IOCON_PIO1_20_VALUE                                           0x0000010B   /*!< Register name: IOCON_PIO1_20 */
#define INIT_PINS_IOCON_PIO1_22_VALUE                                           0x0000010B   /*!< Register name: IOCON_PIO1_22 */
#define INIT_PINS_IOCON_PIO1_28_VALUE                                           0x00000100   /*!< Register name: IOCON_PIO1_28 */
#define INIT_PINS_IOCON_PIO1_4_VALUE                                            0x0000010B   /*!< Register name: IOCON_PIO1_4 */
#define INIT_PINS_IOCON_PIO1_5_VALUE                                            0x00000400   /*!< Register name: IOCON_PIO1_5 */
#define INIT_PINS_IOCON_PIO1_6_VALUE                                            0x0000010B   /*!< Register name: IOCON_PIO1_6 */
#define INIT_PINS_IOCON_PIO1_8_VALUE                                            0x0000010B   /*!< Register name: IOCON_PIO1_8 */



/*******************************************************************************
 * EOF
 ******************************************************************************/
