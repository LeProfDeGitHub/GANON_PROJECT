// RFM96.h
//
// Definitions for HopeRF RFM96 LoRa radios
//
// Portions adapted from RadioHead RH_RF95.h
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2014 Mike McCauley
// $Id: RH_RF95.h,v 1.22 2019/07/14 00:18:48 mikem Exp $
// 


#ifndef RFM96_h
#define RFM96_h

#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Public define -------------------------------------------------------------*/

// Max number of octets the LORA Rx/Tx FIFO can hold
#define RFM96_FIFO_SIZE 255

// The crystal oscillator frequency of the module
#define RFM96_FXOSC 32000000.0

// The Frequency Synthesizer step = RFM96_FXOSC / 2^^19
#define RFM96_FSTEP  (RFM96_FXOSC / (1 << 19))


// Register Write Not Read mas
#define RFM96_REG_WRITE_MASK                        0x80
#define RFM96_REG_READ_MASK                         0x00


// Register names (LoRa Mode, from table 85)
#define RFM96_REG_00_FIFO                           0x00
#define RFM96_REG_01_OP_MODE                        0x01
#define RFM96_REG_02_RESERVED                       0x02
#define RFM96_REG_03_RESERVED                       0x03
#define RFM96_REG_04_RESERVED                       0x04
#define RFM96_REG_05_RESERVED                       0x05
#define RFM96_REG_06_FRF_MSB                        0x06
#define RFM96_REG_07_FRF_MID                        0x07
#define RFM96_REG_08_FRF_LSB                        0x08
#define RFM96_REG_09_PA_CONFIG                      0x09
#define RFM96_REG_0A_PA_RAMP                        0x0a
#define RFM96_REG_0B_OCP                            0x0b
#define RFM96_REG_0C_LNA                            0x0c
#define RFM96_REG_0D_FIFO_ADDR_PTR                  0x0d
#define RFM96_REG_0E_FIFO_TX_BASE_ADDR              0x0e
#define RFM96_REG_0F_FIFO_RX_BASE_ADDR              0x0f
#define RFM96_REG_10_FIFO_RX_CURRENT_ADDR           0x10
#define RFM96_REG_11_IRQ_FLAGS_MASK                 0x11
#define RFM96_REG_12_IRQ_FLAGS                      0x12
#define RFM96_REG_13_RX_NB_BYTES                    0x13
#define RFM96_REG_14_RX_HEADER_CNT_VALUE_MSB        0x14
#define RFM96_REG_15_RX_HEADER_CNT_VALUE_LSB        0x15
#define RFM96_REG_16_RX_PACKET_CNT_VALUE_MSB        0x16
#define RFM96_REG_17_RX_PACKET_CNT_VALUE_LSB		0x17
#define RFM96_REG_18_MODEM_STAT                     0x18
#define RFM96_REG_19_PKT_SNR_VALUE                  0x19
#define RFM96_REG_1A_PKT_RSSI_VALUE                 0x1a
#define RFM96_REG_1B_RSSI_VALUE                     0x1b
#define RFM96_REG_1C_HOP_CHANNEL                    0x1c
#define RFM96_REG_1D_MODEM_CONFIG1                  0x1d
#define RFM96_REG_1E_MODEM_CONFIG2                  0x1e
#define RFM96_REG_1F_SYMB_TIMEOUT_LSB               0x1f
#define RFM96_REG_20_PREAMBLE_MSB                   0x20
#define RFM96_REG_21_PREAMBLE_LSB                   0x21
#define RFM96_REG_22_PAYLOAD_LENGTH                 0x22
#define RFM96_REG_23_MAX_PAYLOAD_LENGTH             0x23
#define RFM96_REG_24_HOP_PERIOD                     0x24
#define RFM96_REG_25_FIFO_RX_BYTE_ADDR              0x25
#define RFM96_REG_26_MODEM_CONFIG3                  0x26

#define RFM96_REG_27_PPM_CORRECTION                 0x27
#define RFM96_REG_28_FEI_MSB                        0x28
#define RFM96_REG_29_FEI_MID                        0x29
#define RFM96_REG_2A_FEI_LSB                        0x2a
#define RFM96_REG_2C_RSSI_WIDEBAND                  0x2c
#define RFM96_REG_31_DETECT_OPTIMIZ                 0x31
#define RFM96_REG_33_INVERT_IQ                      0x33
#define RFM96_REG_37_DETECTION_THRESHOLD            0x37
#define RFM96_REG_39_SYNC_WORD                      0x39

#define RFM96_REG_40_DIO_MAPPING1                   0x40
#define RFM96_REG_41_DIO_MAPPING2                   0x41
#define RFM96_REG_42_VERSION                        0x42

#define RFM96_REG_4B_TCXO           				0x4b
#define RFM96_REG_4D_PA_DAC         				0x4d
#define RFM96_REG_5B_FORMER_TEMP    				0x5b
#define RFM96_REG_61_AGC_REF        				0x61
#define RFM96_REG_62_AGC_THRESH1    				0x62
#define RFM96_REG_63_AGC_THRESH2    				0x63
#define RFM96_REG_64_AGC_THRESH3    				0x64

// RFM96_REG_01_OP_MODE                             0x01
#define RFM96_LONG_RANGE_MODE                       0x80
#define RFM96_ACCESS_SHARED_REG                     0x40
#define RFM96_LOW_FREQUENCY_MODE                    0x08
#define RFM96_MODE                                  0x07
#define RFM96_MODE_SLEEP                            0x00
#define RFM96_MODE_STDBY                            0x01
#define RFM96_MODE_FSTX                             0x02
#define RFM96_MODE_TX                               0x03
#define RFM96_MODE_FSRX                             0x04
#define RFM96_MODE_RXCONTINUOUS                     0x05
#define RFM96_MODE_RXSINGLE                         0x06
#define RFM96_MODE_CAD                              0x07

// RFM96_REG_09_PA_CONFIG                           0x09
#define RFM96_PA_SELECT                             0x80
#define RFM96_MAX_POWER                             0x70
#define RFM96_OUTPUT_POWER                          0x0f

// RFM96_REG_0A_PA_RAMP                             0x0a
#define RFM96_LOW_PN_TX_PLL_OFF                     0x10
#define RFM96_PA_RAMP                               0x0f
#define RFM96_PA_RAMP_3_4MS                         0x00
#define RFM96_PA_RAMP_2MS                           0x01
#define RFM96_PA_RAMP_1MS                           0x02
#define RFM96_PA_RAMP_500US                         0x03
#define RFM96_PA_RAMP_250US                         0x0
#define RFM96_PA_RAMP_125US                         0x05
#define RFM96_PA_RAMP_100US                         0x06
#define RFM96_PA_RAMP_62US                          0x07
#define RFM96_PA_RAMP_50US                          0x08
#define RFM96_PA_RAMP_40US                          0x09
#define RFM96_PA_RAMP_31US                          0x0a
#define RFM96_PA_RAMP_25US                          0x0b
#define RFM96_PA_RAMP_20US                          0x0c
#define RFM96_PA_RAMP_15US                          0x0d
#define RFM96_PA_RAMP_12US                          0x0e
#define RFM96_PA_RAMP_10US                          0x0f

// RFM96_REG_0B_OCP                                 0x0b
#define RFM96_OCP_ON                                0x20
#define RFM96_OCP_TRIM                              0x1f

// RFM96_REG_0C_LNA                                 0x0c
#define RFM96_LNA_GAIN                              0xe0
#define RFM96_LNA_GAIN_G1                           0x20
#define RFM96_LNA_GAIN_G2                           0x40
#define RFM96_LNA_GAIN_G3                           0x60
#define RFM96_LNA_GAIN_G4                           0x80
#define RFM96_LNA_GAIN_G5                           0xa0
#define RFM96_LNA_GAIN_G6                           0xc0
#define RFM96_LNA_BOOST_LF                          0x18
#define RFM96_LNA_BOOST_LF_DEFAULT                  0x00
#define RFM96_LNA_BOOST_HF                          0x03
#define RFM96_LNA_BOOST_HF_DEFAULT                  0x00
#define RFM96_LNA_BOOST_HF_150PC                    0x11

// RFM96_REG_11_IRQ_FLAGS_MASK                      0x11
#define RFM96_RX_TIMEOUT_MASK                       0x80
#define RFM96_RX_DONE_MASK                          0x40
#define RFM96_PAYLOAD_CRC_ERROR_MASK                0x20
#define RFM96_VALID_HEADER_MASK                     0x10
#define RFM96_TX_DONE_MASK                          0x08
#define RFM96_CAD_DONE_MASK                         0x04
#define RFM96_FHSS_CHANGE_CHANNEL_MASK              0x02
#define RFM96_CAD_DETECTED_MASK                     0x01

// RFM96_REG_12_IRQ_FLAGS                           0x12
#define RFM96_RX_TIMEOUT                            0x80
#define RFM96_RX_DONE                               0x40
#define RFM96_PAYLOAD_CRC_ERROR                     0x20
#define RFM96_VALID_HEADER                          0x10
#define RFM96_TX_DONE                               0x08
#define RFM96_CAD_DONE                              0x04
#define RFM96_FHSS_CHANGE_CHANNEL                   0x02
#define RFM96_CAD_DETECTED                          0x01

// RFM96_REG_18_MODEM_STAT                          0x18
#define RFM96_RX_CODING_RATE                        0xe0
#define RFM96_MODEM_STATUS_CLEAR                    0x10
#define RFM96_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define RFM96_MODEM_STATUS_RX_ONGOING               0x04
#define RFM96_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define RFM96_MODEM_STATUS_SIGNAL_DETECTED          0x01

// RFM96_REG_1C_HOP_CHANNEL                         0x1c
#define RFM96_PLL_TIMEOUT                           0x80
#define RFM96_RX_PAYLOAD_CRC_IS_ON                  0x40
#define RFM96_FHSS_PRESENT_CHANNEL                  0x3f

// RFM96_REG_1D_MODEM_CONFIG1                       0x1d
#define RFM96_BW                                    0xf0

#define RFM96_BW_7_8KHZ                             0x00
#define RFM96_BW_10_4KHZ                            0x10
#define RFM96_BW_15_6KHZ                            0x20
#define RFM96_BW_20_8KHZ                            0x30
#define RFM96_BW_31_25KHZ                           0x40
#define RFM96_BW_41_7KHZ                            0x50
#define RFM96_BW_62_5KHZ                            0x60
#define RFM96_BW_125KHZ                             0x70
#define RFM96_BW_250KHZ                             0x80
#define RFM96_BW_500KHZ                             0x90
#define RFM96_CODING_RATE                           0x0e
#define RFM96_CODING_RATE_4_5                       0x02
#define RFM96_CODING_RATE_4_6                       0x04
#define RFM96_CODING_RATE_4_7                       0x06
#define RFM96_CODING_RATE_4_8                       0x08
#define RFM96_IMPLICIT_HEADER_MODE_ON               0x01
#define RFM96_IMPLICIT_HEADER_MODE_OFF              0x00

// RFM96_REG_1E_MODEM_CONFIG2                       0x1e
#define RFM96_SPREADING_FACTOR                      0xf0
#define RFM96_SPREADING_FACTOR_64CPS                0x60
#define RFM96_SPREADING_FACTOR_128CPS               0x70
#define RFM96_SPREADING_FACTOR_256CPS               0x80
#define RFM96_SPREADING_FACTOR_512CPS               0x90
#define RFM96_SPREADING_FACTOR_1024CPS              0xa0
#define RFM96_SPREADING_FACTOR_2048CPS              0xb0
#define RFM96_SPREADING_FACTOR_4096CPS              0xc0
#define RFM96_TX_CONTINUOUS_MODE                    0x08

#define RFM96_PAYLOAD_CRC_ON                        0x04
#define RFM96_PAYLOAD_CRC_OFF                       0x00
#define RFM96_SYM_TIMEOUT_MSB                       0x03

// RFM96_REG_26_MODEM_CONFIG3
#define RFM96_MOBILE_NODE                           0x08 // HopeRF term
#define RFM96_LOW_DATA_RATE_OPTIMIZE                0x08 // Semtechs term
#define RFM96_AGC_AUTO_ON                           0x04

// RFR96_REG_42_VERSION                             0x42
#define RFM96_VERSION                               0x12 // 0x11 is also found

// RFM96_REG_4B_TCXO                                0x4b
#define RFM96_TCXO_TCXO_INPUT_ON                    0x10

// RFM96_REG_4D_PA_DAC                              0x4d
#define RFM96_PA_DAC_DISABLE                        0x04
#define RFM96_PA_DAC_ENABLE                         0x07

/* Public function prototypes ------------------------------------------------*/

typedef struct {
	SPI_HandleTypeDef    *spiHandle;		// SPI Handle
	GPIO_TypeDef 	     *csPinBank;		// Chip Select Pin Bank
	uint16_t 		      csPin;			// Chip Select Pin
	GPIO_TypeDef 	     *resetPinBank;		// Reset Pin Bank
	uint16_t 		      resetPin;			// Reset Pin
} RFM96_Chip;


void	RFM96_Init(RFM96_Chip        *rfm96_chip,
			       SPI_HandleTypeDef *spiHandle,
			       GPIO_TypeDef      *csPinBank,
			       uint16_t           csPin,
				   GPIO_TypeDef      *resetPinBank,
				   uint16_t           resetPin,
				   double			   frequency);
void	RFM96_Reset(RFM96_Chip *rfm96_chip);
uint8_t	RFM96_GetVersion(RFM96_Chip *rfm96_chip);
void	RFM96_LoRaStandBy(RFM96_Chip *rfm96_chip);
void	RFM96_LoRaSleep(RFM96_Chip *rfm96_chip);
void	RFM96_SetFrequency(RFM96_Chip *rfm96_chip, double frequency);
double	RFM96_GetFrequency(RFM96_Chip *rfm96_chip);
void 	RFM96_SetTxPower(RFM96_Chip *rfm96_chip, int8_t power);
void	RFM96_SetOCP(RFM96_Chip *rfm96_chip, uint8_t mA);
void	RFM96_BeginPacket(RFM96_Chip *rfm96_chip);
void	RFM96_EndPacket(RFM96_Chip *rfm96_chip);
void	RFM96_Write(RFM96_Chip *rfm96_chip, uint8_t *data, uint16_t len);
void	RFM96_WriteString(RFM96_Chip *rfm96_chip, char *data);
void	RFM96_Print(RFM96_Chip *rfm96_chip, char *data);
int 	RFM96_ParsePacket(RFM96_Chip *rfm96_chip);
void	RFM96_Read(RFM96_Chip *rfm96_chip, uint8_t *buff, int size);
void	RFM96_ReadRegisters(RFM96_Chip *rfm96_chip, uint8_t reg, uint8_t *data, uint16_t len);
uint8_t RFM96_ReadRegister(RFM96_Chip *rfm96_chip, uint8_t reg);
void	RFM96_WriteRegisters(RFM96_Chip *rfm96_chip, uint8_t reg, uint8_t *data, uint16_t len);
void	RFM96_WriteRegister(RFM96_Chip *rfm96_chip, uint8_t reg, uint8_t data);
void	RFM96_TransmitReceive(RFM96_Chip *rfm96_chip, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t tx_size, uint16_t rx_size);


#endif