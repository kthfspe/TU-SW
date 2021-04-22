/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void CC1200_50kbps_settings(void);
void CC1200_250kbps_settings(void);
void CC1200_500kbps_settings(void);


void CS_Select(void);
void CS_Deselect(void);
void wait_Miso(void);
void writeReg(uint16_t regAddr, uint8_t value);
void halRfWriteReg(uint16_t setting, uint8_t value);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PA_EN_Pin GPIO_PIN_4
#define PA_EN_GPIO_Port GPIOE
#define LNA_EN_Pin GPIO_PIN_5
#define LNA_EN_GPIO_Port GPIOE
#define HGM_Pin GPIO_PIN_6
#define HGM_GPIO_Port GPIOE
#define HIGH_Pin GPIO_PIN_6
#define HIGH_GPIO_Port GPIOA
#define Button_Pin GPIO_PIN_7
#define Button_GPIO_Port GPIOA
#define Button_EXTI_IRQn EXTI9_5_IRQn
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOE
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOE
#define SPI3_CS_Pin GPIO_PIN_6
#define SPI3_CS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

//COMMAND STROBES
#define SRES           0x30        //Reset chip
#define SFSTXON        0x31        //Enable and calibrate frequency synthesizer (if SETTLING_CFG.FS_AUTOCAL = 1).If in RX and PKT_CFG2.CCA_MODE ≠ 0: Go to a wait state where only the synthesizer isrunning (for quick
#define SXOFF          0x32        //Enter XOFF state when CSn
#define SCAL           0x33        //Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode withoutsetting manual calibration mode (SETTLING_CFG.FS_AUTOCAL
#define SRX            0x34        //Enable RX. Perform calibration first if coming from IDLE and SETTLING_CFG.FS_AUTOCAL
#define STX            0x35        //In IDLE state: Enable TX. Perform calibration first if SETTLING_CFG.FS_AUTOCAL = 1. If in RX state and PKT_CFG2.CCA_MODE ≠ 0: Only go to TX if channel
#define SIDLE          0x36        //Exit RX/TX, turn off frequency synthesizer and exit eWOR mode
#define SAFC           0x37        //Automatic
#define SWOR           0x38        //Start automatic RX polling sequence (eWOR) as described in Section 9.6 ifWOR_CFG0.RC_PD
#define SPWD           0x39        //Enter SLEEP mode when CSn
#define SFRX           0x3A        //Flush the RX FIFO. Only issue SFRX in IDLE or
#define SFTX           0x3B        //Flush the TX FIFO. Only issue SFTX in IDLE or
#define SWORRST        0x3C        //Reset the eWOR timer to the
#define SNOP           0x3D        //No operation. May be used to get access to the chip

//configuration registers
#define IOCFG3                   0x0000
#define IOCFG2                   0x0001
#define IOCFG1                   0x0002
#define IOCFG0                   0x0003
#define SYNC3                    0x0004
#define SYNC2                    0x0005
#define SYNC1                    0x0006
#define SYNC0                    0x0007
#define SYNC_CFG1                0x0008
#define SYNC_CFG0                0x0009
#define DEVIATION_M              0x000A
#define MODCFG_DEV_E             0x000B
#define DCFILT_CFG               0x000C
#define PREAMBLE_CFG1            0x000D
#define PREAMBLE_CFG0            0x000E
#define IQIC                     0x000F
#define CHAN_BW                  0x0010
#define MDMCFG1                  0x0011
#define MDMCFG0                  0x0012
#define SYMBOL_RATE2             0x0013
#define SYMBOL_RATE1             0x0014
#define SYMBOL_RATE0             0x0015
#define AGC_REF                  0x0016
#define AGC_CS_THR               0x0017
#define AGC_GAIN_ADJUST          0x0018
#define AGC_CFG3                 0x0019
#define AGC_CFG2                 0x001A
#define AGC_CFG1                 0x001B
#define AGC_CFG0                 0x001C
#define FIFO_CFG                 0x001D
#define DEV_ADDR                 0x001E
#define SETTLING_CFG             0x001F
#define FS_CFG                   0x0020
#define WOR_CFG1                 0x0021
#define WOR_CFG0                 0x0022
#define WOR_EVENT0_MSB           0x0023
#define WOR_EVENT0_LSB           0x0024
#define RXDCM_TIME               0x0025
#define PKT_CFG2                 0x0026
#define PKT_CFG1                 0x0027
#define PKT_CFG0                 0x0028
#define RFEND_CFG1               0x0029
#define RFEND_CFG0               0x002A
#define PA_CFG1                  0x002B
#define PA_CFG0                  0x002C
#define ASK_CFG                  0x002D
#define PKT_LEN                  0x002E

// Extended Configuration Registers
#define IF_MIX_CFG               0x2F00
#define FREQOFF_CFG              0x2F01
#define TOC_CFG                  0x2F02
#define MARC_SPARE               0x2F03
#define ECG_CFG                  0x2F04
#define MDMCFG2                  0x2F05
#define EXT_CTRL                 0x2F06
#define RCCAL_FINE               0x2F07
#define RCCAL_COARSE             0x2F08
#define RCCAL_OFFSET             0x2F09
#define FREQOFF1                 0x2F0A
#define FREQOFF0                 0x2F0B
#define FREQ2                    0x2F0C
#define FREQ1                    0x2F0D
#define FREQ0                    0x2F0E
#define IF_ADC2                  0x2F0F
#define IF_ADC1                  0x2F10
#define IF_ADC0                  0x2F11
#define FS_DIG1                  0x2F12
#define FS_DIG0                  0x2F13
#define FS_CAL3                  0x2F14
#define FS_CAL2                  0x2F15
#define FS_CAL1                  0x2F16
#define FS_CAL0                  0x2F17
#define FS_CHP                   0x2F18
#define FS_DIVTWO                0x2F19
#define FS_DSM1                  0x2F1A
#define FS_DSM0                  0x2F1B
#define FS_DVC1                  0x2F1C
#define FS_DVC0                  0x2F1D
#define FS_LBI                   0x2F1E
#define FS_PFD                   0x2F1F
#define FS_PRE                   0x2F20
#define FS_REG_DIV_CML           0x2F21
#define FS_SPARE                 0x2F22
#define FS_VCO4                  0x2F23
#define FS_VCO3                  0x2F24
#define FS_VCO2                  0x2F25
#define FS_VCO1                  0x2F26
#define FS_VCO0                  0x2F27
#define GBIAS6                   0x2F28
#define GBIAS5                   0x2F29
#define GBIAS4                   0x2F2A
#define GBIAS3                   0x2F2B
#define GBIAS2                   0x2F2C
#define GBIAS1                   0x2F2D
#define GBIAS0                   0x2F2E
#define IFAMP                    0x2F2F
#define LNA                      0x2F30
#define RXMIX                    0x2F31
#define XOSC5                    0x2F32
#define XOSC4                    0x2F33
#define XOSC3                    0x2F34
#define XOSC2                    0x2F35
#define XOSC1                    0x2F36
#define XOSC0                    0x2F37
#define ANALOG_SPARE             0x2F38
#define PA_CFG3                  0x2F39
#define IRQ0M                    0x2F3F
#define IRQ0F                    0x2F40

// Status Registers
#define WOR_TIME1                0x2F64
#define WOR_TIME0                0x2F65
#define WOR_CAPTURE1             0x2F66
#define WOR_CAPTURE0             0x2F67
#define BIST                     0x2F68
#define DCFILTOFFSET_I1          0x2F69
#define DCFILTOFFSET_I0          0x2F6A
#define DCFILTOFFSET_Q1          0x2F6B
#define DCFILTOFFSET_Q0          0x2F6C
#define IQIE_I1                  0x2F6D
#define IQIE_I0                  0x2F6E
#define IQIE_Q1                  0x2F6F
#define IQIE_Q0                  0x2F70
#define RSSI1                    0x2F71
#define RSSI0                    0x2F72
#define MARCSTATE                0x2F73
#define LQI_VAL                  0x2F74
#define PQT_SYNC_ERR             0x2F75
#define DEM_STATUS               0x2F76
#define FREQOFF_EST1             0x2F77
#define FREQOFF_EST0             0x2F78
#define AGC_GAIN3                0x2F79
#define AGC_GAIN2                0x2F7A
#define AGC_GAIN1                0x2F7B
#define AGC_GAIN0                0x2F7C
#define CFM_RX_DATA_OUT          0x2F7D
#define CFM_TX_DATA_IN           0x2F7E
#define ASK_SOFT_RX_DATA         0x2F7F
#define RNDGEN                   0x2F80
#define MAGN2                    0x2F81
#define MAGN1                    0x2F82
#define MAGN0                    0x2F83
#define ANG1                     0x2F84
#define ANG0                     0x2F85
#define CHFILT_I2                0x2F86
#define CHFILT_I1                0x2F87
#define CHFILT_I0                0x2F88
#define CHFILT_Q2                0x2F89
#define CHFILT_Q1                0x2F8A
#define CHFILT_Q0                0x2F8B
#define GPIO_STATUS              0x2F8C
#define FSCAL_CTRL               0x2F8D
#define PHASE_ADJUST             0x2F8E
#define PARTNUMBER               0x2F8F
#define PARTVERSION              0x2F90
#define SERIAL_STATUS            0x2F91
#define MODEM_STATUS1            0x2F92
#define MODEM_STATUS0            0x2F93
#define MARC_STATUS1             0x2F94
#define MARC_STATUS0             0x2F95
#define PA_IFAMP_TEST            0x2F96
#define FSRF_TEST                0x2F97
#define PRE_TEST                 0x2F98
#define PRE_OVR                  0x2F99
#define ADC_TEST                 0x2F9A
#define DVC_TEST                 0x2F9B
#define ATEST                    0x2F9C
#define ATEST_LVDS               0x2F9D
#define ATEST_MODE               0x2F9E
#define XOSC_TEST1               0x2F9F
#define XOSC_TEST0               0x2FA0
#define AES                      0x2FA1
#define MDM_TEST                 0x2FA2

#define RXFIRST                  0x2FD2
#define TXFIRST                  0x2FD3
#define RXLAST                   0x2FD4
#define TXLAST                   0x2FD5
#define NUM_TXBYTES              0x2FD6  /* Number of bytes in TXFIFO */
#define NUM_RXBYTES              0x2FD7  /* Number of bytes in RXFIFO */
#define FIFO_NUM_TXBYTES         0x2FD8
#define FIFO_NUM_RXBYTES         0x2FD9
#define RXFIFO_PRE_BUF           0x2FDA

// DATA FIFO Access
#define SINGLE_TXFIFO            0x003F     /*  TXFIFO  - Single accecss to Transmit FIFO */
#define BURST_TXFIFO             0x007F     /*  TXFIFO  - Burst accecss to Transmit FIFO  */
#define SINGLE_RXFIFO            0x00BF     /*  RXFIFO  - Single accecss to Receive FIFO  */
#define BURST_RXFIFO             0x00FF     /*  RXFIFO  - Busrrst ccecss to Receive FIFO  */

// Other Register Access
#define RADIO_BURST_ACCESS   	0x40
#define RADIO_SINGLE_ACCESS  	0x00
#define RADIO_READ_ACCESS    	0x80
#define RADIO_WRITE_ACCESS   	0x00
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
