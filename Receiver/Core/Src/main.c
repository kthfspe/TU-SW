/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
#define SINGLE_TXFIFO            0x003F     /*  TXFIFO  - Single access to Transmit FIFO */
#define BURST_TXFIFO             0x007F     /*  TXFIFO  - Burst access to Transmit FIFO  */
#define SINGLE_RXFIFO            0x00BF     /*  RXFIFO  - Single access to Receive FIFO  */
#define BURST_RXFIFO             0x00FF     /*  RXFIFO  - Burst access to Receive FIFO  */

// Other Register Access, look at page 11 of user manual
#define RADIO_BURST_ACCESS   	0x40
#define RADIO_SINGLE_ACCESS  	0x00
#define RADIO_READ_ACCESS    	0x80
#define RADIO_WRITE_ACCESS   	0x00

// Data modes
#define DATA_MODE_50kbps   	    0x1
#define DATA_MODE_250kbps       0x2
#define DATA_MODE_500kbps   	0x3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */
int dataSize = 4; //Data byte size sent
char SPI_BUFFER[8]; //8 byte buffer that stores temporary data immediately after read form transceiver
char MSG[4]; //data array with size 4 since that is the data size that is sent

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Sets CS low, selecting the transceiver
void CS_Select()  {
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);


}

// Sets CS high, deselecting the transceiver
void CS_Deselect()  {
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

}

// Waits until MISO goes low, to not interact with the transceiver before it's ready
void wait_Miso() {
	while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11)>0){
		__NOP();
	}
}

//Write to register
void writeReg(uint16_t regAddr, uint8_t value)
{
	uint8_t extended_or_not  = (uint8_t)(regAddr>>8);
	uint8_t extended_addr = (uint8_t)(regAddr & 0x00FF);
	if (extended_or_not == 0){
		CS_Select();                     									   // Select CS
		wait_Miso();                          							  	 // Wait until MISO goes low
		HAL_SPI_Transmit(&hspi3, (uint8_t*)&regAddr, 1, 100);                // Send register address
		HAL_SPI_Transmit(&hspi3, (uint8_t*)&value, 1, 100);                  // Send value
		CS_Deselect();                    	   							   // Deselect CC1101
	}
	else if (extended_or_not == 0x2F){//read page 9 of user manual, 2F is the flag byte which is followed by the address
		CS_Select();                     								   // Select CS
		wait_Miso();                          							   // Wait until MISO goes low
		HAL_SPI_Transmit(&hspi3, (uint8_t*)&extended_or_not, 1, 100);      // Access the extended registers
		HAL_SPI_Transmit(&hspi3, (uint8_t*)&extended_addr, 1, 100);        // Send register address
		HAL_SPI_Transmit(&hspi3, (uint8_t*)&value, 1, 100);				   // Send value
		CS_Deselect();                    	   							   // Deselect CC1101
	}
}

// Write to register (ONLY USED FOR SETTING THE INTIAL SETTINGS. The name is "halRfWriteReg" so it matches with SMARTRF Studio
void halRfWriteReg(uint16_t setting, uint8_t value)
{
	writeReg(setting, value);
}

// Send command strobe (FIFO_BYTES_AVAILABLE CASE 1)
void command_strobe1(uint8_t cmd){
    CS_Select();// set the CS pin to LOW
    wait_Miso();// Wait until MISO goes low
    HAL_SPI_Transmit(&hspi3, (uint8_t*)&cmd, 1, 100);// Send register address
    CS_Deselect();
  }

// Read register
void readReg(uint16_t regAddr) {
    uint8_t addr;
    uint8_t extended_or_not  = (uint8_t)(regAddr>>8);
    uint8_t extended_addr = (uint8_t)(regAddr & 0x00FF);
    // With cc1200 there are 2 types of register accesses and the following code choose the right one.
    if (extended_or_not == 0){
		addr = regAddr | RADIO_READ_ACCESS;			  					// Bitwise and to get the right address
		CS_Select();                									// set the SS pin to LOW
		wait_Miso();                          							// Wait until MISO goes low
		HAL_SPI_Transmit(&hspi3, (uint8_t*)&addr, 1, 100);              // Send register address
		HAL_SPI_Receive(&hspi3, (uint8_t*)&SPI_BUFFER, 1, 100);         // Read result
		CS_Deselect();                    							 	// set the SS pin to HIGH
	}
    else if (extended_or_not == 0x2F){
    	extended_or_not = extended_or_not | RADIO_READ_ACCESS;			  				 // Bitwise and to get the right address
		CS_Select();                									// set the SS pin to LOW
		wait_Miso();                          							// Wait until MISO goes low
		HAL_SPI_Transmit(&hspi3, (uint8_t*)&extended_or_not, 1, 100);	// Access the extended registers
		HAL_SPI_Transmit(&hspi3, (uint8_t*)&extended_addr, 1, 100);     // Send register address
		HAL_SPI_Receive(&hspi3, (uint8_t*)&SPI_BUFFER, 1, 100);         // Read result
		CS_Deselect();                    							 	// set the SS pin to HIGH

    }
}

//attempts to put the transceiver in receive mode
//while error: resets transceiver
void set_in_receive_mode(){
	command_strobe1(SRX);//Set in receive mode
	readReg(MARCSTATE); //state of transceiver, 01101101 = RXIDLE
	while(SPI_BUFFER[0]==0b10001){//bit sequence of RX_FIFO_ERROR
		command_strobe1(SRES);//reset chip
		command_strobe1(SRX);//Set in receive mode
		readReg(MARCSTATE); //state of transceiver
	}
	memset(SPI_BUFFER, 0, sizeof SPI_BUFFER);//Clear buffer since marcstate will be in SPI_BUFFER[0]
}

void process_data(){
	//TODO send on can
	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
	//clear MSG
	memset(MSG, 0, sizeof MSG);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);//Active Low so this deactivates it
  HAL_GPIO_WritePin(PA_EN_GPIO_Port, PA_EN_Pin, GPIO_PIN_RESET);//To not transmit
  HAL_GPIO_WritePin(HGM_GPIO_Port, HGM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LNA_EN_GPIO_Port, LNA_EN_Pin, GPIO_PIN_SET);//To receive
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  //Set transceiver in receive mode, while error: continue trying to set it in receive
  set_in_receive_mode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //Wait for packets(as an interrupt)
	  //if data in buffer, process it, then clear data buffer

	  if(MSG[0] != 0){//there is data in the buffer
		  process_data();
	  }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PA_EN_Pin|HGM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LNA_EN_Pin|LED_GREEN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HIGH_GPIO_Port, HIGH_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA_EN_Pin LNA_EN_Pin HGM_Pin LED_GREEN_Pin
                           LED_BLUE_Pin */
  GPIO_InitStruct.Pin = PA_EN_Pin|LNA_EN_Pin|HGM_Pin|LED_GREEN_Pin
                          |LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : HIGH_Pin */
  GPIO_InitStruct.Pin = HIGH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HIGH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_0_Pin */
  GPIO_InitStruct.Pin = GPIO_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){

	if(GPIO_PIN == GPIO_0_Pin){//Triggered by receiver flagging it has a packet
		//Extract packet from transceiver
		readReg(SINGLE_RXFIFO);
		memcpy(MSG, SPI_BUFFER, dataSize);
		memset(SPI_BUFFER, 0, sizeof SPI_BUFFER);
		//TODO Check packet for bit errors, if any discard packet, if not extract data

		//set transceiver in receive mode, then return to while loop
		set_in_receive_mode();
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
