/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



// FIFO REGISTERS
#define TX_FIFO        0x3F
#define RX_FIF0 	   0x3F

//REGISTER TYPES
#define STATUS_REGISTER  0xC0
#define CONFIG_REGISTER  0x80

//HEADERS (for read and write commands)
// No special header for write single since its header is zero
#define WRITE_BURST              0x40
#define READ_SINGLE              0x80
#define READ_BURST               0xC0

// COMMAND STROBES
#define STX                      0x35
#define SFTX					 0x3B
#define SRX               		 0x34
#define SFRX					 0x3A
#define SIDLE 					 0x36

// STATUS REGISTERS
#define PARTNUM        0x30        //PARTNUM Part number for CC1101
#define VERSION        0x31        //VERSION Current version number
#define FREQEST        0x32        //FREQEST Frequency Offset Estimate
#define LQI            0x33        //LQI Demodulator estimate for Link Quality
#define RSSI           0x34        //RSSI Received signal strength indication
#define MARCSTATE      0x35        //MARCSTATE Control state machine state
#define WORTIME1       0x36        //WORTIME1 High byte of WOR timer
#define WORTIME0       0x37        //WORTIME0 Low byte of WOR timer
#define PKTSTATUS      0x38        //PKTSTATUS Current GDOx status and packet status
#define VCO_VC_DAC     0x39        //VCO_VC_DAC Current setting from PLL calibration module
#define TXBYTES        0x3A        //TXBYTES Underflow and number of bytes in the TX FIFO
#define RXBYTES        0x3B        //RXBYTES Overflow and number of bytes in the RX FIFO
#define RCCTRL1_STATUS 0x3C        //RCCTRL1_STATUS Last RC oscillator calibration result
#define RCCTRL0_STATUS 0x3D        //RCCTRL0_STATUS Last RC oscillator calibration result

// CONFIGURATION REGISTERS
#define IOCFG2         0x00        //GDO2 output pin configuration
#define IOCFG1         0x01        //GDO1 output pin configuration
#define IOCFG0         0x02        //GDO0 output pin configuration
#define FIFOTHR        0x03        //RX FIFO and TX FIFO thresholds
#define SYNC1          0x04        //Sync word, high byte
#define SYNC0          0x05        //Sync word, low byte
#define PKTLEN         0x06        //Packet length
#define PKTCTRL1       0x07        //Packet automation control
#define PKTCTRL0       0x08        //Packet automation control
#define ADDR           0x09        //Device address
#define CHANNR         0x0A        //Channel number
#define FSCTRL1        0x0B        //Frequency synthesizer control
#define FSCTRL0        0x0C        //Frequency synthesizer control
#define FREQ2          0x0D        //Frequency control word, high byte
#define FREQ1          0x0E        //Frequency control word, middle byte
#define FREQ0          0x0F        //Frequency control word, low byte
#define MDMCFG4        0x10        //Modem configuration
#define MDMCFG3        0x11        //Modem configuration
#define MDMCFG2        0x12        //Modem configuration
#define MDMCFG1        0x13        //Modem configuration
#define MDMCFG0        0x14        //Modem configuration
#define DEVIATN        0x15        //Modem deviation setting
#define MCSM2          0x16        //Main Radio Control State Machine configuration
#define MCSM1          0x17        //Main Radio Control State Machine configuration
#define MCSM0          0x18        //Main Radio Control State Machine configuration
#define FOCCFG         0x19        //Frequency Offset Compensation configuration
#define BSCFG          0x1A        //Bit Synchronization configuration
#define AGCCTRL2       0x1B        //AGC control
#define AGCCTRL1       0x1C        //AGC control
#define AGCCTRL0       0x1D        //AGC control
#define WOREVT1        0x1E        //High byte Event 0 timeout
#define WOREVT0        0x1F        //Low byte Event 0 timeout
#define WORCTRL        0x20        //Wake On Radio control
#define FREND1         0x21        //Front end RX configuration
#define FREND0         0x22        //Front end TX configuration
#define FSCAL3         0x23        //Frequency synthesizer calibration
#define FSCAL2         0x24        //Frequency synthesizer calibration
#define FSCAL1         0x25        //Frequency synthesizer calibration
#define FSCAL0         0x26        //Frequency synthesizer calibration
#define RCCTRL1        0x27        //RC oscillator configuration
#define RCCTRL0        0x28        //RC oscillator configuration
#define FSTEST         0x29        //Frequency synthesizer calibration control
#define PTEST          0x2A        //Production test
#define AGCTEST        0x2B        //AGC test
#define TEST2          0x2C        //Various test settings
#define TEST1          0x2D        //Various test settings
#define TEST0          0x2E        //Various test settings
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int state = 1;
int prev_state = 0;
int gone_rx = 1;
char SPI_BUFFER[8];
char MSG[1000];
int *pos = MSG;
int RX_BUFFER[66];
uint16_t timer_val;
int recieved_packets = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Select CS
void CS_Select()  {
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_RESET);
}


// Deselect CS
void CS_Deselect()  {
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_SET);
}

// Waits until MISO goes low
void wait_Miso() {
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)>0){

	}
}

//Write to register
void writeReg(uint8_t regAddr, uint8_t value)
{
  CS_Select();                     									   // Select CS
  wait_Miso();                          							   // Wait until MISO goes low
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&regAddr, 1, 100);                // Send register address
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&value, 1, 100);                  // Send value
  CS_Deselect();                    	   							   // Deselect CC1101
}

// Write to register (ONLY USED FOR SETTING THE INTIAL SETTINGS. The name is "halRfWriteReg" so it matches with SMARTRF Studio
void halRfWriteReg(uint8_t setting, uint8_t value)
{
	writeReg(setting, value);
}

// Send command strobe (FIFO_BYTES_AVAILABLE CASE 1)
void command_strobe1(uint8_t cmd){
    cmd = cmd | 0x00;                									// | 0x00 means that R/W=0(FIFO_BYTES_AVAILABLE means the TXFIFO when status byte has been sent)/ burst=0(strobe)
    CS_Select();            		 									// set the CS pin to LOW
    wait_Miso();                          								// Wait until MISO goes low
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&cmd, 1, 100);                   // Send register address
    CS_Deselect();
  }

// Send data
void send_data(char databuffer[],uint8_t length ){
	int i;
	uint8_t element;
	CS_Select();                		 								// set the CS pin to LOW
    wait_Miso();                         								// Wait until MISO goes low
    uint8_t TXfifo = TX_FIFO;
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&TXfifo, 1, 100);               // Access the TX_FIFO register
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&length, 1, 100);				// Send length of data
    for (i = 0; i < length; ++i)
        {
          element = databuffer[i];
		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&TXfifo, 1, 100);               // Send the data
		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&element, 1, 100);                  // Send data
        }
    CS_Deselect();
}
// Send data with sequence numbers
void send_data_sequence(char databuffer[],uint8_t length,uint16_t sequence ){
	int i;
	uint8_t element;
	uint8_t LSB_seq;
	uint8_t MSB_seq;
	LSB_seq = sequence & 0xFF;
	MSB_seq = sequence >> 8;
	CS_Select();                		 								// set the CS pin to LOW
    wait_Miso();                         								// Wait until MISO goes low
    uint8_t TXfifo = TX_FIFO;
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&TXfifo, 1, 100);               // Access the TX_FIFO register
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&length, 1, 100);			   // Send length of data
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&TXfifo, 1, 100);               // Access the TX_FIFO register
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&MSB_seq, 1, 100);			   // Send MSB of sequence number
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&TXfifo, 1, 100);               // Access the TX_FIFO register
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&LSB_seq, 1, 100);			   // Send LSB of sequence number
    for (i = 0; i < length-2; ++i)
        {
          element = databuffer[i];
		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&TXfifo, 1, 100);               // Send the data
		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&element, 1, 100);                  // Send data
        }
    CS_Deselect();
}
// Read register
void readReg(uint8_t regAddr, uint8_t regType) {
    uint8_t addr;
    addr = regAddr | regType;			  								 // Bitwise and to get the right address
    CS_Select();                										 // set the SS pin to LOW
    wait_Miso();                          								 // Wait until MISO goes low
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&addr, 1, 100);                   // Send register address
    HAL_SPI_Receive(&hspi1, (uint8_t*)&SPI_BUFFER, 1, 100);            	 // Read result (need to send dummy message)
    CS_Deselect();                    							 		 // set the SS pin to HIGH

}

void readRegBurst(int buffer[], uint8_t regAddr, uint8_t len)
{
  uint8_t addr;
  int i;
  addr = regAddr | READ_BURST;
  CS_Select();                      								   // Select CS
  wait_Miso();                          							   // Wait until MISO goes low
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&addr, 1, 100);                   // Send register address
  for (i = 0; i < len; ++i)
  	  {
	  HAL_SPI_Receive(&hspi1, (uint8_t*)&buffer[i], 1, 100);   		   	   // Read data and store in buffer
  	  }
  CS_Deselect();                   									   // Deselect CS
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 int bytes_in_RXFIFO;
 int i;
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  CS_Deselect();
  sprintf(MSG, "Serial Monitor Engaged\r\n");
  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
  memset(MSG, 0, sizeof (MSG));

  readReg(MARCSTATE, STATUS_REGISTER);

  sprintf(MSG,"Tranceiver_state: %i\r\n", (unsigned int)SPI_BUFFER[0]);
  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
  memset(MSG, 0, sizeof MSG);
  // Start timer
  HAL_TIM_Base_Start(&htim16);

  //RF-chip SETUP SETTINGS (Imported from SMART RF Studios)
  halRfWriteReg(IOCFG2,0x06);    //GDO2 Output Pin Configuration
  halRfWriteReg(IOCFG0,0x06);    //GDO0 Output Pin Configuration
  halRfWriteReg(FIFOTHR,0x4C);   //RX FIFO and TX FIFO Thresholds
  halRfWriteReg(FSCTRL1,0x08);   //Frequency Synthesizer Control
  halRfWriteReg(FREQ2,0x10);     //Frequency Control Word, High Byte
  halRfWriteReg(FREQ1,0xEA);     //Frequency Control Word, Middle Byte
  halRfWriteReg(FREQ0,0x56);     //Frequency Control Word, Low Byte
  halRfWriteReg(MDMCFG4,0x7B);   //Modem Configuration
  halRfWriteReg(MDMCFG3,0x83);   //Modem Configuration
  halRfWriteReg(MDMCFG2,0x13);   //Modem Configuration
  halRfWriteReg(DEVIATN,0x42);   //Modem Deviation Setting
  halRfWriteReg(MCSM0,0x18);     //Main Radio Control State Machine Configuration
  halRfWriteReg(FOCCFG,0x1D);    //Frequency Offset Compensation Configuration
  halRfWriteReg(BSCFG,0x1C);     //Bit Synchronization Configuration
  halRfWriteReg(AGCCTRL2,0xC7);  //AGC Control
  halRfWriteReg(AGCCTRL1,0x00);  //AGC Control
  halRfWriteReg(AGCCTRL0,0xB2);  //AGC Control
  halRfWriteReg(WORCTRL,0xFB);   //Wake On Radio Control
  halRfWriteReg(FREND1,0xB6);    //Front End RX Configuration
  halRfWriteReg(FSCAL3,0xEA);    //Frequency Synthesizer Calibration
  halRfWriteReg(FSCAL2,0x2A);    //Frequency Synthesizer Calibration
  halRfWriteReg(FSCAL1,0x00);    //Frequency Synthesizer Calibration
  halRfWriteReg(FSCAL0,0x1F);    //Frequency Synthesizer Calibration
  halRfWriteReg(AGCTEST,0x3B);   //AGC Test
  halRfWriteReg(TEST2,0x81);     //Various Test Settings
  halRfWriteReg(TEST1,0x35);     //Various Test Settings
  halRfWriteReg(TEST0,0x09);     //Various Test Settings

  command_strobe1(SIDLE);
  command_strobe1(SFRX);

  //Manually setting the inerrupts so they come after RF-chip SETUP SETTINGS
  // The GDO0 pin glitches the uC otherwise. These settings need to be commented out in MX_GPIO_Init()
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (state == 1 && prev_state != 1 ){
		command_strobe1(SIDLE);
		readReg(MARCSTATE, STATUS_REGISTER);



		sprintf(MSG, "MCU_state: 1    Tranceiver_state: %i\r\n", (unsigned int)SPI_BUFFER[0]);
		HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		memset(MSG, 0, sizeof MSG);

		readReg(RXBYTES, STATUS_REGISTER);
		bytes_in_RXFIFO = SPI_BUFFER[0];
		sprintf(MSG, "Amount of bytes in FIFORX: %i\r\n", (unsigned int)bytes_in_RXFIFO);
		HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		memset(MSG, 0, sizeof MSG);


		sprintf(MSG, "Recieved packets (CRC ok!): %i\r\n", (unsigned int)recieved_packets);
		timer_val = __HAL_TIM_GET_COUNTER(&htim16);
		HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		timer_val = __HAL_TIM_GET_COUNTER(&htim16) - timer_val;
		memset(MSG, 0, sizeof MSG);


		timer_val = __HAL_TIM_GET_COUNTER(&htim16) - timer_val;
		sprintf(MSG, "Counter value: %u Time: %u us \r\n", timer_val, timer_val*4);
		HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		memset(MSG, 0, sizeof MSG);

		gone_rx = 0;
		prev_state = 1;
		recieved_packets = 0;

	  }
	  if (state == 2 && prev_state != 2){

		  if (gone_rx == 0){
		  command_strobe1(SRX);
		  sprintf(MSG, "Gone to RX\r\n");
		  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		  memset(MSG, 0, sizeof MSG);

		  gone_rx = 1;

		  }

		  readReg(MARCSTATE, STATUS_REGISTER);

		  sprintf(MSG, "MCU_state: 2    Tranceiver_state: %i\r\n", (unsigned int)SPI_BUFFER[0]);
		  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		  memset(MSG, 0, sizeof MSG);

		  readReg(RXBYTES, STATUS_REGISTER);
		  sprintf(MSG, "Amount of bytes in FIFORX: %i\r\n", (unsigned int)SPI_BUFFER[0]);
		  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		  memset(MSG, 0, sizeof MSG);


		  prev_state = 2;




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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 192-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65536-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GDO0_Pin */
  GPIO_InitStruct.Pin = GDO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GDO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin LED_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  //HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void  HAL_GPIO_EXTI_Callback(u_int16_t GPIO_Pin){
	int bytes_in_RXFIFO;
	int i;
	int str;

	if (GPIO_Pin == B1_Pin) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		if (state == 1){
			state =2;
			  }
		else {
			state = 1;
		}
	}

	if (GPIO_Pin == GDO0_Pin){
		//timer_val = __HAL_TIM_GET_COUNTER(&htim16);
		while(HAL_GPIO_ReadPin(GDO0_GPIO_Port,GDO0_Pin)>0){
						i += 1;
					}
		readReg(RXBYTES, STATUS_REGISTER);
		bytes_in_RXFIFO = SPI_BUFFER[0];

		if (bytes_in_RXFIFO != 0){


			readReg(RXBYTES, STATUS_REGISTER);
			bytes_in_RXFIFO = SPI_BUFFER[0];

			readRegBurst(RX_BUFFER, RX_FIF0, bytes_in_RXFIFO);

//			*pos = MSG;
//			for (i = 0; i < bytes_in_RXFIFO; ++i)
//			  {
//				  pos += sprintf(pos, " %i", (unsigned int)RX_BUFFER[i]);
//			  }
//			pos += sprintf(pos, "\r\n");
//
//			HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
//			memset(MSG, 0, sizeof MSG);
//			pos = MSG;

			if (RX_BUFFER[bytes_in_RXFIFO-1] & 0x80){
				recieved_packets += 1;
//				sprintf(MSG, "CRC OK!\r\n");
//				HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
//				memset(MSG, 0, sizeof MSG);
			}
			else{
				sprintf(MSG, "CRC NOT OK!\r\n");
				HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
				memset(MSG, 0, sizeof MSG);
			}

			command_strobe1(SRX);



//			timer_val = __HAL_TIM_GET_COUNTER(&htim16) - timer_val;
//			sprintf(MSG, "Counter value: %u Time: %u us \r\n", timer_val, timer_val*4);
//			HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
//			memset(MSG, 0, sizeof MSG);
		}
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
