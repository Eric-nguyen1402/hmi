/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "mpf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
MPF_DATA_T      	  mpfData;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;
uint8_t               RxData[8];
uint8_t               Cmd_End[3]={0xFF,0xFF,0xFF};
char 			  	  aRxBuffer[20];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* Function using for PRINTF in stm32 */
#ifdef  __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/* Function of main program */
double Round_up(double input);
char* Cut_firstandlast(char *input);
double Convert_array(char *array);
int compare_array(int array1[], int array2[], int array3[], int array4[], int size);
void Initial_void(void);
void network_init(void);
void Selection(void);
int hexadecimalToDecimal(char hexVal[]);
void Check_RFID(void);
void RFID(void);
void Notify_RFID(void);
void Current(void);
void Left(void);
void Right(void);
void Compare_TQV(void);
void Send_hmi(char*array, int val1, double val2);
void Send_char(char*array1, char*id, char*array2);
void Send_current(char*array1, char*id, char*array2);
void Qei_Tqv_Current(void);
void Send_fault(void);
void Ethernet_Send(void);
void Ethernet_Receive(void);
void Get_data(char*array1, char*array2, int val1, int val2, int val3);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1,0xFFFF);
	return ch;
}

   void HMI_Send_string (char *id, char *string)          // Function: Send string to HMI display
   {
	   char buf[50];
	   sprintf(buf,"%s.txt=\"%s\"",id,string);
	   HAL_UART_Transmit(&huart4,(uint8_t *)buf,strlen(buf),1000);
	   HAL_UART_Transmit(&huart4,Cmd_End,3,100);
   }

/* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */
     void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
     {
       if(huart->Instance == USART2)
       {
      	 uint8_t ret = HAL_OK;
      	     msg++;
      	     if( msg == msg_buff + MAX_RECV_LEN)
      	     {
      	         msg = msg_buff;
      	     }
      	     do
      	     {
      	    	 HAL_UART_Transmit(&huart2, (uint8_t *)msg, 1, 10);
      	         ret = HAL_UART_Receive_IT(&huart2,(uint8_t *)msg,1);
      	         HAL_GPIO_TogglePin(GPIOE,LED_GREEN_Pin);
      	         Flag.usart2 = true;
      	     }while(ret != HAL_OK);
       }
       if (huart->Instance == UART4)
       {
      	 uint8_t ret1 = HAL_OK;
   			 msg1++;
   			 if( msg1 == msg1_buff + MAX_RECV_LEN_S)
   			 {
   				 msg1 = msg1_buff;
   			 }
   			 do
   			 {
   				 ret1 = HAL_UART_Receive_IT(&huart4,(uint8_t *)msg1,1);
   				if (HAL_UART_STATE_READY == huart->RxState && HAL_LOCKED == huart->Lock)
   				{
   					__HAL_UNLOCK(huart);
   				}
   				HAL_GPIO_TogglePin(GPIOE,LED_GREEN_Pin);

   				 Flag.uart4 = true;
   			 }while(ret1 != HAL_OK);
       }
     }

/* This callback is called by the CAN_AddTxMessage */
     void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
     {
   	   	   	   	   	   	   	   	   /* Get RX message */
      	  if( HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK)
      	  {
      		  Error_Handler();
      	  }
      	  if ((RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 8))
      	  {
      	     switch(RxHeader.StdId)
      	     {
				 case 0x12E:
					  memcpy((&(mpfData.data302.voltageLow)), RxData, 8);
					 /* Updata Moto Crank RPM Data */
					 mpfData.nbiotData.motoCrankRPM = ((uint16_t)mpfData.data302.crankRPMhigh << 8) + (uint16_t)mpfData.data302.crankRPMlow;
					  HAL_GPIO_WritePin(GPIOE,LED_RED_Pin,GPIO_PIN_SET);
					  Flag.qei = true;
					  break;
				 case 0x192:
					 memcpy((&(mpfData.data402.torqueLow)), RxData, 8);
					 /* Updata Moto Torque Data */
					 mpfData.nbiotData.motoTorque = (double)((uint16_t)mpfData.data402.torqueLow + ((uint16_t)mpfData.data402.torqueHigh << 8)) * 5 / 1024;
					  HAL_GPIO_WritePin(GPIOE,LED_RED_Pin,GPIO_PIN_SET);
					  Flag.tqv = true;
					  break;
			 }
      	  }
     }


/* This callback is called by the Timer_IRQ interrupt */
     void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
     {
   	   if (htim->Instance == TIM2)
   	   {
   		   if(temp_time2 == 1)
   		   {
   			   Tem.counter = Tem.counter + 1;
   		   }
   	   }
   	   if (htim->Instance == TIM4)
   	   {
   		  switch(temp_time4)
   		  {
   		  case 1:
   			  Left();
   			  break;
   		  case 2:
   			  Right();
   			  break;
   		  case 3:
   			snprintf(time_spent,10,"%d",Tem.counter);
		   for (int i =0, j= 80; ( i<= 4)&&(j <= 83); i++,j++)
		   {
			msg3_buff[j]=time_spent[i];
			if(Tem.counter < 100)
			   {
				   msg3_buff[82]='~';
				   msg3_buff[83]='*';

			   }
			   else if(Tem.counter < 1000)
			   {
				   msg3_buff[83]='~';
				   msg3_buff[84]='*';

			   }
			   else if(Tem.counter < 10000)
			   {
				   msg3_buff[84]='~';
				   msg3_buff[85]='*';
			   }
		   }
   			  break;
   		  case 4:
   			  Compare_TQV();
   			  break;
   		  case 5:
   			  HAL_NVIC_SystemReset();
   		  }
   	   }
     }
/*--------------------------------- Ethernet Function---------------------------------------------------------*/
     void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
     {
       printf("SPI_error %ld",hspi2.ErrorCode);
     }
     void W5500_Select(void) {
         HAL_GPIO_WritePin(GPIOB, W5500_CS_Pin, GPIO_PIN_RESET);  // CS LOW
     }
     void W5500_Unselect(void) {
         HAL_GPIO_WritePin(GPIOB, W5500_CS_Pin, GPIO_PIN_SET);    // CS HIGH
     }
     void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
         HAL_SPI_Receive(&hspi2, buff, len, HAL_MAX_DELAY);
     }
     void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
         HAL_SPI_Transmit(&hspi2, buff, len, HAL_MAX_DELAY);
     }
     uint8_t W5500_ReadByte(void) {
         uint8_t byte;
         W5500_ReadBuff(&byte, sizeof(byte));
         return byte;
     }
     void W5500_WriteByte(uint8_t byte) {
         W5500_WriteBuff(&byte, sizeof(byte));
     }
     void Callback_IPAssigned(void) {
         printf("Callback: IP assigned! Leased time: %ld sec\r\n", getDHCPLeasetime());
         ip_assigned = true;
     }
     void Callback_IPConflict(void) {
         printf("Callback: IP conflict!\r\n");
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  	Initial_void();
	/* Network initialization */
	network_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if 1
	  	  /*READ RFID*/
	if(Flag.usart2 == true)
	{
		RFID();
		Flag.usart2 = false;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 100);
		HAL_Delay(100);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,0);
	}
#endif
#if 1
//	if(flat7 == false)
//	{
//		HAL_Delay(3);
//		HAL_NVIC_SystemReset();
//	}
	if (Flag.uart4 == true)
	  {
		if (msg1_buff[0]==0xff)
		{
			//flat7 = false;
			temp_time4 = 5;
		}
		/* Touch button HOME or RETURN PAGE 0 will reset system*/
		if(msg1_buff[1] == 0x99)
			{
				//flat7 = false;
				temp_time4 = 5;
			}
#if 1
		/*TEST QEI-TQV-CURRENT*/
		if(msg1_buff[1] == 0x31)
			{
				Qei_Tqv_Current();
			}
			    Selection();
#endif
		/*ETHERNET CONNECT*/
		if(msg1_buff[1] == 0x32)
			{
#if 1
				/*Button slowest speed for adding oil motor */
				if(msg1_buff[3] == 0x46)
				{
					TxData[5]= 1;
					TxHeader.StdId = 0x1F5;
					HAL_Delay(2);
						 if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData, &TxMailbox) != HAL_OK)
							{
							   Error_Handler();
							}
				}
				/*Check RFID numbers are different and no lost or the same */
				Notify_RFID();
#endif
#if 1
				/*Touch button SEND*/
				if (msg1_buff[2] == 0x13)
				{
					flat2 = false;
					flat3 = false;
					Ethernet_Send();
				}
				/*Touch button GET*/
				if (msg1_buff[2] == 0x12)
				{
					Ethernet_Receive();
					  if(Flag.recv == true)
					  {
						  /* Display data */
						Get_data(cur1,cur11,6,3,23);	//Sensor washer

						Get_data(cur2,cur22,10,3,24);	//Gear washer

						Get_data(cur3,cur33,14,3,25);	//Shaft washer

						Get_data(cur4,cur44,18,3,26);	//Shaft washer(short side)

						Get_data(cur5,cur55,22,3,27);	//shaft washer

						Get_data(cur,curr,0,5,28);		//Current
						Flag.recv = false;
					  }
				}
#endif
			}
#endif
		}
//#endif
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef  sFilterConfig = {0};
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /*##-1- Configure the CAN Filter ###########################################*/

          sFilterConfig.FilterBank = 0;
          sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
          sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
          sFilterConfig.FilterIdHigh = 0x0000;
          sFilterConfig.FilterIdLow = 0x0000;
          sFilterConfig.FilterMaskIdHigh = 0x0000;
          sFilterConfig.FilterMaskIdLow = 0x0000;
          sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
          sFilterConfig.FilterActivation = ENABLE;
          sFilterConfig.SlaveStartFilterBank = 14;

          if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
          {
            /* Filter configuration Error */
            Error_Handler();
          }
      if (HAL_CAN_Start(&hcan) != HAL_OK)
       {
         /* Start Error */
         Error_Handler();
       }
      /*##-2- Activate CAN RX notification #######################################*/
      if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
      {
        /* Notification Error */
        Error_Handler();
      }
      /*##-3- Configure Transmission process #####################################*/

      TxHeader.ExtId = 0x01;
      TxHeader.RTR = CAN_RTR_DATA;
      TxHeader.IDE = CAN_ID_STD;
      TxHeader.DLC = 8;
      TxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 31999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin|W5500_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W5500_R_GPIO_Port, W5500_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin W5500_CS_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_R_Pin */
  GPIO_InitStruct.Pin = W5500_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W5500_R_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*---------------------------------------------------Secondary Function------------------------------------------------*/
double Round_up(double input)
{
	double value = (int)(input * 100 + .5);
	return value / 100;
}
char * Cut_firstandlast(char *input)
{
  int len = strlen(input);
  if(len > 0)
    input++;                //Go past the first char
  if(len > 1)
    input[len - 2] = '\0'; //Replace the last char with a null termination
  return input;
}

double Convert_array(char *array)
{
	char x = array[14];
	double y;
	int pos = 0;
	int size = sizeof(array);
	for(int i = pos; i< size + 2; i++)
	{
		array[i]=array[i+1];
	}
	size--;
	int len = strlen(array);
	if (len > 0)
	{
		array[len-8] = '\0';
	}
	y = atof(array);
	switch (x)
	{
	case '0':
		break;
	case '1':
		y = y / 10;
		break;
	case '2':
		y = y / 100;
		break;
	}
	return y;
}

int hexadecimalToDecimal(char hexVal[])
{
    int len = strlen(hexVal);

    // Initializing base value to 1, i.e 16^0
    int base = 1;
    int dec_val = 0;

    // Extracting characters as digits from last character
    for (int i=len-1; i>=0; i--)
    {
        // if character lies in '0'-'9', converting
        // it to integral 0-9 by subtracting 48 from
        // ASCII value.
        if (hexVal[i]>='0' && hexVal[i]<='9')
        {
            dec_val += (hexVal[i] - 48)*base;
            // incrementing base by power
            base = base * 16;
        }
        // if character lies in 'A'-'F' , converting
        // it to integral 10 - 15 by subtracting 55
        // from ASCII value
        else if (hexVal[i]>='A' && hexVal[i]<='F')
        {
            dec_val += (hexVal[i] - 55)*base;
            // incrementing base by power
            base = base*16;
        }
    }
    return dec_val;
}
int compare_array(int array1[], int array2[], int array3[], int array4[], int size)
{

   int g,h,k,l,m,n = 0;
	for(int i=1; i< size ; i++)
	{
		if(array1[i] != array2[i])
		{
			g = 1;
		}
		if(array1[i] != array3[i])
		{
			h = 1;
		}
		if(array1[i] != array4[i])
		{
			k = 1;
		}
		if(array2[i] != array3[i])
		{
			l = 1;
		}
		if(array2[i] != array4[i])
		{
			m = 1;
		}
		if(array3[i] != array4[i])
		{
			n = 1;
		}
	}
	if ((g==1)&&(h==1)&&(k==1)&&(l==1)&&(m==1)&&(n==1))
	{
		temp_array = 1;
	}
	else {temp_array = 2;}
	return temp_array;
}
void Send_hmi(char*array, int val1, double val2)
{
	sprintf(array,"t%d.txt=\"%.2f\"",val1,val2);
	HAL_UART_Transmit(&huart4,(uint8_t *)array,strlen(array),1000);
	HAL_UART_Transmit(&huart4,Cmd_End,3,100);
}
void Send_char(char*array1, char*id, char*array2)
{
	sprintf(array1,"%s.txt=\"%s\"",id,array2);
	HAL_UART_Transmit(&huart4,(uint8_t *)array1,strlen(array1),1000);
	HAL_UART_Transmit(&huart4,Cmd_End,3,100);
}
void Send_current(char*array1, char*id, char*array2)
{
	sprintf(array1,"%s.txt=\"%.6s\"",id,array2);
	HAL_UART_Transmit(&huart4,(uint8_t *)array1,strlen(array1),1000);
	HAL_UART_Transmit(&huart4,Cmd_End,3,100);
}
/*-----------------------------------------------------Initial void--------------------------------------------------*/
void Initial_void(void)
{
	/* Open all interrupts in use*/
	if(HAL_UART_Receive_IT(&huart2,(uint8_t *)msg,1) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_UART_Receive_IT(&huart4,(uint8_t *)msg1,1) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	/*Reset W5500 before running program*/
	HAL_GPIO_WritePin(GPIOC,W5500_R_Pin,GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOC,W5500_R_Pin,GPIO_PIN_RESET);
	/* Set all value in buff */
	msg4_buff[0]='~';
	msg4_buff[1]='G';
	msg4_buff[2]='E';
	msg4_buff[3]='T';
	msg4_buff[4]='~';

	for (int a=0,b=37,c=48,d=59; (a<=9)&&(b<=46)&&(c<=57)&&(d<=68); a++,b++,c++,d++)
	{
	   msg3_buff[a]='0';
	   msg3_buff[b]='0';
	   msg3_buff[c]='0';
	   msg3_buff[d]='0';
	}
	msg3_buff[10]='~';

	for(int i=16; i <= 36; i+=4)
	{
		msg3_buff[i]='~';
		if (i == 36)
		{
			i+=11;
			for(i=47; i <=69; i+=11)
			{
				msg3_buff[i]='~';
				if (i == 69)
				{
					i+=5;
					for(i=74; i < 80; i+=5)
					{
						msg3_buff[i]='~';
					}
				}
			}
		}
	}
	for(int i=18; i<35; i+=4)
	{
		msg3_buff[i]='.';
	}
	for(int i=17; i <= 25; i+=4)
	{
		msg3_buff[i]='0';
		if(i == 25)
		{
			for(i =25; i <= 35; i+=2)
			{
				msg3_buff[i]='0';
			}
		}
	}

}
/*-----------------------------------------------------Ethernet void--------------------------------------------------*/
void network_init(void)
{
	/* Chip selection call back */
	#if   _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_
		reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
	#elif _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_FDM_
		reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);  // CS must be tried with LOW.
	#else
	#if (_WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SIP_) != _WIZCHIP_IO_MODE_SIP_
		#error "Unknown _WIZCHIP_IO_MODE_"
	#else
		reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
	#endif
	#endif
	/* SPI Read & Write callback function */
	reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);

	/* WIZCHIP SOCKET Buffer initialize */
	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1){
		printf("WIZCHIP Initialized fail.\r\n");
	while(1);
	}
	/* PHY link status check */
	do{
		if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1){
			printf("Unknown PHY Link status.\r\n");
		}
	}while(tmp == PHY_LINK_OFF);
    uint8_t tmpstr[6];
	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
	ctlnetwork(CN_GET_NETINFO, (void*)&gWIZNETINFO);
	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);
//	uint8_t ip[4];
//	getSIPR (ip);
//	    printf("IP : %d.%d.%d.%d\r\n", ip[0],ip[1],ip[2],ip[3]);
//	    getSUBR(ip);
//	    printf("SN : %d.%d.%d.%d\r\n", ip[0],ip[1],ip[2],ip[3]);
//	    getGAR(ip);
//	    printf("GW : %d.%d.%d.%d\r\n", ip[0],ip[1],ip[2],ip[3]);
//	    printf("Network is ready.\r\n");

}
/*----------------------------------------------------Selection void----------------------------------------------------*/
void Selection(void)
{
	switch (msg1_buff[4])
	  {
	  case 0x70:
		  msg3_buff[19]='3';
		  break;
	  case 0x71:
		  msg3_buff[19]='4';
		  break;
	  case 0x72:
		  msg3_buff[19]='5';
		  break;
	  case 0x73:
		  msg3_buff[19]='6';
		  break;
	  case 0x93:
		  msg3_buff[19]='8';
	  }
	switch (msg1_buff[5])
	  {
	  case 0x74:
		  msg3_buff[23]='3';
		  break;
	  case 0x75:
		  msg3_buff[23]='4';
		  break;
	  case 0x76:
		  msg3_buff[23]='5';
		  break;
	  case 0x77:
		  msg3_buff[23]='6';
		  break;
	  case 0x78:
		  msg3_buff[23]='7';
		  break;
	  }
	switch (msg1_buff[6])
	  {
	  case 0x79:
		  msg3_buff[25]='1';
		  msg3_buff[27]='8';
		  break;
	  case 0x80:
		  msg3_buff[25]='1';
		  msg3_buff[27]='9';
		  break;
	  case 0x81:
		  msg3_buff[25]='2';
		  msg3_buff[27]='0';
		  break;
	  case 0x82:
		  msg3_buff[29]='2';
		  msg3_buff[31]='3';
		  break;
	  case 0x83:
		  msg3_buff[29]='2';
		  msg3_buff[31]='4';
		  break;
	  case 0x84:
		  msg3_buff[29]='2';
		  msg3_buff[31]='5';
		  break;
	  case 0x85:
		  msg3_buff[29]='2';
		  msg3_buff[31]='6';
		  break;
	  case 0x86:
		  msg3_buff[29]='2';
		  msg3_buff[31]='7';
		  break;
	  case 0x87:
		  msg3_buff[33]='5';
		  msg3_buff[35]='5';
		  break;
	  case 0x88:
		  msg3_buff[33]='5';
		  msg3_buff[35]='6';
		  break;
	  case 0x89:
		  msg3_buff[33]='5';
		  msg3_buff[35]='7';
		  break;
	  case 0x90:
		  msg3_buff[33]='5';
		  msg3_buff[35]='8';
		  break;
	  case 0x91:
		  msg3_buff[33]='5';
		  msg3_buff[35]='9';
		  break;
	  case 0x92:
		  msg3_buff[33]='6';
		  msg3_buff[35]='0';
		  break;
	  }
}
/*----------------------------------------------------RFID void----------------------------------------------------*/
void RFID(void)
{
	HAL_Delay(3);
	char *msg_bufff = Cut_firstandlast(msg_buff);
	memcpy(msg2_buff,&msg_bufff[2],8*sizeof(char));
	snprintf(buff4,sizeof(buff4),"%010d",hexadecimalToDecimal(msg2_buff));
			if(msg1_buff[1] == 0x31)
			{
				for (int a=0; a < 10; a++)             // RFID MOTOR
					{
					   msg3_buff[a] = buff4[a];
					   com1[a] = buff4[a];
					}
					t1=1;
					Send_char(buff,"t10",buff4);
			}
			else if(msg1_buff[1]== 0x32)
			{
				switch (msg1_buff[3])
					{
						case 0x40:;
							for(int i = 0,j=37;(i <= 9)&&(j <= 46); i++,j++)      // RFID TORQUE
								{
									msg3_buff[j] = buff4[i];
									com2[i] = buff4[i];
								}
							t2=1;
							Send_char(buff,"bt21",buff4);
							break;
						case 0x42:
							for(int i = 0,j=5;(i <= 9)&&(j <= 16); i++,j++)       // RFID MOTOR WHEN USING GET
								{
									msg4_buff[j] = buff4[i];
								}
							Send_char(buff,"bt20",buff4);
							break;
						case 0x44:
							for(int i = 0,j=48;(i <= 9)&&(j <= 57); i++,j++)      // RFID MODEL
								{
									msg3_buff[j] = buff4[i];
									com3[i] = buff4[i];
								}
							t3=1;
							Send_char(buff,"bt22",buff4);
							compare_array(com1,com2,com3,com4,10);
							break;
					}
			}
		  else
			{
				for(int i = 0,j=59;(i <= 9)&&(j <= 68); i++,j++)              //RFID Employee
					{
						msg3_buff[j] = buff4[i];
						com4[i] = buff4[i];
					}
			  t4=1;
			  Send_char(buff,"t00",buff4);
			  memset(msg1_buff,0,sizeof(msg1_buff));
			}
	  memset(msg_buff, 0, sizeof(msg_buff));
	  msg = msg_buff;
	  (&huart2)->pRxBuffPtr = (uint8_t *)msg;
}
void Notify_RFID(void)
{
	if(flat4 == false)
		{
			if(((t1+t2+t3+t4) == 4)&&(flat2)&&(temp_array == 1))
			{
				HMI_Send_string("t29","可以發送");
			}
			if(((t1+t2+t3+t4) != 4)&&(flat3))
			{
				HMI_Send_string("t29","RFID 遺失");
			}
			if ((temp_array == 2)&&(flat3))
			{
				HMI_Send_string("t29","RFID 一樣");
			}
		}
	if(flat6 == false)
		{
			if(((t1+t2+t3+t4) == 4)&&(flat2)&&(temp_array == 1))
			{
				HMI_Send_string("t29","CAN SEND");
			}
			if(((t1+t2+t3+t4) != 4)&&(flat3))
			{
				HMI_Send_string("t29","LOST RFID");
			}
			if ((temp_array == 2)&&(flat3))
			{
				HMI_Send_string("t29","SAME RFID");
			}
		}
	if (flat5 == false)
	{
		if(((t1+t2+t3+t4) == 4)&&(flat2)&&(temp_array == 1))
			{
				HMI_Send_string("t29","CÓ THỂ GỬI");
			}
			if(((t1+t2+t3+t4) != 4)&&(flat3))
			{
				HMI_Send_string("t29","THIẾU RFID");
			}
			if ((temp_array == 2)&&(flat3))
			{
				HMI_Send_string("t29","TRÙNG RFID");
			}
	}
}
/*----------------------------------------------------QEI-TQV-CURRENT void----------------------------------------------------*/
void Current(void)
{
	/*Send CAN Bus value to boost maximum motor speed */
	TxData[0]= 0xCA;
	TxData[1]= 1;
	TxHeader.StdId = 0x321;
		 if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData, &TxMailbox) != HAL_OK)
			{
			 //  Error_Handler();
			   printf("fault current \r\n" );
			}
									   /* Motor Run */
		 HAL_UART_Transmit(&huart1,(uint8_t *)ar, strlen(ar),1000);
		 HAL_UART_Receive(&huart1,(uint8_t *)aRxBuffer, 10,500);

		 Send_current(buf1,"t17",aRxBuffer);

		 HAL_UART_Transmit(&huart1,(uint8_t *)ar1, strlen(ar1),1000);
		 HAL_UART_Receive(&huart1,(uint8_t *)aRxBuffer1, 20,500);

		 Tem.z = Convert_array(aRxBuffer1);
		 snprintf(aRxBuffer_new,20,"%f", Tem.z);

		 for(int i = 0,j=11;(i <= 5)&&(j <= 15); i++,j++)
			{
				msg3_buff[j]=aRxBuffer_new[i];
			}
		 Send_current(buf2,"t18",aRxBuffer_new);
}
void Left(void)
{
	TxHeader.StdId = 0x12D;
	if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData, &TxMailbox) != HAL_OK)
	   {
		   Error_Handler();
	   }
	if(Flag.qei == true)
	  {
		sprintf(buff2,"t13.txt=\"%2.2x\"",mpfData.nbiotData.motoCrankRPM);
		HAL_UART_Transmit(&huart4,(uint8_t *)buff2,strlen(buff2),1000);
		HAL_UART_Transmit(&huart4,Cmd_End,3,100);
	  }
}
void Right(void)
{
	TxHeader.StdId = 0x12D;
	if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData, &TxMailbox) != HAL_OK)
	 {
		Error_Handler();
	 }
	if(Flag.qei == true)
	{
		sprintf(buff2,"t12.txt=\"%2.2x\"",mpfData.nbiotData.motoCrankRPM);
		HAL_UART_Transmit(&huart4,(uint8_t *)buff2,strlen(buff2),1000);
		HAL_UART_Transmit(&huart4,Cmd_End,3,100);
	}
}
void Compare_TQV(void)
{
	Send_hmi(buff,15,mpfData.nbiotData.motoTorque);
	Tem.size++;
	arr[Tem.size] = mpfData.nbiotData.motoTorque;
	if (Tem.size == 300){Tem.size =0;}
							   /* Find max and min*/
			 Tem.min = arr[1];
			 Tem.max = arr[1];
				   for(int i =2; i<Tem.size; i++)
				   {
						if (arr[i] < Tem.min)
						   {
							Tem.min = arr[i];
						   }

						if (arr[i] > Tem.max)
						   {
							Tem.max = arr[i];
						   }

				   }
				   Send_hmi(buff3,16,Tem.min);

				   Send_hmi(buff1,14,Tem.max);

				snprintf(max1,10,"%f",Round_up(Tem.max));
				for(int i =0, j = 75;(i<= 3)&&(j <= 78); i++, j++)
				{
					msg3_buff[j]= max1[i];
				}
				snprintf(min1,10,"%f",Round_up(Tem.min));
				for(int i =0, j = 70;(i<= 3)&&(j <= 73); i++, j++)
				{
					msg3_buff[j]= min1[i];
				}
			  if (msg1_buff[3] == 52)                 // Model "41" Motor
			  {
				  if (Tem.min <= 4.10 )
				  {
					switch(msg1_buff[7])
						{
						case 0x27:
							HMI_Send_string("t11","Success");
							flat6 = false;
							break;
						case 0x28:
							HMI_Send_string("t11","成功");
							flat4 = false;
							break;
						case 0x29:
							HMI_Send_string("t11","Đạt");
							flat5 = false;
							break;
						}
				  }
				  else
				  {
					switch(msg1_buff[7])
						{
						case 0x27:
							HMI_Send_string("t11","Fail");
							flat6 = false;
							break;
						case 0x28:
							HMI_Send_string("t11","失敗");
							flat4 = false;
							break;
						case 0x29:
							HMI_Send_string("t11","Không đạt");
							flat5 = false;
							break;
						}
				  }
			  }
			  else                                   // For Another motors (6.0 or M2...)
			  {
						  /* Calculate value and give the motor's result*/
				if((Tem.max - Tem.min) <= 0.3 && (Tem.max - Tem.min) >= 0.11 )
					{
					switch(msg1_buff[7])
						{
						case 0x27:
							HMI_Send_string("t11","Success");
							flat6 = false;
							break;
						case 0x28:
							HMI_Send_string("t11","成功");
							flat4 = false;
							break;
						case 0x29:
							HMI_Send_string("t11","Đạt");
							flat5 = false;
							break;
						}
					}
				else
					{
					switch(msg1_buff[7])
						{
						case 0x27:
							HMI_Send_string("t11","Fail");
							flat6 = false;
							break;
						case 0x28:
							HMI_Send_string("t11","失敗");
							flat4 = false;
							break;
						case 0x29:
							HMI_Send_string("t11","Không đạt");
							flat5 = false;
							break;
						}
					}
			  }
}
void Qei_Tqv_Current(void)
{
	flat=true;
	temp_time2 = 1;
		if(msg1_buff[2]==0x36)
		{
		  TxData[0]= 00;
		  TxHeader.StdId = 0x191;
			if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler();
			}
			HAL_Delay(0.5);
			if (mpfData.data402.cw != 0)
			{
				HAL_Delay(2);
				temp_time4 = 1;
			}
			else
			{
				HAL_Delay(2);
				temp_time4 = 2;
			}
/*-------------------------------------------------------------------------------TEST TQV-----------------------------------------------------------------------------*/
			if(Flag.tqv == true)
			{
				HAL_Delay(3);
				temp_time4 = 4;
			}
		}
		else if(msg1_buff[2] == 37)
		{
			memset(TxData,0, sizeof(TxData));
			memset(RxData,0, sizeof(RxData));
			memset(max1,0,sizeof(max1));
			memset(min1,0,sizeof(min1));
			memset(arr,0,sizeof(arr));
			Flag.tqv = false;
			Flag.qei = false;
		}
/*-------------------------------------------------------------------------------READ CURRENT-----------------------------------------------------------------------------*/
	  if (msg1_buff[3] == 0x01)
		{
			Current();
		}
	  else
		{
		  memset(TxData,0,sizeof(TxData));
		}
}
/*----------------------------------------------------ETHERNET SEND void----------------------------------------------------*/
void Send_fault(void)
{
	if (gDATABUF[0] == 78)
		{
			flat3=false;
			flat=false;
			if(flat4 == false)
			{
				switch (gDATABUF[1])
				{
				case 49:            /* Fault N1 */
					HMI_Send_string("t70","沒有 RFID 馬達");
					break;
				case 50:			/* Fault N2 */
					HMI_Send_string("t70","沒有 RFID 品號");
					break;
				case 51:			/* Fault N3 */
					HMI_Send_string("t70","沒有 RFID 扭力");
					break;
				case 52:			/* Fault N4 */
					HMI_Send_string("t70","沒有 RFID 工人");
					break;
				case 53:			/* Fault N5 */
					HMI_Send_string("t70","錯誤字串");
					break;
				}
			}
			if(flat6 == false)
			{
				switch (gDATABUF[1])
				{
				case 49:            /* Fault N1 */
					HMI_Send_string("g70","Wrong RFID Motor. Testing again");
					break;
				case 50:			/* Fault N2 */
					HMI_Send_string("g70","Wrong RFID Model. Testing again");
					break;
				case 51:			/* Fault N3 */
					HMI_Send_string("g70","Wrong RFID Torque. Testing again");
					break;
				case 52:			/* Fault N4 */
					HMI_Send_string("g70","Wrong RFID Employee. Testing again");
					break;
				case 53:			/* Fault N5 */
					HMI_Send_string("g70","Wrong string. Testing again");
					break;
				}
			}
			if(flat5 == false)
			{
				switch (gDATABUF[1])
				{
				case 49:            /* Fault N1 */
					HMI_Send_string("t70","Lỗi RFID động cơ. Xin kiểm tra lại");
					break;
				case 50:			/* Fault N2 */
					HMI_Send_string("t70","Lỗi RFID mẫu motor. Xin kiểm tra lại");
					break;
				case 51:			/* Fault N3 */
					HMI_Send_string("t70","Lỗi RFID momen. Xin kiểm tra lại");
					break;
				case 52:			/* Fault N4 */
					HMI_Send_string("t70","Lỗi RFID nhân viên. Xin kiểm tra lại");
					break;
				case 53:			/* Fault N5 */
					HMI_Send_string("t70","Lỗi dữ liệu. Xin kiểm tra lại");
					break;
				}
			}
		}
}
void Ethernet_Send(void)
{
	//temp_time4 = 3;
	HAL_Delay(4);
		  switch(getSn_SR(SOCK_TCPS))
		  {
				case SOCK_INIT:
						connect(SOCK_TCPS,DstIP,DstPort);   // TCP Client connect to server
				break;
				case SOCK_ESTABLISHED:
						len=getSn_RX_RSR(SOCK_TCPS);
						memset(gDATABUF,0,sizeof(gDATABUF));
						if (flat)
						{
							send(SOCK_TCPS,(uint8_t *)msg3_buff,strlen(msg3_buff));
							flat=false;
							current = Tem.counter;
						}
						if ((Tem.counter-current)>10)
						{
							flat = true;
						}
						for (int i = 0; i<=100; i++)
						{
							if (i == 100)
							{
								flat = true;
							}
						}
							recv(SOCK_TCPS,gDATABUF,len);

							if (gDATABUF[0] == 79)
							{
								if(flat4 == false)
									{
										HMI_Send_string("t70","發送成功,測試馬達完成");    // Remember ","
									}
								if(flat6 == false)
									{
										HMI_Send_string("g70","Send successfully. Testing motor is completed");
									}
								if(flat5 == false)
									{
										HMI_Send_string("t70","Gửi thành công. Hoàn tất kiểm tra");
									}
								flat=false;
								disconnect(SOCK_TCPS);
								memset(gDATABUF,0,sizeof(gDATABUF));
							}
							Send_fault();
				break;
				case SOCK_CLOSE_WAIT:
						disconnect(SOCK_TCPS);
				break;
				case SOCK_CLOSED:
						socket(SOCK_TCPS,Sn_MR_TCP,5000,0x00);
				break;
		  }
}
/*----------------------------------------------------ETHERNET Receive void----------------------------------------------------*/
void Ethernet_Receive(void)
{
		switch(getSn_SR(SOCK_TCPS1))
		{
			case SOCK_INIT:
					connect(SOCK_TCPS1,DstIP,DstPort);   // TCP Client connect to server
			break;
			case SOCK_ESTABLISHED:
					len1=getSn_RX_RSR(SOCK_TCPS1);
					memset(hDATABUF,0,sizeof(hDATABUF));
					if(flat1)
					{
						send(SOCK_TCPS1,(uint8_t *)msg4_buff,strlen(msg4_buff));
						flat1 = false;
					}
					recv(SOCK_TCPS1,hDATABUF,len1);
					if(len1)
					{
						for(int a = 0;a< sizeof(hDATABUF); a++)
						{
							kDATABUF[a]=hDATABUF[a];
						}
					Flag.recv = true;
					flat1 = false;
					memset(hDATABUF,0,sizeof(hDATABUF));
					}
			break;
			case SOCK_CLOSE_WAIT:
					disconnect(SOCK_TCPS1);
			break;
			case SOCK_CLOSED:
					socket(SOCK_TCPS1,Sn_MR_TCP,5000,0x00);
			break;
		}
}

/*----------------------------------------------------GET INFORMATION void----------------------------------------------------*/
void Get_data(char*array1, char*array2, int val1, int val2, int val3)
{
	memcpy(array1,&kDATABUF[val1],val2*sizeof(char));
	sprintf(array2,"t%d.txt=\"%s\"",val3,array1);
	HAL_UART_Transmit(&huart4,(uint8_t *)array2,strlen(array2),1000);
	HAL_UART_Transmit(&huart4,Cmd_End,3,100);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
