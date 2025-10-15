/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct UartCommData
{
    uint32_t tick;
    float value;
};
// 接收状态机状态枚举
typedef enum {
    STATE_WAIT_HEADER_1,
    STATE_WAIT_HEADER_2,
    STATE_WAIT_HEADER_3,
    STATE_RECEIVE_DATA
} ReceiveState;

struct CANCommData
{
    uint32_t tick;    // 4 字节
    float value1;     // (需要压缩)
    uint8_t value2;   // 1 字节
    bool flag1;       // (需要压缩)
    bool flag2;       // (需要压缩)
    bool flag3;       // (需要压缩)
    bool flag4;       // (需要压缩)
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t tick = 0; 
//LED
volatile uint8_t led_state = 0;

//UART
struct UartCommData uart_data_tx = {0};
struct UartCommData uart_data_rx = {0}; 

uint8_t uart_tx_buf[9];
uint8_t uart_rx_buf[1]; 

ReceiveState rx_state = STATE_WAIT_HEADER_1; 
uint8_t rx_packet_buf[9];                    
uint8_t rx_data_idx = 0;                     

//CAN 
CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;            

CAN_FilterTypeDef     sFilterConfig;

struct CANCommData    can_tx_data;
struct CANCommData    can_rx_data;   

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void encode_uart_data(uint8_t* buffer, const struct UartCommData* data);
void decode_uart_byte(uint8_t byte);
void unpack_data(const uint8_t* buffer);
void encode_can_frame(const struct CANCommData* data, uint8_t* data_frame);
void decode_can_frame(const uint8_t* data_frame, struct CANCommData* data); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_DMA(&huart2, uart_rx_buf, 1);

  // --- CAN Filter Configuration ---
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // --- Start CAN Service ---
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  // --- Activate CAN RX Notification ---
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  // --- CAN Tx Header Configuration ---
  TxHeader.StdId = 0x100; // 设置发送ID
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8; // 数据长度为8字节 
  TxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    if (RxHeader.StdId == 0x100)
    {
        decode_can_frame(RxData, &can_rx_data);
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) 
  {
    // 1. LED闪烁
    static uint32_t led_counter = 0;
    led_counter++;
    if (led_counter >= 500) // 周期为1000ms
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); 
      led_state = HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin);
      led_counter = 0; 
    }
    
    tick++;

    // 2. UART接收数据
    // 以1kHz频率更新待发送的数据
    uart_data_tx.tick = tick;
    uart_data_tx.value = sinf((float)tick / 1000.0f); 
    //以10Hz频率发送数据
    if (tick % 100 == 0)
    {
      encode_uart_data(uart_tx_buf, &uart_data_tx);
      HAL_UART_Transmit_IT(&huart1, uart_tx_buf, 9); 
    }
    
    // 3. CAN接收数据
    // 以1kHz频率更新待发送的数据
    can_tx_data.tick = tick;
    can_tx_data.value1 = cosf((float)tick / 1000.0f); 
    can_tx_data.value2 = (uint8_t)(tick % 256);
    can_tx_data.flag1 = (tick % 200) < 100;
    can_tx_data.flag2 = (tick % 300) < 150;
    can_tx_data.flag3 = (tick % 400) < 200;
    can_tx_data.flag4 = (tick % 500) < 250;
    
    // 以20Hz频率发送数据
    if (tick % 100 == 0) 
    {
      encode_can_frame(&can_tx_data, TxData);
      if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        Error_Handler();
      }
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        decode_uart_byte(uart_rx_buf[0]);

        HAL_UART_Receive_DMA(&huart2, uart_rx_buf, 1);
    }
}


/**
 * @brief  uart编码
 */
void encode_uart_data(uint8_t* buffer, const struct UartCommData* data)
{
    // 帧头
    buffer[0] = 0xAA;
    buffer[1] = 0xBB;
    buffer[2] = 0xCC;

    buffer[3] = (uint8_t)(data->tick >> 24);
    buffer[4] = (uint8_t)(data->tick >> 16);
    buffer[5] = (uint8_t)(data->tick >> 8);
    buffer[6] = (uint8_t)(data->tick);

    int16_t scaled_value = (int16_t)(data->value * 30000.0f);
    buffer[7] = (uint8_t)(scaled_value >> 8);
    buffer[8] = (uint8_t)(scaled_value);
}
/**
 * @brief  uart解码
 */
void unpack_data(const uint8_t* buffer)
{
    uart_data_rx.tick = ((uint32_t)buffer[3] << 24) |
                        ((uint32_t)buffer[4] << 16) |
                        ((uint32_t)buffer[5] << 8)  |
                        ((uint32_t)buffer[6]);
    // 合并2字节为 int16_t, 再转回 float value
    int16_t scaled_value = ((int16_t)buffer[7] << 8) | buffer[8];
    uart_data_rx.value = (float)scaled_value / 30000.0f;
}
/**
 * @brief  接收状态机
 */
void decode_uart_byte(uint8_t byte)
{
    switch (rx_state)
    {
        case STATE_WAIT_HEADER_1:
            if (byte == 0xAA) {
                rx_packet_buf[0] = byte;
                rx_state = STATE_WAIT_HEADER_2;
            }
            break;

        case STATE_WAIT_HEADER_2:
            if (byte == 0xBB) {
                rx_packet_buf[1] = byte;
                rx_state = STATE_WAIT_HEADER_3;
            } else {
                rx_state = STATE_WAIT_HEADER_1; 
            }
            break;

        case STATE_WAIT_HEADER_3:
            if (byte == 0xCC) {
                rx_packet_buf[2] = byte;
                rx_data_idx = 3; 
                rx_state = STATE_RECEIVE_DATA;
            } else {
                rx_state = STATE_WAIT_HEADER_1; 
            }
            break;

        case STATE_RECEIVE_DATA:
            rx_packet_buf[rx_data_idx++] = byte;
            if (rx_data_idx >= 9) {
                unpack_data(rx_packet_buf);
                rx_state = STATE_WAIT_HEADER_1;
            }
            break;
    }
}

/**
  * @brief can编码
  */
void encode_can_frame(const struct CANCommData* data, uint8_t* data_frame)
{
    // 1. 打包 tick 
    data_frame[0] = (uint8_t)(data->tick >> 24);
    data_frame[1] = (uint8_t)(data->tick >> 16);
    data_frame[2] = (uint8_t)(data->tick >> 8);
    data_frame[3] = (uint8_t)(data->tick);

    // 2. 打包 value1
    int16_t scaled_value1 = (int16_t)(data->value1 * 30000.0f);
    data_frame[4] = (uint8_t)(scaled_value1 >> 8);
    data_frame[5] = (uint8_t)(scaled_value1);

    // 3. 打包 value2
    data_frame[6] = data->value2;

    // 4. 将 4 个布尔值打包到 1 个字节中
    data_frame[7] = 0; 
    if (data->flag1) data_frame[7] |= (1 << 0); 
    if (data->flag2) data_frame[7] |= (1 << 1); 
    if (data->flag3) data_frame[7] |= (1 << 2); 
    if (data->flag4) data_frame[7] |= (1 << 3); 
}

/**
  * @brief can解码
  */
void decode_can_frame(const uint8_t* data_frame, struct CANCommData* data)
{
    // 1. 解包 tick
    data->tick = ((uint32_t)data_frame[0] << 24) |
                 ((uint32_t)data_frame[1] << 16) |
                 ((uint32_t)data_frame[2] << 8)  |
                 ((uint32_t)data_frame[3]);

    // 2. 解包 value1
    int16_t scaled_value1 = ((int16_t)data_frame[4] << 8) | data_frame[5];
    data->value1 = (float)scaled_value1 / 30000.0f;

    // 3. 解包 value2
    data->value2 = data_frame[6];

    // 4. 从 1 个字节中解包 4 个布尔值
    data->flag1 = (data_frame[7] & (1 << 0)) ? true : false;
    data->flag2 = (data_frame[7] & (1 << 1)) ? true : false;
    data->flag3 = (data_frame[7] & (1 << 2)) ? true : false;
    data->flag4 = (data_frame[7] & (1 << 3)) ? true : false;
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
#ifdef USE_FULL_ASSERT
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
