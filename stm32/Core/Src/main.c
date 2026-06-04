/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"
#include "RobStride.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MASTER_ID 0xFD
#define MOTOR_ID 0x7F // 你的电机ID=127
// 电机使能运行ID（通信类型3）
#define MOTOR_ENABLE_ID ((0x03 << 24) | (0x00 << 16) | (MASTER_ID << 8) | MOTOR_ID)
// 运控模式控制ID（通信类型1）
#define MOTOR_CONTROL_ID ((0x01 << 24) | (0x00 << 16) | (MASTER_ID << 8) | MOTOR_ID)
// 测试帧ID（保留调试用）
#define TEST_FRAME_ID ((0x12 << 24) | (0x00 << 16) | (MASTER_ID << 8) | MOTOR_ID)


// AT帧格式：[0x41,0x54] + [4字节CAN_ID] + [1字节数据长度] + [N字节数据] + [0x0D,0x0A]
#define AT_FRAME_HEAD_1 0x41
#define AT_FRAME_HEAD_2 0x54
#define AT_FRAME_TAIL_1 0x0D
#define AT_FRAME_TAIL_2 0x0A
#define AT_FRAME_MAX_LEN 32 // 最大帧长度


// 官方库通信类型定义（和手册完全一致）
#define COMM_TYPE_MotionControl     0x01
#define COMM_TYPE_FEEDBACK  0x02  // 电机反馈
#define COMM_TYPE_ENABLE    0x03  // 电机使能
#define COMM_TYPE_DISABLE   0x04  // 电机失能
#define COMM_TYPE_SetPosZero        0x06
#define COMM_TYPE_Can_ID            0x07

#define COMM_TYPE_GET_PARAM 0x11  // 单个参数读取
#define COMM_TYPE_SET_PARAM 0x12  // 单个参数写入


// 参数索引定义（来自官方库参数表）
#define PARAM_RUN_MODE      0x7005  // 运行模式
#define TARGET_CURRENT      0x7006
#define PARAM_SPD_REF       0x700A  // 速度指令
#define PARAM_LOC_REF       0x7016  // 位置指令
#define PARAM_LIMIT_SPEED   0x7017
#define PARAM_LIMIT_CUR     0x7018  // 电流限制


// 运行模式定义
#define MODE_MOTION         0  // 运控模式
#define MODE_POSITION       1  // 位置模式
#define MODE_SPEED          2  // 速度模式
#define MODE_CURRENT        3  // 电流模式
#define MODE_Set_Zero       4


// AT帧解析状态机
typedef enum {
    AT_STATE_WAIT_HEAD1,
    AT_STATE_WAIT_HEAD2,
    AT_STATE_READ_CAN_ID,
    AT_STATE_READ_LEN,
    AT_STATE_READ_DATA,
    AT_STATE_WAIT_TAIL1,
    AT_STATE_WAIT_TAIL2
} AT_ParseState;

// AT帧结构体
typedef struct {
    uint32_t can_id;
    uint8_t data_len;
    uint8_t data[8];
} AT_Frame;

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// AT帧解析相关
uint8_t at_rx_buf[AT_FRAME_MAX_LEN];
uint8_t at_rx_ptr = 0;
AT_ParseState at_parse_state = AT_STATE_WAIT_HEAD1;
AT_Frame at_rx_frame;

uint8_t receiveData[50];

uint8_t pRxdata[8], pTxdata[8]; 
RobStride_Motor RobStride_01(0x7F, false);

uint8_t mode = 0; 

	// FDCAN全局变量
uint8_t FDCAN_RxData[8];
uint32_t FDCAN_RxId;
uint8_t FDCAN_RxLen;
uint8_t FDCAN_RxFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

extern DMA_HandleTypeDef hdma_usart1_rx;


// 发送AT帧到串口
void AT_SendFrame(uint32_t can_id, uint8_t *data, uint8_t len)
{
    uint8_t buf[AT_FRAME_MAX_LEN];
    buf[0] = AT_FRAME_HEAD_1;
    buf[1] = AT_FRAME_HEAD_2;
    // CAN_ID大端序（和厂家一致）
    buf[2] = (can_id >> 24) & 0xFF;
    buf[3] = (can_id >> 16) & 0xFF;
    buf[4] = (can_id >> 8) & 0xFF;
    buf[5] = can_id & 0xFF;
    buf[6] = len;
    // 数据
    memcpy(&buf[7], data, len);
    // 帧尾
    buf[7 + len] = AT_FRAME_TAIL_1;
    buf[8 + len] = AT_FRAME_TAIL_2;
    // 发送
    HAL_UART_Transmit_DMA(&huart1, buf, 9 + len);
}


//CAN 
/************************** FDCAN核心函数 **************************/
/**
 * @brief  FDCAN发送经典CAN 2.0B扩展帧
 * @param  can_id 29位扩展ID（高3位为0）
 * @param  data 数据指针（0~8字节）
 * @param  len 数据长度
 * @retval 0成功，1失败
 */
uint8_t FDCAN_SendExtFrame(uint32_t can_id, uint8_t *data, uint8_t len)
{
     FDCAN_TxHeaderTypeDef TxHeader = {0};
    if (len > 8) return 1;
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) return 1;

    TxHeader.Identifier = can_id & 0x1FFFFFFF;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = len;  // 经典CAN模式直接赋值len
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) == HAL_OK) ? 0 : 1;
}
/************************** 接收中断回调（处理电机反馈）**************************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, FDCAN_RxData) == HAL_OK)
        {
            if (RxHeader.IdType == FDCAN_EXTENDED_ID)
            {
//                FDCAN_RxId = RxHeader.Identifier;
//                FDCAN_RxLen = RxHeader.DataLength >> 16;// 经典CAN模式需要右移16位
//                FDCAN_RxFlag = 1;
							 // 解析厂家类库
                RobStride_01.RobStride_Motor_Analysis(FDCAN_RxData, RxHeader.Identifier);
                // ✅ 封装成AT帧回传给Unity
                AT_SendFrame(RxHeader.Identifier, FDCAN_RxData, RxHeader.DataLength & 0x0F);
                // 翻转LED3表示收到CAN反馈
                HAL_GPIO_TogglePin(GPIOE, LED3_Pin);
							FDCAN_RxFlag = 1;
            }
        }
    }
}



/************************** 节点1：电机使能/失能 **************************/
/**
 * @brief  电机使能（官方库通信类型3）
 * @retval 0成功
 */
uint8_t Motor_Enable(void)
{
    uint8_t data[8] = {0};
    uint32_t can_id = (COMM_TYPE_ENABLE << 24) | (MASTER_ID << 8) | MOTOR_ID;
    return FDCAN_SendExtFrame(can_id, data, 8);
}

/**
 * @brief  电机失能（官方库通信类型4）
 * @param  clear_error 0=不清除错误 1=清除错误
 * @retval 0成功
 */
uint8_t Motor_Disable(uint8_t clear_error)
{
    uint8_t data[8] = {0};
    data[0] = clear_error;
    uint32_t can_id = (COMM_TYPE_DISABLE << 24) | (MASTER_ID << 8) | MOTOR_ID;
    return FDCAN_SendExtFrame(can_id, data, 8);
}

/************************** 节点2：单个参数写入 **************************/
/**
 * @brief  写入单个参数（官方库通信类型18）
 * @param  index 参数索引
 * @param  value 参数值（浮点数）
 * @retval 0成功
 */
uint8_t Motor_SetParam(uint16_t index, float value)
{
  uint8_t data[8] = {0};
    
    // Byte0~1：参数索引（低字节在前，和官方库完全一致）
    data[0] = index & 0xFF;
    data[1] = (index >> 8) & 0xFF;
    
    // Byte4~7：参数值（IEEE754单精度浮点数，低字节在前）
    memcpy(&data[4], &value, 4);
    
    uint32_t can_id = (COMM_TYPE_SET_PARAM << 24) | (MASTER_ID << 8) | MOTOR_ID;
    return FDCAN_SendExtFrame(can_id, data, 8);
}

/************************** 节点2.5：单字节模式写入 **************************/
/**
 * @brief  写入运行模式（单字节模式，对应官方库'j'模式）
 * @param  index 参数索引
 * @param  mode 模式值（0=MOTION, 1=POSITION, 2=SPEED, 3=CURRENT, 4=ZERO）
 * @retval 0成功
 */
uint8_t Motor_SetParamMode(uint16_t index, uint8_t mode)
{
    uint8_t data[8] = {0};
    data[0] = index & 0xFF;
    data[1] = (index >> 8) & 0xFF;
    data[4] = mode;  // 'j' mode: single byte, not float32
    uint32_t can_id = (COMM_TYPE_SET_PARAM << 24) | (MASTER_ID << 8) | MOTOR_ID;
    return FDCAN_SendExtFrame(can_id, data, 8);
}


/************************** 节点3：速度模式控制 **************************/
/**
 * @brief  进入速度模式并设置速度
 * @param  speed 目标速度(rad/s)，范围-44~44
 * @param  current_limit 电流限制(A)，范围0~23
 * @retval 0成功
 */
uint8_t Motor_EnterSpeedMode(float speed, float current_limit)
{
    // 1. 先失能电机
    Motor_Disable(1);
    HAL_Delay(100);
    
    // 2. 设置运行模式为速度模式（单字节格式，对应官方库'j'模式）
    if(Motor_SetParamMode(PARAM_RUN_MODE, MODE_SPEED) != 0) return 1;
    HAL_Delay(100);
    
    // 3. 设置电流限制
    if(Motor_SetParam(PARAM_LIMIT_CUR, current_limit) != 0) return 1;
    HAL_Delay(100);
    
    // 4. 设置初始速度为0
    if(Motor_SetParam(PARAM_SPD_REF, 0.0f) != 0) return 1;
    HAL_Delay(100);
    
    // 5. 使能电机
    if(Motor_Enable() != 0) return 1;
    HAL_Delay(100);
    
    // 6. 设置目标速度
    if(Motor_SetParam(PARAM_SPD_REF, speed) != 0) return 1;
    
    return 0;
}
/************************** 节点5：反馈数据解析 **************************/
typedef struct {
    float position;  // 当前位置(rad)
    float speed;     // 当前速度(rad/s)
    float current;   // 当前电流(A)
    float temp;      // 当前温度(℃)
    uint8_t error;   // 错误码
    const char* error_msg; // 错误信息
} Motor_Feedback_t;

// ✅ 厂家官方错误码解析表（手册4.1.10）
const char* Motor_GetErrorMsg(uint8_t error)
{
    if(error == 0x00) return "no error";
    if(error & 0x01) return "over voltage";    // 过压
    if(error & 0x02) return "driver fault";    // 驱动芯片故障
    if(error & 0x04) return "over temp";       // 过温
    if(error & 0x08) return "encoder fault";   // 磁编码故障
    if(error & 0x10) return "overload";        // 过载
    if(error & 0x20) return "not calibrated";  // 未标定
    if(error & 0x80) return "hall fault";      // HALL编码故障
    return "unknown error";
}

Motor_Feedback_t Motor_ParseFeedback(uint32_t can_id, uint8_t *data)
{
    Motor_Feedback_t fb = {0};
    
    // 只解析通信类型2的反馈帧
    if(((can_id >> 24) & 0xFF) == COMM_TYPE_FEEDBACK)
    {
        // 解析错误码
        fb.error = (can_id >> 16) & 0xFF;
        fb.error_msg = Motor_GetErrorMsg(fb.error);

     // ✅ 修复1：大端序！高字节在前（手册明确要求）
        // 解析位置（Byte0~1，无符号16位，映射到-4π~4π）
        uint16_t pos_raw = (data[0] << 8) | data[1];
        fb.position = (pos_raw - 32768.0f) * 4.0f * 3.1415926535f / 32768.0f;
        
        // 解析速度（Byte2~3，无符号16位，映射到-44~44rad/s）
        uint16_t speed_raw = (data[2] << 8) | data[3];
        fb.speed = (speed_raw - 32768.0f) * 44.0f / 32768.0f;
        
        // 解析电流（Byte4~5，无符号16位，映射到-17~17A）
        uint16_t current_raw = (data[4] << 8) | data[5];
        fb.current = (current_raw - 32768.0f) * 17.0f / 32768.0f;
        
        // 解析温度（Byte6~7，大端序，乘以0.1）
        uint16_t temp_raw = (data[6] << 8) | data[7];
        fb.temp = temp_raw * 0.1f;
    }
    
    return fb;
}

uint8_t Motor_DisableHallFault(void)
{
    // 参数索引0x7028：HALL故障检测开关（0=关闭，1=开启）
    // 这是厂家扩展参数，所有RobStride电机通用
    return Motor_SetParam(0x7028, 0.0f);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart1)
    {
        for(int i = 0; i < Size; i++)
        {
            uint8_t ch = receiveData[i];
            switch(at_parse_state)
            {
                case AT_STATE_WAIT_HEAD1:
                    if(ch == AT_FRAME_HEAD_1) at_parse_state = AT_STATE_WAIT_HEAD2;
                    break;
                case AT_STATE_WAIT_HEAD2:
                    if(ch == AT_FRAME_HEAD_2)
                    {
                        at_parse_state = AT_STATE_READ_CAN_ID;
                        at_rx_ptr = 0;
                    }
                    else at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
                case AT_STATE_READ_CAN_ID:
                    at_rx_buf[at_rx_ptr++] = ch;
                    if(at_rx_ptr == 4)
                    {
                        at_rx_frame.can_id = (at_rx_buf[0] << 24) | (at_rx_buf[1] << 16) | (at_rx_buf[2] << 8) | at_rx_buf[3];
                        at_parse_state = AT_STATE_READ_LEN;
                        at_rx_ptr = 0;
                    }
                    break;
                case AT_STATE_READ_LEN:
                    at_rx_frame.data_len = ch;
                    if(at_rx_frame.data_len > 8) at_rx_frame.data_len = 8;
                    at_parse_state = AT_STATE_READ_DATA;
                    at_rx_ptr = 0;
                    break;
                case AT_STATE_READ_DATA:
                    at_rx_frame.data[at_rx_ptr++] = ch;
                    if(at_rx_ptr == at_rx_frame.data_len)
                        at_parse_state = AT_STATE_WAIT_TAIL1;
                    break;
                case AT_STATE_WAIT_TAIL1:
                    if(ch == AT_FRAME_TAIL_1) at_parse_state = AT_STATE_WAIT_TAIL2;
                    else at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
                case AT_STATE_WAIT_TAIL2:
                    if(ch == AT_FRAME_TAIL_2)
                    {
                        FDCAN_SendExtFrame(at_rx_frame.can_id, at_rx_frame.data, at_rx_frame.data_len);
                        HAL_GPIO_TogglePin(GPIOE, LED2_Pin);
                    }
                    at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
                default:
                    at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}
                    else at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
                case AT_STATE_READ_CAN_ID:
                    at_rx_buf[at_rx_ptr++] = ch;
                    if(at_rx_ptr == 4)
                    {
                        // 解析CAN_ID（大端序）
                        at_rx_frame.can_id = (at_rx_buf[0] << 24) | (at_rx_buf[1] << 16) | (at_rx_buf[2] << 8) | at_rx_buf[3];
                        at_parse_state = AT_STATE_READ_LEN;
                        at_rx_ptr = 0;
                    }
                    break;
                case AT_STATE_READ_LEN:
                    at_rx_frame.data_len = ch;
                    if(at_rx_frame.data_len > 8) at_rx_frame.data_len = 8; // 限制最大8字节
                    at_parse_state = AT_STATE_READ_DATA;
                    at_rx_ptr = 0;
                    break;
                case AT_STATE_READ_DATA:
                    at_rx_frame.data[at_rx_ptr++] = ch;
                    if(at_rx_ptr == at_rx_frame.data_len)
                    {
                        at_parse_state = AT_STATE_WAIT_TAIL1;
                    }
                    break;
                case AT_STATE_WAIT_TAIL1:
                    if(ch == AT_FRAME_TAIL_1) at_parse_state = AT_STATE_WAIT_TAIL2;
                    else at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
                case AT_STATE_WAIT_TAIL2:
                    if(ch == AT_FRAME_TAIL_2)
                    {
                        // ✅ 收到完整AT帧，发送到CAN总线
                        FDCAN_SendExtFrame(at_rx_frame.can_id, at_rx_frame.data, at_rx_frame.data_len);
                        // 翻转LED2表示收到指令
                        HAL_GPIO_TogglePin(GPIOE, LED2_Pin);
                    }
                    at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
                default:
                    at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
//   HAL_UART_Transmit_DMA(&huart1, receiveData,Size);	
//	 if(receiveData[0]==0xAA)
//	 {
//	  if (receiveData[1]==Size)
//		 {
//		  uint8_t sum=0;
//			for (int i=0;i<Size-1 ; i++)  sum+=receiveData[i];
//			if (sum==receiveData[Size -1])
//			{
//			
//		  	 for (int i=2;i<Size-1;i+=2)
//				{
//					GPIO_PinState state = GPIO_PIN_SET;
//				  if (receiveData[i+1] == 0x00)  state = GPIO_PIN_RESET;
//					if (receiveData[i] == 0x01)
//					HAL_GPIO_WritePin(GPIOE,LED1_Pin, state) ;
//					else if (receiveData[i] == 0x02)
//					HAL_GPIO_WritePin(GPIOE,LED2_Pin, state) ;
//					else if (receiveData[i] == 0x03)
//					HAL_GPIO_WritePin(GPIOE,LED3_Pin, state) ;
//				}
//			}
//		 }
//	 }
        }
		 }
	 }
	 HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData,sizeof(receiveData));
   __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);


 }
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);


// 配置FDCAN

    if(FDCAN_FilterConfig()!= 0)
        printf("FDCAN filter set failed！\r\n");
    else
        printf("FDCAN filter set seccessfully\r\n");
		
    if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        printf("FDCAN set failed!\r\n");
    else
        printf("FDCAN set seccessfully\r\n");
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
 // ... 前面的初始化代码不变 ...
    printf("KEY1：进入速度模式，速度5rad/s，电流限制10A\r\n");
    printf("KEY2：停止电机（速度设为0）\r\n");
    printf("KEY3：失能电机\r\n");

  
   uint8_t motor_running = 0;
    uint32_t print_cnt = 0;
    while (1)
    {
			
		 if(FDCAN_RxFlag)
        {
            Motor_Feedback_t fb = Motor_ParseFeedback(FDCAN_RxId, FDCAN_RxData);
            
            // 每500ms打印一次（更流畅）
            if(print_cnt % 10 == 0)
            {
                // ✅ 纯英文输出，彻底避免乱码
                printf("Pos: %.2f rad | Spd: %.2f rad/s | Cur: %.2f A | Temp: %.1f C | Err: 0x%02X (%s)\r\n",
                       fb.position, fb.speed, fb.current, fb.temp, fb.error, fb.error_msg);
            }
            
            print_cnt++;
            FDCAN_RxFlag = 0;
        }
uint8_t key = KEY_Scan();
        
        if(key == KEY1_PRES && !motor_running)
        {
            printf("Clearing errors...\r\n");
            Motor_Disable(1); // 清除所有错误
            HAL_Delay(200);
            
            printf("Entering speed mode...\r\n");
            if(Motor_EnterSpeedMode(5.0f, 10.0f) == 0)
            {
                printf("Speed mode started successfully!\r\n");
                motor_running = 1;
                HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
            }
            else
            {
                printf("Speed mode start failed!\r\n");
            }
        }
        else if(key == KEY2_PRES && motor_running)
        {
            if(Motor_SetParam(PARAM_SPD_REF, 0.0f) == 0)
            {
                printf("Motor stopped\r\n");
                motor_running = 0;
            }
            else
            {
                printf("Motor stop failed\r\n");
            }
        }
        else if(key == KEY3_PRES)
        {
            Motor_Disable(1);
            printf("Motor disabled\r\n");
            motor_running = 0;
            HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
        }

        // 持续发送速度指令
        if(motor_running)
        {
            Motor_SetParam(PARAM_SPD_REF, 5.0f);
        }

        HAL_GPIO_TogglePin(GPIOE, LED1_Pin);
        HAL_Delay(50);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    /* HAL FDCAN Error callback uses only the handle; read error via HAL_FDCAN_GetError */
    uint32_t Error = HAL_FDCAN_GetError(hfdcan);
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "FDCAN Error: 0x%08lX\r\n", (unsigned long)Error);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, len);
    // Toggle LED3 to indicate error
    HAL_GPIO_TogglePin(GPIOE, LED3_Pin);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
