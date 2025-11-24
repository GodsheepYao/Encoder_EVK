/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
// Wrap value to range.
// With default rounding mode (round to nearest),
// the result will be in range -y/2 to y/2
inline float wrap_pm(float x, float y) {
#ifdef FPU_FPV4
    float intval = (float)round_int(x / y);
#else
    float intval = nearbyintf(x / y);
#endif
    return x - intval * y;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt / UART4 wake-up interrupt through EXTI line 34.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
    if(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&huart4,UART_IT_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        HAL_UART_DMAStop(&huart4);
        if (tamaga_encoder_data[0] == 0x02) {
            // 多摩川原始值
            encoder_abs = (((uint32_t)tamaga_encoder_data[4] << 16) | 
                                ((uint32_t)tamaga_encoder_data[3] << 8) | 
                                ((uint32_t)tamaga_encoder_data[2]));
            // 编码器端输出，单位：度
            encoder_elec_angle = wrap_pm((float)encoder_abs / 8388607.0f * 360.0f * motor_gear_ratio * motor_pole_pairs, 360.0f);
            encoder_rotor_pos = wrap_pm((float)encoder_abs / 8388607.0f * 360.0f * motor_gear_ratio, 360.0f);
            encoder_output_pos = wrap_pm((float)encoder_abs / 8388607.0f * 360.0f, 360.0f);
            
            for(uint8_t i = 0; i < 100; i++) {
                HAL_GPIO_WritePin(magnetic_spi_GPIO_Port, magnetic_spi_Pin, GPIO_PIN_SET);
            }
            HAL_GPIO_WritePin(magnetic_spi_GPIO_Port, magnetic_spi_Pin, GPIO_PIN_RESET);
            // 多摩川和正余弦的差值
            encoder_sincos_diff = utils_angle_difference(encoder_output_pos, sincos_output_pos);
        }
        HAL_UART_Receive_DMA(&huart4, tamaga_encoder_data, sizeof(tamaga_encoder_data));
    }
  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        // 定时器1更新回调，串口请求多摩川编码器数据
        HAL_UART_Transmit_DMA(&huart4, (uint8_t[]){0x02}, 1);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        const uint16_t sin_cnt = ADC_Value[1];
        const uint16_t cos_cnt = ADC_Value[0];
         // 归一化
        float mod_sin = (sin_cnt - s_offset_) * s_gain_;
        float mod_cos = (cos_cnt - c_offset_) * c_gain_;
        // 滤波
        static float sin_filter = 0.0f;
        static float cos_filter = 0.0f;
        UTILS_LP_FAST(sin_filter, mod_sin, 0.9f);
        UTILS_LP_FAST(cos_filter, mod_cos, 0.9f);
        // 正余弦上电偏置
        static float offset = 0.0f;
        // 反正切
        sincos_angle = utils_fast_atan2(sin_filter, cos_filter);
        static float last_angle = 0.0f;
        static bool initialized = false;
        if (!initialized) {
            offset = sincos_angle;
            last_angle = sincos_angle;
            initialized = true;
        }
        sincos_angle -= offset; 
        // 连续化多圈，增量累加，单位：电角度rad
        float diff = utils_angle_difference_rad(sincos_angle, last_angle);
        last_angle = sincos_angle;
        serial_angle += diff;
        // 正余弦输出，单位：度
        sincos_elec_angle = wrap_pm(sincos_angle / (float)(2.0f * M_PI) * 360.0f, 360.0f);
        sincos_rotor_pos = wrap_pm(serial_angle / (float)(2.0f * M_PI * motor_pole_pairs) * 360.0f, 360.0f);
        sincos_output_pos = wrap_pm(serial_angle / (float)(2.0f * M_PI * motor_pole_pairs * motor_gear_ratio) * 360.0f, 360.0f);
    }
}

float compensation_data[360] = {
    -0.048935f, -0.055968f, -0.087847f, -0.092103f, -0.099892f, -0.109428f,
    -0.109028f, -0.097980f, -0.098921f, -0.088409f, -0.107033f, -0.121985f,
    -0.104965f, -0.099396f, -0.114030f, -0.092357f, -0.080263f, -0.103401f,
    -0.112436f, -0.088663f, -0.103606f, -0.128423f, -0.104781f, -0.113971f,
    -0.154912f, -0.148892f, -0.146629f, -0.175168f, -0.171473f, -0.133594f,
    -0.166373f, -0.187923f, -0.155966f, -0.157752f, -0.191605f, -0.181053f,
    -0.133488f, -0.194848f, -0.198926f, -0.158182f, -0.190334f, -0.190506f,
    -0.156767f, -0.171431f, -0.179482f, -0.147677f, -0.140611f, -0.146444f,
    -0.132833f, -0.118547f, -0.138860f, -0.128060f, -0.105641f, -0.126068f,
    -0.120962f, -0.098608f, -0.122709f, -0.126781f, -0.093344f, -0.109220f,
    -0.108240f, -0.086613f, -0.086076f, -0.081010f, -0.056653f, -0.061949f,
    -0.063774f, -0.027305f, -0.043025f, -0.049979f, -0.027291f, -0.005165f,
    -0.012750f, -0.005124f, 0.015455f, 0.022141f, 0.026082f, 0.025282f,
    0.055728f, 0.088947f, 0.080390f, 0.118720f, 0.161121f, 0.141374f,
    0.138522f, 0.159716f, 0.181817f, 0.175705f, 0.157307f, 0.159747f,
    0.187112f, 0.156082f, 0.118940f, 0.143933f, 0.139635f, 0.101448f,
    0.132415f, 0.145696f, 0.116178f, 0.157299f, 0.182279f, 0.142298f,
    0.178388f, 0.213715f, 0.184554f, 0.196695f, 0.248823f, 0.232022f,
    0.227851f, 0.291968f, 0.284444f, 0.276954f, 0.354014f, 0.338146f,
    0.333642f, 0.395034f, 0.403494f, 0.358189f, 0.407673f, 0.441251f,
    0.397106f, 0.393026f, 0.432604f, 0.409258f, 0.410300f, 0.426870f,
    0.421436f, 0.434107f, 0.446745f, 0.423625f, 0.456458f, 0.486589f,
    0.465507f, 0.457175f, 0.518220f, 0.495985f, 0.473376f, 0.531032f,
    0.553701f, 0.501301f, 0.535914f, 0.593040f, 0.560451f, 0.547699f,
    0.630743f, 0.628455f, 0.580666f, 0.628402f, 0.659362f, 0.613739f,
    0.645715f, 0.682031f, 0.639963f, 0.615956f, 0.627241f, 0.633294f,
    0.598429f, 0.619510f, 0.651235f, 0.608691f, 0.594862f, 0.631356f,
    0.626731f, 0.586475f, 0.605730f, 0.628523f, 0.597914f, 0.587104f,
    0.616770f, 0.603167f, 0.574641f, 0.582682f, 0.588529f, 0.570209f,
    0.561516f, 0.592709f, 0.576754f, 0.549002f, 0.568591f, 0.563934f,
    0.538157f, 0.539321f, 0.541944f, 0.524738f, 0.511806f, 0.497317f,
    0.492598f, 0.476607f, 0.461853f, 0.450382f, 0.429891f, 0.419113f,
    0.433443f, 0.408589f, 0.404435f, 0.387523f, 0.401271f, 0.385872f,
    0.362277f, 0.365398f, 0.373406f, 0.356180f, 0.328722f, 0.337121f,
    0.363899f, 0.343178f, 0.341720f, 0.357613f, 0.332308f, 0.325271f,
    0.313891f, 0.323697f, 0.351990f, 0.336207f, 0.317946f, 0.357949f,
    0.352961f, 0.316921f, 0.309362f, 0.322900f, 0.328959f, 0.313899f,
    0.296541f, 0.305855f, 0.325019f, 0.300623f, 0.289124f, 0.302975f,
    0.319252f, 0.290447f, 0.268640f, 0.289474f, 0.285043f, 0.265873f,
    0.253205f, 0.270916f, 0.288154f, 0.256993f, 0.258343f, 0.280935f,
    0.285673f, 0.259611f, 0.271773f, 0.285151f, 0.284797f, 0.292369f,
    0.279467f, 0.291692f, 0.292644f, 0.271675f, 0.255179f, 0.278442f,
    0.262874f, 0.245511f, 0.255925f, 0.242139f, 0.222009f, 0.236199f,
    0.225791f, 0.198783f, 0.201867f, 0.184475f, 0.155342f, 0.139025f,
    0.128533f, 0.116843f, 0.099690f, 0.093195f, 0.067696f, 0.068792f,
    0.081285f, 0.071010f, 0.054519f, 0.105514f, 0.106140f, 0.075130f,
    0.134185f, 0.149438f, 0.128729f, 0.144547f, 0.158705f, 0.176139f,
    0.163688f, 0.151709f, 0.159530f, 0.163283f, 0.142005f, 0.126325f,
    0.142094f, 0.142351f, 0.105395f, 0.105308f, 0.128853f, 0.115920f,
    0.059360f, 0.097825f, 0.122241f, 0.063849f, 0.039134f, 0.081334f,
    0.066484f, 0.023355f, 0.060516f, 0.085526f, 0.037517f, 0.055354f,
    0.107156f, 0.084403f, 0.070390f, 0.125820f, 0.148119f, 0.119252f,
    0.149908f, 0.185987f, 0.152595f, 0.152770f, 0.201201f, 0.182630f,
    0.144401f, 0.182698f, 0.210222f, 0.152005f, 0.128220f, 0.188534f,
    0.176131f, 0.145396f, 0.185898f, 0.190498f, 0.136365f, 0.130306f,
    0.175246f, 0.145707f, 0.114420f, 0.139909f, 0.148782f, 0.112734f,
    0.119533f, 0.144540f, 0.121715f, 0.114017f, 0.132748f, 0.133661f,
    0.125832f, 0.139069f, 0.129030f, 0.121481f, 0.111965f, 0.105674f,
    0.099905f, 0.096346f, 0.085155f, 0.045527f, 0.050486f, 0.062822f,
    0.033000f, 0.027274f, 0.039881f, 0.006299f, -0.017423f, -0.042722f
};

float filter_diff = 0;
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        // 磁编原始值
        magnetic_abs = (spi_rx_buffer[0] << 8) | (spi_rx_buffer[1] >> 8);
        // 磁编上电偏置
        static float offset = 0.0f;
        static bool initialized = false;
        if (!initialized) {
            offset = magnetic_abs;
            initialized = true;
        }
        magnetic_abs -= offset;
        // 找到当前磁编码器对应的补偿值
        int index = (int)((float)magnetic_abs / 65535.0f * 360); // 计算对应补偿数据的索引
        if (index >= 360) index = 359;  // 确保索引不越界
        // 磁编输出，单位：度
        magnetic_elec_angle = wrap_pm((float)magnetic_abs / 65535.0f * 360.0f * motor_gear_ratio * motor_pole_pairs, 360.0f);
        magnetic_rotor_pos = wrap_pm((float)magnetic_abs / 65535.0f * 360.0f * motor_gear_ratio, 360.0f);
        magnetic_output_pos = wrap_pm((float)magnetic_abs / 65535.0f * 360.0f /* + compensation_data[index] */, 360.0f);
        
        for(uint8_t i = 0; i < 100; i++) {
            HAL_GPIO_WritePin(magnetic_spi_GPIO_Port, magnetic_spi_Pin, GPIO_PIN_SET);
        }
        HAL_GPIO_WritePin(magnetic_spi_GPIO_Port, magnetic_spi_Pin, GPIO_PIN_RESET);
        // 和磁编的差值
        encoder_magnetic_diff = utils_angle_difference(encoder_output_pos, magnetic_output_pos);
        sincos_magnetic_diff = utils_angle_difference(sincos_output_pos, magnetic_output_pos);
        
        UTILS_LP_FAST(filter_diff, encoder_magnetic_diff, 0.005f);
    }
}
/* USER CODE END 1 */
