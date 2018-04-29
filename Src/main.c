
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether 
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"
#include "ys.h"

static void LL_Init(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static inline void blip(uint32_t ms);


//#define FIR_TAPS 74
#define FIR_TAPS 104
#define FIR_STATE_SIZE FIR_TAPS+FILTER_BLOCK_SIZE-1

#define IIR_STAGES 5
#define IIR_COEFFS 5*IIR_STAGES
#define IIR_STATE_SIZE 4*IIR_STAGES
#define BIQUAD_POST_SHIFT 1


arm_fir_instance_f32 fir;
arm_fir_instance_q31 firFixed;
arm_biquad_cascade_df2T_instance_f32 iirFloat;
//arm_biquad_casd_df1_inst_f32 iirFloat;
arm_biquad_cas_df1_32x64_ins_q31 iirFixed;
/* pState is of length numTaps+blockSize-1 samples */
float firState[FIR_STATE_SIZE];
float iirState[IIR_STATE_SIZE];
q63_t iirStateFixed[IIR_STATE_SIZE];

q31_t filterInputFixed[FILTER_BLOCK_SIZE];

float filterOutput[FILTER_BLOCK_SIZE];

extern float *const firCoeffs;
q31_t firCoeffsFixed[FIR_TAPS-1];

extern float iirCoeffsBy2[IIR_COEFFS];
extern float iirCoeffs[IIR_COEFFS];
extern q31_t iirCoeffsFixed[];

volatile uint32_t cycles;


int main(void)
{
    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    //MX_USART2_UART_Init();



    // Enable DWT
    CoreDebug->DEMCR |= 0x01000000;
    DWT->CYCCNT = 0;
    DWT->CTRL |= 1;

    LL_GPIO_ResetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
    
    /* f32 FIR */
    arm_fir_init_f32(&fir, FIR_TAPS, firCoeffs, firState, FILTER_BLOCK_SIZE);

    while (LL_GPIO_IsInputPinSet(USER_BTN_GPIO_Port, USER_BTN_GPIO_Pin));
    LL_GPIO_SetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
    LL_mDelay(10);
    DWT->CYCCNT = 0;
    arm_fir_f32(&fir, (float *)filterInput, filterOutput, FILTER_BLOCK_SIZE);
    cycles = DWT->CYCCNT;
    LL_GPIO_ResetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
    LL_mDelay(500);

    /* q31 FIR */
    arm_float_to_q31(firCoeffs, firCoeffsFixed, FIR_TAPS-1);
    arm_float_to_q31((float *)filterInput, filterInputFixed, FILTER_BLOCK_SIZE);

    arm_fir_init_q31(&firFixed, FIR_TAPS, firCoeffsFixed, (q31_t *)firState,
                     FILTER_BLOCK_SIZE);

    while (LL_GPIO_IsInputPinSet(USER_BTN_GPIO_Port, USER_BTN_GPIO_Pin));
    LL_GPIO_SetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
    LL_mDelay(10);
    DWT->CYCCNT = 0;
    arm_fir_q31(&firFixed, filterInputFixed, (q31_t *)filterOutput,
                FILTER_BLOCK_SIZE);
    cycles = DWT->CYCCNT;
    LL_GPIO_ResetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
    LL_mDelay(500);

    arm_q31_to_float((q31_t *)filterOutput, filterOutput, FILTER_BLOCK_SIZE);



    /* f32 IIR */
    arm_biquad_cascade_df2T_init_f32(&iirFloat, IIR_STAGES, iirCoeffs,
                                     iirState);
    //arm_biquad_cascade_df1_init_f32(&iirFloat, IIR_STAGES, iirCoeffs,
    //                                iirState);
    while (LL_GPIO_IsInputPinSet(USER_BTN_GPIO_Port, USER_BTN_GPIO_Pin));
    LL_GPIO_SetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
    LL_mDelay(10);
    DWT->CYCCNT = 0;
    arm_biquad_cascade_df2T_f32(&iirFloat, filterInput, filterOutput,
                                 FILTER_BLOCK_SIZE);
    cycles = DWT->CYCCNT;
    //arm_biquad_cascade_df1_f32(&iirFloat, filterInput, filterOutput,
    //                             FILTER_BLOCK_SIZE);
    LL_GPIO_ResetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
    LL_mDelay(500);




    arm_float_to_q31(iirCoeffsBy2, iirCoeffsFixed, IIR_COEFFS);

    /* q31 IIR */
    arm_biquad_cas_df1_32x64_init_q31(&iirFixed, IIR_STAGES, iirCoeffsFixed,
                                      iirStateFixed, BIQUAD_POST_SHIFT);


    while (LL_GPIO_IsInputPinSet(USER_BTN_GPIO_Port, USER_BTN_GPIO_Pin));
    LL_GPIO_SetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
    LL_mDelay(10);
    DWT->CYCCNT = 0;
    arm_biquad_cas_df1_32x64_q31(&iirFixed, filterInputFixed,
                                 (q31_t *)filterOutput, FILTER_BLOCK_SIZE);
    cycles = DWT->CYCCNT;
    LL_GPIO_ResetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
    LL_mDelay(500);

    arm_q31_to_float((q31_t *)filterOutput, filterOutput, FILTER_BLOCK_SIZE);


    DWT->CTRL &= ~1;

    /* Infinite loop */
    while (1) {

        //LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        LL_mDelay(10);

        if (!LL_GPIO_IsInputPinSet(USER_BTN_GPIO_Port, USER_BTN_GPIO_Pin)) {
            LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
            LL_GPIO_SetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
        }
        else {
            LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
            LL_GPIO_ResetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);
        }

    }
    /* USER CODE END 3 */

}

static void LL_Init(void)
{


    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* BusFault_IRQn interrupt configuration */
    NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* UsageFault_IRQn interrupt configuration */
    NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* SVCall_IRQn interrupt configuration */
    NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* DebugMonitor_IRQn interrupt configuration */
    NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* PendSV_IRQn interrupt configuration */
    NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
    {
        Error_Handler();  
    }
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    LL_RCC_MSI_Enable();

    /* Wait till MSI is ready */
    while(LL_RCC_MSI_IsReady() != 1)
    {

    }
    LL_RCC_MSI_EnablePLLMode();

    LL_RCC_MSI_EnableRangeSelection();

    LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);

    LL_RCC_MSI_SetCalibTrimming(0);

    LL_PWR_EnableBkUpAccess();

    LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);

    LL_RCC_LSE_Enable();

    /* Wait till LSE is ready */
    while(LL_RCC_LSE_IsReady() != 1)
    {

    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 16, LL_RCC_PLLR_DIV_2);

    LL_RCC_PLL_EnableDomain_SYS();

    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while(LL_RCC_PLL_IsReady() != 1)
    {

    }
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {

    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    LL_Init1msTick(32000000);

    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

    LL_SetSystemCoreClock(32000000);

    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

    LL_USART_InitTypeDef USART_InitStruct;

    LL_GPIO_InitTypeDef GPIO_InitStruct;

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    /**USART2 GPIO Configuration  
      PA2   ------> USART2_TX
      PA15 (JTDI)   ------> USART2_RX 
      */
    GPIO_InitStruct.Pin = VCP_TX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = VCP_RX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
    LL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_7B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &USART_InitStruct);

    LL_USART_ConfigAsyncMode(USART2);

    LL_USART_Enable(USART2);

}

/** Configure pins as 
 * Analog 
 * Input 
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

    LL_GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    /* LED */
    LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);

    GPIO_InitStruct.Pin = LD3_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

    /* Power measurement trigger */
    LL_GPIO_ResetOutputPin(D7_GPIO_Port, D7_GPIO_Pin);

    GPIO_InitStruct.Pin = D7_GPIO_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(D7_GPIO_Port, &GPIO_InitStruct);

    /* User button */
    GPIO_InitStruct.Pin = USER_BTN_GPIO_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

}


static inline void blip(uint32_t ms) {
    LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
    LL_mDelay(ms);
    LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
