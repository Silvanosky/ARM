/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mbedtls.h"
#include "UART.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <time.h>
#include "mbedtls/memory_buffer_alloc.h"
#include "mbedtls/platform.h"
#include "mbedtls/md_internal.h"
#include "mbedtls/hmac_drbg.h"
#include "mbedtls/rsa.h"
#define MBEDTLS_MEMORY_VERIFY_ALLOC (1 << 0)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
mbedtls_hmac_drbg_context cont;
mbedtls_rsa_context rsa_cont;

/* USER CODE BEGIN PV */


static bool received = false;
unsigned strlen(const unsigned char* str){
  unsigned size = 0;
  for (; str[size] != '\0'; ++size);
  return size;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    received = true;
  }
}

void UART_SEND(char *str) {
  HAL_UART_Transmit (&huart1, (unsigned char *)str,
      strlen((unsigned char *)str), 300);
}

void UART_SEND_LEN(const char *str, unsigned length) {
  HAL_UART_Transmit (&huart1, (unsigned char *)str,
      length, 300);
}

void UART_RECEIVE(unsigned char* buffer, size_t buffer_len, char* instruction) {
  HAL_StatusTypeDef state;
  while ((state = HAL_UART_Receive (&huart1, buffer, buffer_len, 300)) != HAL_OK) {
    if (instruction) {
      UART_SEND(instruction);
    } else {
      if (state == HAL_BUSY)
        UART_SEND("errorBUSY\n");
      else if (state == HAL_TIMEOUT)
        UART_SEND("errorTIMEOUT\n");
      else if (state == HAL_ERROR)
        UART_SEND("errorERROR\n");
    }
  }
  UART_SEND("FINISHED\n");
}
void myExit(int status) {
  if (status == 1)
    UART_SEND("exit with error code 1\n");
  else if (!status) 
    UART_SEND("exit with normal error code\n");
  else
    UART_SEND("exit with random error code\n");
  return;
}

int mysnprintf(char* str, size_t n, const char* format, ...) {
  UART_SEND("snprintf called\n");
  return 0;
}

char convertToHexa(uint32_t u) {
  if (u <= 9)
    return u + '0';
  else if (u <= 15)
    return u - 10 + 'A';
  else {
    UART_SEND("tu code comme un pied");
    return '0';
  }
}


unsigned char val = 0;
void print_mpi_UART(mbedtls_mpi *mpi) {
  size_t n = mpi->n * 8 + 2;
  char buffer[n];
  size_t buffer_index = 0;


  for (size_t index = 0; index < mpi->n; index++) {
    uint32_t val = mpi->p[index];

    for (unsigned shift = 0; shift < 32; shift+=4) {
      buffer[buffer_index++] = convertToHexa( (val << shift) >> 28);
    }

  }

  for (; buffer_index < n; buffer_index++)
    buffer[buffer_index++] = '\0';

  UART_SEND(buffer);
}
#define DEBUG_MODE


bool keyGenerated = false;
void genKey() {
  const char *personalization = "dfajenFNXOmdfjacnI>ndfN";

  const mbedtls_md_info_t* md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

  HAL_Delay(1000);
  int error = mbedtls_hmac_drbg_seed_buf(&cont,
                                    md_info,
                                    (unsigned char *) personalization,
                                    strlen((const unsigned char*)personalization));
  //int error = mbedtls_ctr_drbg_seed(&rng_context, myfun, NULL,
  //    (unsigned char *) personalization, size);

  if (error != 0) {
    if (error == MBEDTLS_ERR_MD_BAD_INPUT_DATA) {
      UART_SEND("error1\n");
    }
    else if (error == MBEDTLS_ERR_MD_ALLOC_FAILED) {

      UART_SEND("error2\n");
    }
    else {
      UART_SEND("eror?\n");
    }

  }
  HAL_Delay(1000);
  error = mbedtls_rsa_gen_key(&rsa_cont, mbedtls_hmac_drbg_random, &cont, 1024, 65537);
  if (!error) {
    keyGenerated = true;
  } else {
    UART_SEND("rsa generation failed\n");
  }
}
void sendPublicKey() {
  if (!keyGenerated) {
    UART_SEND("error no private key generated\n");
    return;
  }
  print_mpi_UART(&rsa_cont.N);
  print_mpi_UART(&rsa_cont.E);
  UART_SEND("\n");
}

#define SHA256_SIZE 64
void signSHA256() {
  unsigned char sha256[SHA256_SIZE];
  unsigned char signedSHA[500];
  int error;

  UART_RECEIVE(sha256, SHA256_SIZE, NULL);

  HAL_Delay(100);
  error = mbedtls_rsa_rsassa_pkcs1_v15_sign(&rsa_cont, mbedtls_hmac_drbg_random, &cont,
      MBEDTLS_RSA_PRIVATE, MBEDTLS_MD_SHA256, SHA256_SIZE, sha256, signedSHA);

  HAL_Delay(100);
  if (error) {

    if (error == MBEDTLS_ERR_RSA_BAD_INPUT_DATA)
      UART_SEND("error: signature failed input\n");
    else
      UART_SEND("error: signature failed\n");

  } else {
    HAL_UART_Transmit (&huart1, signedSHA, sizeof(signedSHA) , 300);
    UART_SEND("\n");
  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
//static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

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
  /* useR CODE BEGIN 1 */
//allows for stack allocation
#define MEM_BUFF_SIZE 10000
  unsigned char memory_buffer[MEM_BUFF_SIZE];
  mbedtls_memory_buffer_alloc_init(memory_buffer, MEM_BUFF_SIZE);

  mbedtls_platform_set_exit(myExit);
  mbedtls_platform_set_snprintf(mysnprintf);

#define PWD_SIZE 50
  unsigned char passwd[PWD_SIZE];
  unsigned char passwd2[PWD_SIZE];
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  mbedtls_hmac_drbg_init(&cont);
  mbedtls_rsa_init(&rsa_cont, MBEDTLS_RSA_PKCS_V15, 0);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  //MX_IWDG_Init();
  MX_MBEDTLS_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  genKey();
  UART_RECEIVE(passwd, PWD_SIZE, "PASS\n");
  

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    __WFE ();
    unsigned char cmd;
    bool authorized = true;
    UART_RECEIVE(&cmd, 1, NULL);


    if (cmd == '1') {
      UART_SEND("errorTIMEOUT\n");
      UART_RECEIVE(passwd2, PWD_SIZE, "PASS\n");
      for (unsigned k = 0; k < PWD_SIZE; ++k) {
        if (passwd[k] != passwd2[k])
          authorized = false;
      }
      if (!authorized) {
        UART_SEND("ACCESSDENIED\n");
        continue;
      }
      UART_SEND("ACCESSGRANTED\n");
      sendPublicKey();


    } else if (cmd == '2') {
      UART_SEND("errorTIMEOUT\n");
      UART_RECEIVE(passwd2, PWD_SIZE, "PASS\n");
      for (unsigned k = 0; k < PWD_SIZE; ++k) {
        if (passwd[k] != passwd2[k])
          authorized = false;
      }
      if (!authorized) {
        UART_SEND("ACCESSDENIED\n");
        continue;
      }
      UART_SEND("ACCESSGRANTED\n");
      signSHA256();


    } else if (cmd == '3') {
      // write flash
      UART_SEND("goodbye");
    }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
/*
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}*/ 

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
