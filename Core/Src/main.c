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
#include "app_fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdint.h"
#include "stdio.h"
#include "font6x8.h"

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

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

volatile uint8_t conditionMet = 0;
uint8_t scd41_addr = 0x62 << 1; //7 bit address shifted left
uint8_t ssd1306_addr = 0x3C << 1; //7 bit address shifted left

uint16_t co2 = 0; //co2 in ppm
float temp; // temp in F
float rh; //rh in %

FATFS   sd_fs;          // FatFs work area
FIL     log_file;       // open file handle
UINT    bw;             // bytes‑written counter

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void scd41_start(void);
uint8_t scd41_read(uint16_t *co2, float *temp, float *rh);

void ssd1306_start(void);
void ssd1306_fill(uint8_t pattern);
void ssd1306_print(uint8_t x, uint8_t page, const char *str);  

void SD_start(void);
void SD_append(uint32_t ms, uint16_t co2, float t_f, float rh);


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
  MX_I2C1_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  scd41_start();
  ssd1306_start();
  SD_start();
  HAL_Delay(5000);  // warm-up delay
  
  // init variables for display and writing
  char line[22];

  while (1)
  {
    ssd1306_fill(0x00);  // clear display
    scd41_read(&co2, &temp, &rh);
    uint32_t now_ms = HAL_GetTick(); 

    SD_append(now_ms, co2, temp, rh);
    sprintf(line, "CO2: %4d ppm", co2);
    ssd1306_print(0, 0, line);
  
    sprintf(line, "T: %2.1f F", temp);
    ssd1306_print(0, 1, line);
  
    sprintf(line, "RH: %2.1f %%", rh);
    ssd1306_print(0, 2, line);

    HAL_Delay(5000);   

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//start periodic measurement (command 0x21B1)
void scd41_start(void){
  uint8_t cmd[2] = {0x21, 0xB1}; //split command into 2 bytes
  HAL_I2C_Master_Transmit(&hi2c1, scd41_addr, cmd, 2, HAL_MAX_DELAY);
}

//read measurement (command 0xEC05)
uint8_t scd41_read(uint16_t *co2, float *temp, float *rh){
  uint8_t cmd[2] = {0xEC, 0x05};
  uint8_t rx[9];

  HAL_I2C_Master_Transmit(&hi2c1, scd41_addr, cmd, 2, HAL_MAX_DELAY);
  HAL_Delay(1);

  if (HAL_I2C_Master_Receive(&hi2c1, scd41_addr, rx, 9, HAL_MAX_DELAY) != HAL_OK)
    return 0;

  *co2 = (rx[0] << 8) | rx[1];
  uint16_t temp_raw = (rx[3] << 8) | rx[4];
  float temp_c = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
  *temp = temp_c * 9.0f / 5.0f + 32.0f;

  uint16_t rh_raw = (rx[6] << 8) | rx[7];
  *rh = 100.0f * ((float)rh_raw / 65535.0f);

  return 1;
}

//initialize ssd1306
void ssd1306_start(void){
  uint8_t cmds[] = { //initialization commands
    0xAE,       // Display OFF
    0xD5, 0x80, // Set display clock divide ratio/oscillator freq
    0xA8, 0x3F, // Set multiplex ratio (1 to 64)
    0xD3, 0x00, // Display offset
    0x40,       // Start line address
    0x8D, 0x14, // Charge pump ON
    0x20, 0x00, // Memory addressing mode: horizontal
    0xA1,       // Segment re-map (mirror horizontally)
    0xC8,       // COM Output Scan Direction
    0xDA, 0x12, // COM pins hardware config
    0x81, 0x7F, // Contrast
    0xD9, 0xF1, // Pre-charge period
    0xDB, 0x40, // VCOMH deselect level
    0xA4,       // Entire display ON resume
    0xA6,       // Normal (not inverted)
    0xAF        // Display ON
  };

    for (uint8_t i = 0; i < sizeof(cmds); i++) {
      HAL_I2C_Mem_Write(&hi2c1, ssd1306_addr, 0x00, 1, &cmds[i], 1, HAL_MAX_DELAY);
    }
}

void ssd1306_fill(uint8_t pattern) {
  uint8_t data[128];
  for (int i = 0; i < 128; i++) data[i] = pattern;

  for (uint8_t page = 0; page < 8; page++) {
    uint8_t set_page[] = {
      0xB0 | page, // Set page (0–7)
      0x00,        // Low column
      0x10         // High column
    };

    for (uint8_t i = 0; i < 3; i++) {
      HAL_I2C_Mem_Write(&hi2c1, ssd1306_addr, 0x00, 1, &set_page[i], 1, HAL_MAX_DELAY);
    }

    // Now send 128 bytes of pixel data
    HAL_I2C_Mem_Write(&hi2c1, ssd1306_addr, 0x40, 1, data, 128, HAL_MAX_DELAY);
  }
}

extern const uint8_t font6x8[];

void ssd1306_draw_char(uint8_t x, uint8_t page, char c) {
  if (c < 32 || c > 127) c = '?';  // fallback
  uint8_t *glyph = (uint8_t *)&font6x8[(c - 32) * 6];
  HAL_I2C_Mem_Write(&hi2c1, ssd1306_addr, 0x00, 1, (uint8_t[]){0xB0 | page, 0x00 | (x & 0x0F), 0x10 | (x >> 4)}, 3, HAL_MAX_DELAY);
  HAL_I2C_Mem_Write(&hi2c1, ssd1306_addr, 0x40, 1, glyph, 6, HAL_MAX_DELAY);
}

void ssd1306_print(uint8_t x, uint8_t page, const char *str) {
  while (*str) {
    ssd1306_draw_char(x, page, *str++);
    x += 6;
  }
}

void SD_start(void)
{
    /* 1. mount the card (auto mount = 1) */
    if (f_mount(&sd_fs, "", 1) != FR_OK)   Error_Handler();

    /* 2. open/append or create */
    if (f_open(&log_file, "log.csv",
               FA_WRITE | FA_OPEN_APPEND) != FR_OK)
        Error_Handler();

    /* 3. if new file, write a header */
    if (f_size(&log_file) == 0) {
        f_write(&log_file, "time_ms,co2,temp_F,rh\r\n", 24, &bw);
        f_sync(&log_file);                 /* flush header to card   */
    }
}

void SD_append(uint32_t ms, uint16_t co2, float t_f, float rh)
{
    char line[48];
    int len = snprintf(line, sizeof(line),
                       "%lu,%u,%.1f,%.1f\r\n",
                       (unsigned long)ms, co2, t_f, rh);

    if (f_write(&log_file, line, len, &bw) != FR_OK)  Error_Handler();
    if (f_sync(&log_file)                      != FR_OK)  Error_Handler();
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
