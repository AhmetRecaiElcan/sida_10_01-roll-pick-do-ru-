/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F407 GPS + Gyro MAVLink Integration
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mavlink/common/mavlink.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;  // GPS için UART

/* USER CODE BEGIN PV */
// GPS Verileri
typedef struct {
    uint8_t fix_type;
    float latitude;
    float longitude;
    float altitude;
    float ground_speed;
    float course;
    uint8_t satellites;
} GPS_Data_t;

GPS_Data_t gps_data = {0};

// GPS UART Buffer
#define GPS_BUFFER_SIZE 256
uint8_t gps_rx_buffer[GPS_BUFFER_SIZE];
uint8_t gps_rx_index = 0;
uint8_t gps_rx_byte;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void Parse_GPS_NMEA(uint8_t *buffer, uint16_t len);
void Extract_GPGGA(char *sentence);
void Extract_GPRMC(char *sentence);
/* USER CODE END PFP */

int main(void)
{
  /* USER CODE BEGIN 1 */
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  uint32_t last_send_time = 0;
  uint32_t last_heartbeat_time = 0;
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(100);

  // Gyro sensör başlatma
  uint8_t id_addr = 0x0F | 0x80;
  uint8_t id_val = 0;
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &id_addr, 1, 50);
  HAL_SPI_Receive(&hspi1, &id_val, 1, 50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  // Gyro sensör konfigürasyonu
  uint8_t reg_addr = 0x20;
  uint8_t reg_val = 0x67;
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 50);
  HAL_SPI_Transmit(&hspi1, &reg_val, 1, 50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  // GPS UART interrupt başlat
  HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);

  HAL_Delay(100);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // LED Toggle

    // GYRO VERİLERİNİ OKU (Sadece X ve Y - MPU9255 gelene kadar)
    uint8_t xl, xh, yl, yh;
    uint8_t addr_xl = 0x28 | 0x80;
    uint8_t addr_xh = 0x29 | 0x80;
    uint8_t addr_yl = 0x2A | 0x80;
    uint8_t addr_yh = 0x2B | 0x80;

    // X axis
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr_xl, 1, 50);
    HAL_SPI_Receive(&hspi1, &xl, 1, 50);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr_xh, 1, 50);
    HAL_SPI_Receive(&hspi1, &xh, 1, 50);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    // Y axis
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr_yl, 1, 50);
    HAL_SPI_Receive(&hspi1, &yl, 1, 50);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr_yh, 1, 50);
    HAL_SPI_Receive(&hspi1, &yh, 1, 50);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    // Gyro verilerini birleştir
    int16_t x_raw = (int16_t)((xh << 8) | xl);
    int16_t y_raw = (int16_t)((yh << 8) | yl);
    float roll = (float)x_raw * 0.0001f;
    float pitch = (float)y_raw * 0.0001f;
    float yaw = 0.0f;  // MPU9255 gelene kadar 0

    // HEARTBEAT gönder (1 saniyede bir)
    if (HAL_GetTick() - last_heartbeat_time >= 1000) {
      last_heartbeat_time = HAL_GetTick();

      mavlink_msg_heartbeat_pack(1, 200, &msg,
                                 MAV_TYPE_GENERIC,      // type
                                 MAV_AUTOPILOT_GENERIC, // autopilot
                                 MAV_MODE_PREFLIGHT,    // base_mode
                                 0,                     // custom_mode
                                 MAV_STATE_STANDBY);    // system_status
      len = mavlink_msg_to_send_buffer(buf, &msg);
      while (CDC_Transmit_FS(buf, len) == USBD_BUSY);
      HAL_Delay(10);
    }

    // 100ms'de bir MAVLink gönder
    if (HAL_GetTick() - last_send_time >= 100) {
      last_send_time = HAL_GetTick();

      // 1. ATTITUDE mesajı gönder (Gyro)
      mavlink_msg_attitude_pack(1, 200, &msg, HAL_GetTick(),
                                roll, pitch, yaw, 0, 0, 0);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      while (CDC_Transmit_FS(buf, len) == USBD_BUSY);

      HAL_Delay(10);

      // 2. GPS_RAW_INT mesajı gönder
      int32_t lat = (int32_t)(gps_data.latitude * 1e7);
      int32_t lon = (int32_t)(gps_data.longitude * 1e7);
      int32_t alt = (int32_t)(gps_data.altitude * 1000);
      uint16_t vel = (uint16_t)(gps_data.ground_speed * 100);
      uint16_t cog = (uint16_t)(gps_data.course * 100);

      mavlink_msg_gps_raw_int_pack(1, 200, &msg,
                                   HAL_GetTick() * 1000,  // time_usec
                                   gps_data.fix_type,      // fix_type
                                   lat, lon, alt,          // lat, lon, alt
                                   UINT16_MAX, UINT16_MAX, // eph, epv (accuracy)
                                   vel, cog,               // vel, cog
                                   gps_data.satellites,    // satellites_visible
                                   0,                      // alt_ellipsoid (MAVLink v2)
                                   0,                      // h_acc (MAVLink v2)
                                   0,                      // v_acc (MAVLink v2)
                                   0,                      // vel_acc (MAVLink v2)
                                   0,                      // hdg_acc (MAVLink v2)
                                   0);                     // yaw (MAVLink v2)
      len = mavlink_msg_to_send_buffer(buf, &msg);
      while (CDC_Transmit_FS(buf, len) == USBD_BUSY);

      HAL_Delay(10);

      // 3. GLOBAL_POSITION_INT mesajı gönder
      mavlink_msg_global_position_int_pack(1, 200, &msg,
                                           HAL_GetTick(),
                                           lat, lon, alt, alt,
                                           0, 0, 0,
                                           UINT16_MAX);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      while (CDC_Transmit_FS(buf, len) == USBD_BUSY);
    }

    HAL_Delay(10);
    /* USER CODE END WHILE */
  }
}

/* USER CODE BEGIN 4 */

// UART MSP Init - GPIO pinlerini USART2'ye ata
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(huart->Instance == USART2)
  {
    // USART2 Clock Enable
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // PA2 -> USART2_TX
    // PA3 -> USART2_RX
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // USART2 interrupt enable
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

// UART Interrupt Callback - GPS verileri için
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    // DEBUG: Ham GPS verisini USB'ye gönder
    CDC_Transmit_FS(&gps_rx_byte, 1);

    if (gps_rx_byte == '\n') {
      gps_rx_buffer[gps_rx_index] = '\0';
      Parse_GPS_NMEA(gps_rx_buffer, gps_rx_index);
      gps_rx_index = 0;
    } else if (gps_rx_index < GPS_BUFFER_SIZE - 1) {
      gps_rx_buffer[gps_rx_index++] = gps_rx_byte;
    }
    HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
  }
}

// NMEA cümlelerini parse et
void Parse_GPS_NMEA(uint8_t *buffer, uint16_t len)
{
  if (len < 6) return;

  char *sentence = (char *)buffer;

  if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
    Extract_GPGGA(sentence);
  } else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
    Extract_GPRMC(sentence);
  }
}

// GPGGA cümlesini parse et (konum, uydu sayısı, altitude)
void Extract_GPGGA(char *sentence)
{
  char *token;
  int field = 0;

  token = strtok(sentence, ",");
  while (token != NULL) {
    field++;
    switch (field) {
      case 3: // Latitude
        if (strlen(token) > 0) {
          float lat_deg = atof(token) / 100.0f;
          int deg = (int)lat_deg;
          float min = (lat_deg - deg) * 100.0f;
          gps_data.latitude = deg + (min / 60.0f);
        }
        break;
      case 4: // N/S
        if (token[0] == 'S') gps_data.latitude = -gps_data.latitude;
        break;
      case 5: // Longitude
        if (strlen(token) > 0) {
          float lon_deg = atof(token) / 100.0f;
          int deg = (int)lon_deg;
          float min = (lon_deg - deg) * 100.0f;
          gps_data.longitude = deg + (min / 60.0f);
        }
        break;
      case 6: // E/W
        if (token[0] == 'W') gps_data.longitude = -gps_data.longitude;
        break;
      case 7: // Fix quality (NMEA -> MAVLink dönüşümü)
        {
          uint8_t nmea_fix = atoi(token);
          // NMEA: 0=NoFix, 1=GPS, 2=DGPS, 4=RTK, 5=Float
          // MAVLink: 0=NoFix, 1=NoFix, 2=2D, 3=3D, 4=DGPS, 5=Float, 6=RTK
          if (nmea_fix == 0) gps_data.fix_type = 0;      // No Fix
          else if (nmea_fix == 1) gps_data.fix_type = 3; // GPS Fix -> 3D Fix
          else if (nmea_fix == 2) gps_data.fix_type = 4; // DGPS
          else if (nmea_fix == 4) gps_data.fix_type = 6; // RTK Fixed
          else if (nmea_fix == 5) gps_data.fix_type = 5; // RTK Float
          else gps_data.fix_type = 3; // Default: 3D Fix
        }
        break;
      case 8: // Satellites
        gps_data.satellites = atoi(token);
        break;
      case 10: // Altitude
        gps_data.altitude = atof(token);
        break;
    }
    token = strtok(NULL, ",");
  }
}

// GPRMC cümlesini parse et (hız, kurs)
void Extract_GPRMC(char *sentence)
{
  char *token;
  int field = 0;

  token = strtok(sentence, ",");
  while (token != NULL) {
    field++;
    switch (field) {
      case 8: // Ground speed (knots)
        gps_data.ground_speed = atof(token) * 0.514444f; // knots to m/s
        break;
      case 9: // Course (0-360 arası)
        {
          float course_raw = atof(token);
          // 0-360 aralığına al
          while (course_raw >= 360.0f) course_raw -= 360.0f;
          while (course_raw < 0.0f) course_raw += 360.0f;
          gps_data.course = course_raw;
        }
        break;
    }
    token = strtok(NULL, ",");
  }
}

/* USER CODE END 4 */

/**
  * @brief USART2 Initialization Function (GPS için)
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;  // M8N default baud rate
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
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2S3_Init(void)
{
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
