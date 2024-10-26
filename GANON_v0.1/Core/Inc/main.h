/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  /* USER CODE END Header */

  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

#include "drivers/BMI088.h"
#include "drivers/BMP388.h"
#include "drivers/buzzer.h"
#include "drivers/gps.h"
#include "drivers/rfm96w.h"
#include "drivers/w25q_mem.h"

#include "scheduler.h"
#include "tools.h"
#include "flash_stream.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


typedef struct COMPONENTS {
    W25Q_Chip* flash;
    RFM96_Chip* lora;
    BMI088* imu;
    BMP388_HandleTypeDef* bmp;
    GPS_t* gps;
    BUZZER* buzzer;
} COMPONENTS;

typedef struct DATAS {
    // Sensors
    BMI088* imu;
    BMP388_HandleTypeDef* bmp;
    GPS_t* gps;

    // Datas
    float accel[3];     // [m/s^2]
    float gyro[3];      // [rad/s]
    // float mag[3];       // ???
    float temp;         // [C]
    float pressure;     // [Pa]
    float longitude;    // [deg]
    float latitude;     // [deg]
    float time;         // [ms]
} DATAS;

typedef enum MACHINE_STATE {
    // ANALYSING_INFO,

    // WAIT_FOR_GPS_FIX,

    // SEND_GONOGO,
    // WAIT_RECIEVE_GONOGO,
    // RECIEVE_GONOGO,

    // BEGIN_FLASH_ERASING,
    // FLASH_ERASING,
    // END_FLASH_ERASING,

    // GETTING_DATA,

    // WAIT_FOR_SENDING_DATA, // <-- mode pour attendre et lire directement les données
    // BEGIN_SENDING_DATA,
    // SENDING_DATA,
    // END_SENDING_DATA,

    START,
    WAIT_USB,
    USB_READY,
    TEST_1,
    TEST_2,
    TEST_3,
    TEST_4,

    // IS_STOPPED,
    // STOP,

    DEFAULT_STATE = 0xff,
} MACHINE_STATE;

typedef struct MACHINE {
    MACHINE_STATE  state;

    COMPONENTS* components;
    FLASH_STREAM* flash_stream;
    DATAS* datas;

    char           tx_buff[256];
    uint8_t        usb_watchdog_bfr[3];

    uint32_t       last_time;
    uint32_t       last_tick;
} MACHINE;


typedef struct ASYNC_test_usb_CONTEXT {
    char* buff;
    int len;
    int index;
    uint32_t delay;
    uint32_t next_time;
} ASYNC_test_usb_CONTEXT;

typedef struct ASYNC_update_usb_watchdog_CONTEXT {
    MACHINE_STATE* state;
    uint8_t usb_watchdog_bfr[3];
    uint32_t next_time;
} ASYNC_update_usb_watchdog_CONTEXT;

typedef struct ASYNC_send_usb_CONTEXT {
    uint32_t delay;
    uint32_t next_time;
} ASYNC_send_usb_CONTEXT;

typedef struct ASYNC_update_IMU_CONTEXT {
    bool      continue_update;
    uint32_t  delay;
    uint32_t  next_time;
} ASYNC_update_IMU_CONTEXT;

typedef enum ASYNC_test_w25q_1_STATE {
    ASYNC_test_w25q_1_START,
    ASYNC_test_w25q_1_WAIT_TX,
    ASYNC_test_w25q_1_DONE_TX,
    ASYNC_test_w25q_1_WAIT_RX,
    ASYNC_test_w25q_1_DONE_RX
} ASYNC_test_w25q_1_STATE;

typedef enum ASYNC_test_fs_STATE {
    ASYNC_test_fs_START,
    ASYNC_test_fs_START_TX,
    ASYNC_test_fs_WAIT_TX,
    ASYNC_test_fs_DONE_TX,
    ASYNC_test_fs_WAIT_RX,
    ASYNC_test_fs_DONE_RX
} ASYNC_test_fs_STATE;

typedef struct ASYNC_test_w25q_1_CONTEXT {
    W25Q_Chip* flash_chip;
    uint8_t tx_buffer[12];
    uint8_t rx_buffer[12];
    bool is_done;

    ASYNC_test_w25q_1_STATE state;
} ASYNC_test_w25q_1_CONTEXT;

typedef struct ASYNC_test_fs_CONTEXT {
    FLASH_STREAM* flash_stream;
    float *tx_buffer; // 12 bytes allocated 
    float *rx_buffer; // 12 bytes allocated
    bool is_done;

    uint8_t idx;
    ASYNC_test_fs_STATE state;
} ASYNC_test_fs_CONTEXT;

typedef enum ASYNC_save_IMU_STATE {
    ASYNC_save_IMU_START_ERASE,
    ASYNC_save_IMU_WAIT_ERASE_TASK,
    ASYNC_save_IMU_WAIT_ERASE_DONE,
    ASYNC_save_IMU_START_UPDATE,
    ASYNC_save_IMU_WAIT_UPDATE,
    ASYNC_save_IMU_WAIT_SAVE,
    ASYNC_save_IMU_END_UPDATE,
    ASYNC_save_IMU_START_LOAD,
    ASYNC_save_IMU_WAIT_LOAD,
    ASYNC_save_IMU_START_SEND,
    ASYNC_save_IMU_WAIT_SEND,
    ASYNC_save_IMU_END_SEND,
} ASYNC_save_IMU_STATE;

typedef struct ASYNC_save_IMU_CONTEXT {
    TASK *task_imu;
    float datas_buff[7];

    bool ready;
    bool save_done;

    ASYNC_save_IMU_STATE state;
    uint32_t max_delay;
    uint32_t imu_delay;
    uint32_t first_time;
    uint32_t next_time;
} ASYNC_save_IMU_CONTEXT;

typedef enum ASYNC_test_w25q_2_STATE {
    ASYNC_test_w25q_2_START,
    ASYNC_test_w25q_2_WAIT,
    ASYNC_test_w25q_2_END,
} ASYNC_test_w25q_2_STATE;

typedef struct ASYNC_test_w25q_2_CONTEXT {
    W25Q_Chip *flash_chip;

    uint8_t *tx_buffer;
    uint8_t *rx_buffer;

    bool is_done;

    ASYNC_test_w25q_2_STATE state;
} ASYNC_test_w25q_2_CONTEXT;

extern SONG_BANK song_bank;
extern MACHINE machine;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */



void init_components(COMPONENTS* components,
                        W25Q_Chip* flash_chip,
                        RFM96_Chip* lora_chip,
                        BMI088* imu,
                        BMP388_HandleTypeDef* bmp,
                        GPS_t* gps,
                        BUZZER* buzzer);

void init_all_components(COMPONENTS* components);

// ===========================================

void init_machine(MACHINE* machine, COMPONENTS* components,
                    FLASH_STREAM* flash_stream, DATAS* datas);

void run_state_machine(MACHINE* machine);


void analysing_info(MACHINE* machine);

void wait_for_gps_fix(MACHINE* machine);

void send_gonon(MACHINE* machine);
void wait_recieve_gonogo(MACHINE* machine);
void recieve_gonogo(MACHINE* machine);

void begin_flash_erasing(MACHINE* machine);
void flash_erasing(MACHINE* machine);
void end_flash_erasing(MACHINE* machine);

void getting_data(MACHINE* machine);

void wait_for_sending_data(MACHINE* machine);
void begin_sending_data(MACHINE* machine);
void sending_data(MACHINE* machine);
void end_sending_data(MACHINE* machine);

void is_stopped(MACHINE* machine);

void test1(MACHINE* machine);

void lora_wait(MACHINE* machine);

void add_task_if_flag(SCHEDULER* scheduler, MACHINE* machine);

// ===========================================

void update_usb_watchdog(uint8_t* usb_watchdog_bfr);
bool is_connected_to_usb(uint8_t* usb_watchdog_bfr);

void ASYNC_update_usb_watchdog_init(TASK *task, MACHINE* machine);
void ASYNC_update_usb_watchdog(SCHEDULER* scheduler, TASK* self);

// ===========================================

void flash_stream_find_last_info_ptr(FLASH_STREAM* stream);
void save_data_to_flash(FLASH_STREAM* stream, DATAS datas);
void save_info_to_flash(FLASH_STREAM* stream, MACHINE_STATE state);
void load_info_from_flash(FLASH_STREAM* stream, uint32_t* last_read_ptr,
                            uint32_t* last_write_ptr, MACHINE_STATE* last_state);

// =============================================

void Datas_Init(DATAS* datas, BMI088* imu, BMP388_HandleTypeDef* bmp, GPS_t* gps);
void load_all_datas_from_sensors(DATAS* datas);
void load_all_datas_from_flash(DATAS* datas, FLASH_STREAM* stream);

// =============================================

void get_csv_header(char *buff);
void get_csv_line(DATAS datas, char *buff);

void ASYNC_test_usb_init(TASK *task, char *buff, int len, uint32_t delay);
void ASYNC_test_usb(SCHEDULER *scheduler, TASK *self);

void ASYNC_send_usb_init(TASK *task, uint32_t delay);
void ASYNC_send_usb(SCHEDULER *scheduler, TASK *self);

void ASYNC_update_IMU_init(TASK *task, uint32_t delay);
void ASYNC_update_IMU(SCHEDULER *scheduler, TASK *self);

void ASYNC_test_w25q_1_init(TASK *task, W25Q_Chip *flash_chip);
void ASYNC_test_w25q_1(SCHEDULER *scheduler, TASK *self);

void ASYNC_test_fs_init(TASK *task, FLASH_STREAM *flash_stream);
void ASYNC_test_fs(SCHEDULER *scheduler, TASK *self);

void ASYNC_save_IMU_init(TASK *task);
void ASYNC_save_IMU(SCHEDULER *scheduler, TASK *self);

void ASYNC_test_w25q_2_init(TASK *task, W25Q_Chip *flash_chip);
void ASYNC_test_w25q_2(SCHEDULER *scheduler, TASK *self);

// =============================================

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define N_CS_ACC_Pin GPIO_PIN_0
#define N_CS_ACC_GPIO_Port GPIOA
#define N_CS_GYRO_Pin GPIO_PIN_1
#define N_CS_GYRO_GPIO_Port GPIOA
#define N_CS_LORA_Pin GPIO_PIN_2
#define N_CS_LORA_GPIO_Port GPIOA
#define N_CS_FLASH_Pin GPIO_PIN_3
#define N_CS_FLASH_GPIO_Port GPIOA
#define N_RST_LORA_Pin GPIO_PIN_10
#define N_RST_LORA_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define N_RST_GPS_Pin GPIO_PIN_9
#define N_RST_GPS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
