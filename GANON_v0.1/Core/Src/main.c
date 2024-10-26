/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  /* Includes ------------------------------------------------------------------*/


  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */
#include "main.h"

#include "peripherals/dma.h"
#include "peripherals/i2c.h"
#include "peripherals/spi.h"
#include "peripherals/tim.h"
#include "peripherals/usart.h"
#include "peripherals/gpio.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "drivers/BMI088.h"
#include "drivers/BMP388.h"
#include "drivers/buzzer.h"
#include "drivers/gps.h"
#include "drivers/rfm96w.h"
#include "drivers/w25q_mem.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SONG_BANK song_bank;
MACHINE machine;
SCHEDULER scheduler;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

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
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 2 */

    SPI_HandleTypeDef_flag_init(&hspi1_flag, &hspi1);
    SPI_HandleTypeDef_flag_init(&hspi2_flag, &hspi2);

    // ======================================= Creation of components =======================================


    // looking for max of context sizes for async tasks
    uint8_t length = 21;
    uint32_t arr[21] = {
        sizeof(ASYNC_BMI088_ReadSensorDMA_CONTEXT),
    
        sizeof(ASYNC_play_note_CONTEXT),
    
        sizeof(ASYNC_W25Q_ReadStatusReg_CONTEXT),
        sizeof(ASYNC_W25Q_WriteEnable_CONTEXT),
        sizeof(ASYNC_W25Q_WaitForReady_CONTEXT),
        sizeof(ASYNC_W25Q_EarseSector_CONTEXT),
        sizeof(ASYNC_W25Q_EarseAll_CONTEXT),
        sizeof(ASYNC_W25Q_ReadData_CONTEXT),
        sizeof(ASYNC_W25Q_PageProgram_CONTEXT),
        sizeof(ASYNC_W25Q_WriteData_CONTEXT),
    
        sizeof(ASYNC_fs_read_write_CONTEXT),
        sizeof(ASYNC_fs_read_write_float_CONTEXT),
    
        sizeof(ASYNC_test_usb_CONTEXT),
        sizeof(ASYNC_update_usb_watchdog_CONTEXT),
        sizeof(ASYNC_send_usb_CONTEXT),
        sizeof(ASYNC_update_IMU_CONTEXT),
        sizeof(ASYNC_test_w25q_1_CONTEXT),
        sizeof(ASYNC_test_fs_CONTEXT),
        sizeof(ASYNC_save_IMU_CONTEXT),
        sizeof(ASYNC_test_w25q_2_CONTEXT),

        sizeof(ASYNC_SPI_TxRx_DMA_CONTEXT)
    };

    uint32_t max = arr[0];
    float mean = 0;
    uint8_t max_idx = 0;
    for (int i = 1; i < length; i++) {
        if (arr[i] > max) {
            max = arr[i];
            max_idx = i;
        }
        mean += arr[i];
    }
    mean /= (float)length;






    W25Q_Chip flash_chip;
    BMI088 imu;
    BMP388_HandleTypeDef bmp;
    RFM96_Chip lora_chip;
    // GPS_t GPS;   <--- GPS is already defined in gps.h
    BUZZER buzzer;

    COMPONENTS components;
    init_components(&components, &flash_chip, &lora_chip, &imu, &bmp, &GPS, &buzzer);

    FLASH_STREAM flash_stream;
    DATAS datas;

    init_machine(&machine, &components, &flash_stream, &datas);

    // ======================================================================================================

    
    // uint8_t *test1 = (uint8_t*)malloc(sizeof(uint8_t) * 20);

    // for (int i = 0; i < 20; i++) {
    //     test1[i] = i;
    // }

    // uint8_t *test2 = 


    __NOP();

    // ======================================================================================================


    init_scheduler(&scheduler);

    __NOP();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        run_scheduler(&scheduler);
        add_task_if_flag(&scheduler, &machine);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 72;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
// ===========================================

void init_components(COMPONENTS* components,
                     W25Q_Chip* flash_chip,
                     RFM96_Chip* lora_chip,
                     BMI088* imu,
                     BMP388_HandleTypeDef* bmp,
                     GPS_t* gps,
                     BUZZER* buzzer) {
    components->flash = flash_chip;
    components->lora = lora_chip;
    components->imu = imu;
    components->bmp = bmp;
    components->gps = gps;
    components->buzzer = buzzer;
}

void init_all_components(COMPONENTS* components) {
    W25Q_Init(components->flash, &hspi2_flag, N_CS_FLASH_GPIO_Port, N_CS_FLASH_Pin, W25Q_Q_FULL_DEVICE_ID);

    BMI088_Init(components->imu, &hspi1_flag, N_CS_ACC_GPIO_Port, N_CS_ACC_Pin, N_CS_GYRO_GPIO_Port, N_CS_GYRO_Pin);

    components->bmp->hi2c = &hi2c1;
    BMP388_Init(components->bmp);

    RFM96_Init(components->lora, &hspi1, N_CS_LORA_GPIO_Port, N_CS_LORA_Pin,
               N_RST_LORA_GPIO_Port, N_RST_LORA_Pin, 433365e3);
    GPS_Init(components->gps, &huart1);

    BUZZER_Init(components->buzzer, &htim1, TIM_CHANNEL_1);
}


// ===========================================


void init_machine(MACHINE* machine, COMPONENTS* components,
                  FLASH_STREAM* flash_stream, DATAS* datas) {
    machine->state = WAIT_USB;

    machine->components = components;
    init_all_components(components);

    machine->flash_stream = flash_stream;
    flash_stream_init(flash_stream, components->flash);

    machine->datas = datas;
    Datas_Init(datas, components->imu, components->bmp, components->gps);

    set_song_bank(&song_bank);

    machine->usb_watchdog_bfr[0] = USBD_BUSY;
    machine->usb_watchdog_bfr[1] = USBD_BUSY;
    machine->usb_watchdog_bfr[2] = USBD_BUSY;

    machine->last_tick = 0;
    machine->last_time = HAL_GetTick();

    // ===== POST VOLE ===== POUR LIRE DIRECTEMENT ======
    // machine->state = WAIT_FOR_SENDING_DATA;
    // flash_stream->write_ptr = 0xffffff;
    // ===== POST VOLE ==================================
}



void add_task_if_flag(SCHEDULER* scheduler, MACHINE* machine) {
    switch (machine->state) {
    case WAIT_USB: {
        machine->state = DEFAULT_STATE;

        TASK *task_buz, *task_usb;

        task_buz = add_task(scheduler, ASYNC_play_note);
        ASYNC_play_note_init(task_buz, machine->components->buzzer,
                             song_bank.beep_0_freqs,
                             song_bank.beep_0_durations, BEEP_SONG_SIZE);

        task_usb = add_task(scheduler, ASYNC_update_usb_watchdog);
        ASYNC_update_usb_watchdog_init(task_usb, machine);
        break; }

    case USB_READY: {
        machine->state = TEST_2;
        break; }

    case TEST_1: {
        machine->state = DEFAULT_STATE;

        TASK *task_buz, *task_imu, *task_usb;

        task_buz = add_task(scheduler, ASYNC_play_note);
        ASYNC_play_note_init(task_buz, machine->components->buzzer,
                             song_bank.beep_2_freqs,
                             song_bank.beep_2_durations, BEEP_SONG_SIZE);

        task_imu = add_task(scheduler, ASYNC_update_IMU);
        ASYNC_update_IMU_init(task_imu, 10);

        task_usb = add_task(scheduler, ASYNC_send_usb);
        ASYNC_send_usb_init(task_usb, 20);
        break; }
    case TEST_2: {
        machine->state = DEFAULT_STATE;

        TASK *task_buz, *task_fs;

        // task_buz = add_task(scheduler, ASYNC_play_note);
        // ASYNC_play_note_init(task_buz, machine->components->buzzer, song_bank.beep_2_freqs,
        //                      song_bank.beep_2_durations, BEEP_SONG_SIZE);

        task_fs = add_task(scheduler, ASYNC_test_fs);
        ASYNC_test_fs_init(task_fs, machine->flash_stream);

        break; }
    case TEST_3: {
        machine->state = DEFAULT_STATE;

        TASK *task = add_task(scheduler, ASYNC_save_IMU);
        ASYNC_save_IMU_init(task);
        break; }
    case TEST_4: {
        machine->state = DEFAULT_STATE;

        TASK *task = add_task(scheduler, ASYNC_test_w25q_2);
        ASYNC_test_w25q_2_init(task, machine->components->flash);
        break; }
    default:
        break;
    }
}

void lora_wait(MACHINE* machine) {
    if (HAL_GetTick() - machine->last_tick > 1000) {
        RFM96_Print(machine->components->lora, ".");
        machine->last_tick = HAL_GetTick();
    }
}


// ===========================================


void update_usb_watchdog(uint8_t *usb_watchdog_bfr) {
    usb_watchdog_bfr[0] = usb_watchdog_bfr[1];
    usb_watchdog_bfr[1] = usb_watchdog_bfr[2];
    usb_watchdog_bfr[2] = CDC_Transmit_FS(NULL, 0);
    HAL_Delay(1);
}

bool is_connected_to_usb(uint8_t *usb_watchdog_bfr) {
    return ((usb_watchdog_bfr[0] == USBD_OK) &&
            (usb_watchdog_bfr[1] == USBD_OK) &&
            (usb_watchdog_bfr[2] == USBD_OK));
}

void ASYNC_update_usb_watchdog_init(TASK *task, MACHINE *machine) {
    ASYNC_update_usb_watchdog_CONTEXT *context = (ASYNC_update_usb_watchdog_CONTEXT*)task->context;

    context->state = &(machine->state);
    context->usb_watchdog_bfr[0] = USBD_BUSY;
    context->usb_watchdog_bfr[1] = USBD_BUSY;
    context->usb_watchdog_bfr[2] = USBD_BUSY;
    context->next_time = 0;
}

void ASYNC_update_usb_watchdog(SCHEDULER *scheduler, TASK *self) {
    ASYNC_update_usb_watchdog_CONTEXT* context = (ASYNC_update_usb_watchdog_CONTEXT*)self->context;
    uint32_t current_time = HAL_GetTick();
    if (current_time >= context->next_time) {
        context->usb_watchdog_bfr[0] = context->usb_watchdog_bfr[1];
        context->usb_watchdog_bfr[1] = context->usb_watchdog_bfr[2];
        context->usb_watchdog_bfr[2] = CDC_Transmit_FS(NULL, 0);
        context->next_time = HAL_GetTick() + 1;

        if (is_connected_to_usb(context->usb_watchdog_bfr)) {
            *(context->state) = USB_READY;
            kill_task(scheduler, self);
        }
    }
}


// ===========================================


void flash_stream_find_last_info_ptr(FLASH_STREAM* stream) {
    uint32_t last_read_ptr, last_write_ptr;
    MACHINE_STATE last_state = DEFAULT_STATE;

    for (stream->last_info_ptr = LAST_INFO_ADDR;
         stream->last_info_ptr > LAST_SECTOR_ADDR + INFO_SIZE && last_state == DEFAULT_STATE;
         stream->last_info_ptr -= INFO_SIZE) {
        load_info_from_flash(stream, &last_read_ptr, &last_write_ptr, &last_state);
    }
    if (last_state != DEFAULT_STATE) {
        stream->last_info_ptr += INFO_SIZE;
    }
    __NOP();
}

void save_data_to_flash(FLASH_STREAM* stream, DATAS datas) {
    float datas_buff[DATA_NUMBER] = { datas.accel[0],
                                      datas.accel[1],
                                      datas.accel[2],
                                      datas.gyro[0],
                                      datas.gyro[1],
                                      datas.gyro[2],
                                      datas.pressure,
                                      datas.temp,
                                      datas.longitude,
                                      datas.latitude,
                                      datas.time };

    flash_stream_write_floats(stream, datas_buff, DATA_NUMBER);
}

void save_info_to_flash(FLASH_STREAM* stream, MACHINE_STATE state) {
    if (stream->last_info_ptr + INFO_SIZE > LAST_SECTOR_ADDR + SECTOR_SIZE) {
        stream->last_info_ptr = LAST_SECTOR_ADDR + INFO_SIZE;
        W25Q_EarseSector(stream->flash_chip, LAST_SECTOR_ADDR);
    }

    uint8_t tx_buff[INFO_SIZE] = { stream->read_ptr >> 24, // First, 4 bytes for read_ptr
                                   stream->read_ptr >> 16,
                                   stream->read_ptr >> 8,
                                   stream->read_ptr >> 0,

                                   stream->write_ptr >> 24, // Second, 4 bytes for write_ptr
                                   stream->write_ptr >> 16,
                                   stream->write_ptr >> 8,
                                   stream->write_ptr >> 0,

                                   state }; // Third, 1 byte for state
    W25Q_PageProgram(stream->flash_chip, tx_buff, stream->last_info_ptr, INFO_SIZE);
    stream->last_info_ptr += INFO_SIZE;
}

void load_info_from_flash(FLASH_STREAM* stream, uint32_t* last_read_ptr,
                          uint32_t* last_write_ptr, MACHINE_STATE* last_state) {
    uint8_t rx_buff[INFO_SIZE];
    uint32_t addr = ((int)(stream->last_info_ptr)) - ((int)INFO_SIZE);

    W25Q_ReadData(stream->flash_chip, rx_buff, addr, INFO_SIZE);
    *last_read_ptr = (rx_buff[0] << 24 | rx_buff[1] << 16 | rx_buff[2] << 8 | rx_buff[3]);
    *last_write_ptr = (rx_buff[4] << 24 | rx_buff[5] << 16 | rx_buff[6] << 8 | rx_buff[7]);
    *last_state = rx_buff[8];
}


// ===========================================


void Datas_Init(DATAS* datas, BMI088* imu, BMP388_HandleTypeDef* bmp, GPS_t* gps) {
    datas->imu = imu;
    datas->bmp = bmp;
    datas->gps = gps;
}

void load_all_datas_from_sensors(DATAS* datas) {
    uint32_t raw_press, raw_temp, sensor_time;
    float press, temp;

    BMI088_ReadAccelerometer(datas->imu);
    datas->accel[0] = datas->imu->acc_mps2[0];
    datas->accel[1] = datas->imu->acc_mps2[1];
    datas->accel[2] = datas->imu->acc_mps2[2];

    BMI088_ReadGyroscope(datas->imu);
    datas->gyro[0] = datas->imu->gyr_rps[0];
    datas->gyro[1] = datas->imu->gyr_rps[1];
    datas->gyro[2] = datas->imu->gyr_rps[2];

    BMP388_ReadRawPressTempTime(datas->bmp, &raw_press, &raw_temp, &sensor_time);
    BMP388_CompensateRawPressTemp(datas->bmp, raw_press, raw_temp, &press, &temp);
    datas->pressure = press;
    datas->temp = temp;

    GPS_UART_Receive(datas->gps);
    datas->longitude = datas->gps->dec_longitude;
    datas->latitude = datas->gps->dec_latitude;

    datas->time = HAL_GetTick();
}

void load_all_datas_from_flash(DATAS* datas, FLASH_STREAM* stream) {
    float datas_buff[DATA_NUMBER];

    flash_stream_read_floats(stream, datas_buff, DATA_NUMBER);

    datas->accel[0]  = datas_buff[0];
    datas->accel[1]  = datas_buff[1];
    datas->accel[2]  = datas_buff[2];
    datas->gyro[0]   = datas_buff[3];
    datas->gyro[1]   = datas_buff[4];
    datas->gyro[2]   = datas_buff[5];
    datas->pressure  = datas_buff[6];
    datas->temp      = datas_buff[7];
    datas->longitude = datas_buff[8];
    datas->latitude  = datas_buff[9];
    datas->time      = datas_buff[10];
}


// =============================================


void get_csv_header(char* buff) {
    sprintf(buff, "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,pressure,temp,long,lat,time\n");
}

void get_csv_line(DATAS datas, char* buff) {
    char float_convert_buff[256];
    float_format(float_convert_buff, datas.accel[0], 2, 10);
    sprintf(buff, "%s,", float_convert_buff);
    float_format(float_convert_buff, datas.accel[1], 2, 10);
    sprintf(buff, "%s%s,", buff, float_convert_buff);
    float_format(float_convert_buff, datas.accel[2], 2, 10);
    sprintf(buff, "%s%s,", buff, float_convert_buff);
    float_format(float_convert_buff, datas.gyro[0], 2, 10);
    sprintf(buff, "%s%s,", buff, float_convert_buff);
    float_format(float_convert_buff, datas.gyro[1], 2, 10);
    sprintf(buff, "%s%s,", buff, float_convert_buff);
    float_format(float_convert_buff, datas.gyro[2], 2, 10);
    sprintf(buff, "%s%s,", buff, float_convert_buff);
    float_format(float_convert_buff, datas.pressure, 2, 10);
    sprintf(buff, "%s%s,", buff, float_convert_buff);
    float_format(float_convert_buff, datas.temp, 2, 10);
    sprintf(buff, "%s%s,", buff, float_convert_buff);
    float_format(float_convert_buff, datas.longitude, 6, 10);
    sprintf(buff, "%s%s,", buff, float_convert_buff);
    float_format(float_convert_buff, datas.latitude, 6, 10);
    sprintf(buff, "%s%s,", buff, float_convert_buff);
    float_format(float_convert_buff, datas.time, 1, 10);
    sprintf(buff, "%s%s\n", buff, float_convert_buff);
}


void ASYNC_test_usb_init(TASK *task, char *buff, int len, uint32_t delay) {
    ASYNC_test_usb_CONTEXT* context = (ASYNC_test_usb_CONTEXT*)task->context;

    context->buff = buff;
    context->len = len;
    context->index = -1;
    context->next_time = 0;
    context->delay = delay;
}

void ASYNC_test_usb(SCHEDULER *scheduler, TASK *self) {
    ASYNC_test_usb_CONTEXT* context = (ASYNC_test_usb_CONTEXT*)self->context;
    uint32_t current_tick = HAL_GetTick();
    if (context->index == -1) {
        HAL_Delay(1);
        CDC_Transmit_FS((uint8_t*)"START \"", 8);
        context->next_time = current_tick + context->delay;
        context->index = 0;
    } else if (context->index < context->len) {
        if (current_tick >= context->next_time) {
            char* c = context->buff + context->index;
            CDC_Transmit_FS((uint8_t*)c, 1);
            context->index++;
            context->next_time = current_tick + context->delay;
        }
    } else {
        HAL_Delay(1);
        CDC_Transmit_FS((uint8_t*)"\" END\n", 7);
        kill_task(scheduler, self);
    }
}


void ASYNC_send_usb_init(TASK *task, uint32_t delay) {
    ASYNC_send_usb_CONTEXT* context = (ASYNC_send_usb_CONTEXT*)task->context;

    context->next_time = 0;
    context->delay = delay;
}

void ASYNC_send_usb(SCHEDULER *scheduler, TASK *self) {
    ASYNC_send_usb_CONTEXT* context = (ASYNC_send_usb_CONTEXT*)self->context;
    uint32_t current_tick = HAL_GetTick();

    if (current_tick >= context->next_time) {
        char buff[256];
        char acc_x[10], acc_y[10], acc_z[10];
        char gyr_x[10], gyr_y[10], gyr_z[10];
        char time[10];
        // int num_calls = machine.components->imu->num_calls;

        float_format(acc_x, machine.components->imu->acc_mps2[0], 5, 10);
        float_format(acc_y, machine.components->imu->acc_mps2[1], 5, 10);
        float_format(acc_z, machine.components->imu->acc_mps2[2], 5, 10);

        float_format(gyr_x, machine.components->imu->gyr_rps[0], 5, 10);
        float_format(gyr_y, machine.components->imu->gyr_rps[1], 5, 10);
        float_format(gyr_z, machine.components->imu->gyr_rps[2], 5, 10);

        float_format(time, (float)current_tick, 1, 10);

        sprintf(buff, "%s, %s, %s, %s, %s, %s, %s\n", time, acc_x, acc_y, acc_z,
                                                            gyr_x, gyr_y, gyr_z);
        // sprintf(buff, "%s, %s, %s\r\n", acc_x, acc_y, acc_z);
        // sprintf(buff, "%s, %s, %s\r\n", gyr_x, gyr_y, gyr_z);
        CDC_Transmit_FS((uint8_t*)buff, strlen(buff));

        context->next_time = current_tick + context->delay;
    }
}


void ASYNC_update_IMU_init(TASK *task, uint32_t delay) {
    ASYNC_update_IMU_CONTEXT* context = (ASYNC_update_IMU_CONTEXT*)task->context;

    context->continue_update = true;
    context->next_time = 0;
    context->delay = delay;
}

void ASYNC_update_IMU(SCHEDULER *scheduler, TASK *self) {
    ASYNC_update_IMU_CONTEXT* context = (ASYNC_update_IMU_CONTEXT*)self->context;
    uint32_t current_tick = HAL_GetTick();

    if (context->continue_update) {
        if (current_tick >= context->next_time) {

            TASK *task_acc, *task_gyr;

            task_acc = add_task(scheduler, ASYNC_BMI088_ReadAccelerometerDMA);
            ASYNC_BMI088_ReadSensorDMA_init(task_acc, machine.components->imu);

            task_gyr = add_task(scheduler, ASYNC_BMI088_ReadGyroscopeDMA);
            ASYNC_BMI088_ReadSensorDMA_init(task_gyr, machine.components->imu);

            context->next_time = current_tick + context->delay;
        }
    } else {
        kill_task(scheduler, self);
    }
}


void ASYNC_test_w25q_1_init(TASK *task, W25Q_Chip *flash_chip) {
    ASYNC_test_w25q_1_CONTEXT* context = (ASYNC_test_w25q_1_CONTEXT*)task->context;

    context->flash_chip = flash_chip;

    for (int i = 0; i < 12; i++) {
        context->tx_buffer[i] = i;
    }

    for (int i = 0; i < 12; i++) {
        context->rx_buffer[i] = 0;
    }

    context->state = ASYNC_test_w25q_1_START;
}

void ASYNC_test_w25q_1(SCHEDULER *scheduler, TASK *self) {
    ASYNC_test_w25q_1_CONTEXT* context = (ASYNC_test_w25q_1_CONTEXT*)self->context;

    switch (context->state) {
    case ASYNC_test_w25q_1_START: {
        W25Q_EarseSector(context->flash_chip, 0);

        context->is_done = false;
        TASK *task = add_task(scheduler, ASYNC_W25Q_PageProgram);
        ASYNC_W25Q_PageProgram_init(task, context->flash_chip, context->tx_buffer, 0, 12);
        task->is_done = &(context->is_done);
        context->state = ASYNC_test_w25q_1_WAIT_TX;
        break; }
    case ASYNC_test_w25q_1_WAIT_TX: {
        uint8_t rx_buff[12];

        if (context->is_done) {
            W25Q_ReadData(context->flash_chip, rx_buff, 0, 12);
            context->state = ASYNC_test_w25q_1_DONE_TX;
        }
        break; }
    case ASYNC_test_w25q_1_DONE_TX: {
        context->is_done = false;
        TASK *task = add_task(scheduler, ASYNC_W25Q_ReadData);
        ASYNC_W25Q_ReadData_init(task, context->flash_chip, context->rx_buffer, 0, 12);
        task->is_done = &(context->is_done);
        context->state = ASYNC_test_w25q_1_WAIT_RX;
        break; }
    case ASYNC_test_w25q_1_WAIT_RX: {
        if (context->is_done) {
            context->state = ASYNC_test_w25q_1_DONE_RX;
        }
        break; }
    case ASYNC_test_w25q_1_DONE_RX: {
        kill_task(scheduler, self);
        break; }
    }
}


void ASYNC_test_fs_init(TASK *task, FLASH_STREAM *flash_stream) {
    ASYNC_test_fs_CONTEXT* context = (ASYNC_test_fs_CONTEXT*)task->context;

    context->flash_stream = flash_stream;

    context->tx_buffer = (float*)malloc(12 * sizeof(float));
    context->rx_buffer = (float*)malloc(12 * sizeof(float));

    for (int i = 0; i < 4; i++) {
        context->tx_buffer[i + 0] = 31.4 + 1/((float)i + 2.0);
        context->tx_buffer[i + 4] = 69.0 + 1/((float)i + 2.0);
        context->tx_buffer[i + 8] = 42.0 + 1/((float)i + 2.0);
    }

    memset(context->rx_buffer, 0, 12 * sizeof(float));

    context->idx = 0;
    context->state = ASYNC_test_fs_START;
}

void ASYNC_test_fs(SCHEDULER *scheduler, TASK *self) {
    ASYNC_test_fs_CONTEXT* context = (ASYNC_test_fs_CONTEXT*)self->context;

    switch (context->state) {
    case ASYNC_test_fs_START: {
        W25Q_EarseSector(context->flash_stream->flash_chip, 0);
        // W25Q_EarseAll(context->flash_stream->flash_chip);
        W25Q_WaitForReady(context->flash_stream->flash_chip);
        context->state = ASYNC_test_fs_START_TX;
        break; }
    case ASYNC_test_fs_START_TX: {
        context->is_done = false;
        TASK *task = add_task(scheduler, ASYNC_fs_write_floats);
        ASYNC_fs_read_write_floats_init(task, context->flash_stream,
                                       context->tx_buffer + 4 * context->idx, 4);

        task->is_done = &(context->is_done);
        context->state = ASYNC_test_fs_WAIT_TX;
        break; }
    case ASYNC_test_fs_WAIT_TX: {
        if (context->is_done) {
            if (context->idx < 3) {
                context->idx += 1;
                context->state = ASYNC_test_fs_START_TX;
            } else {
                context->idx = 0;
                context->state = ASYNC_test_fs_DONE_TX;
            }
        }
        break; }
    case ASYNC_test_fs_DONE_TX: {
        uint8_t rx_buff[12];
        W25Q_ReadData(context->flash_stream->flash_chip, rx_buff, 0, 12);


        context->is_done = false;
        TASK *task = add_task(scheduler, ASYNC_fs_read_floats);
        ASYNC_fs_read_write_floats_init(task, context->flash_stream,
                                       context->rx_buffer + 4 * context->idx, 4);
        task->is_done = &(context->is_done);
        context->state = ASYNC_test_fs_WAIT_RX;
        break; }
    case ASYNC_test_fs_WAIT_RX: {
        if (context->is_done) {
            if (context->idx < 3) {
                context->idx += 1;
                context->state = ASYNC_test_fs_DONE_TX;
            } else {
                context->state = ASYNC_test_fs_DONE_RX;
            }
        }
        break; }
    case ASYNC_test_fs_DONE_RX: {
        free(context->tx_buffer);
        free(context->rx_buffer);
        kill_task(scheduler, self);
        break; }
    }
}


void ASYNC_save_IMU_init(TASK *task) {
    ASYNC_save_IMU_CONTEXT* context = (ASYNC_save_IMU_CONTEXT*)task->context;

    context->max_delay = 30000; // 10 seconds
    context->imu_delay = 10;
    context->state = ASYNC_save_IMU_START_ERASE;

    context->ready = false;
    context->save_done = false;
}

void ASYNC_save_IMU(SCHEDULER* scheduler, TASK* self) {
    ASYNC_save_IMU_CONTEXT* context = (ASYNC_save_IMU_CONTEXT*)self->context;
    uint32_t current_tick = HAL_GetTick();

    switch (context->state) {
    case ASYNC_save_IMU_START_ERASE: {
        context->ready = false;
        TASK *task = add_task(scheduler, ASYNC_W25Q_EraseAll);
        ASYNC_W25Q_EraseAll_init(task, machine.flash_stream->flash_chip);
        task->is_done = &(context->ready);
        context->state = ASYNC_save_IMU_WAIT_ERASE_TASK;
        break; }
    case ASYNC_save_IMU_WAIT_ERASE_TASK: {
        if (context->ready) {
            context->ready = false;
            TASK *task = add_task(scheduler, ASYNC_W25Q_WaitForReady);
            ASYNC_W25Q_WaitForReady_init(task, machine.flash_stream->flash_chip);
            task->is_done = &(context->ready); 
            context->state = ASYNC_save_IMU_WAIT_ERASE_DONE;
        }
        break; }
    case ASYNC_save_IMU_WAIT_ERASE_DONE: {
        if (context->ready) {
            context->state = ASYNC_save_IMU_START_UPDATE;

            TASK *task = add_task(scheduler, ASYNC_play_note);
            ASYNC_play_note_init(task, machine.components->buzzer,
                                 song_bank.beep_1_freqs,
                                 song_bank.beep_1_durations,
                                 BEEP_SONG_SIZE);
        }
        break; }
    case ASYNC_save_IMU_START_UPDATE: {
        context->first_time = current_tick;
        context->task_imu = add_task(scheduler, ASYNC_update_IMU);
        ASYNC_update_IMU_init(context->task_imu, context->imu_delay);

        context->state = ASYNC_save_IMU_WAIT_UPDATE;
        break; }
    case ASYNC_save_IMU_WAIT_UPDATE: {
        if (machine.components->imu->new_acc_data && machine.components->imu->new_gyr_data) {
            machine.components->imu->new_acc_data = false;
            machine.components->imu->new_gyr_data = false;

            context->save_done = false;

            context->datas_buff[0] = machine.components->imu->acc_mps2[0];
            context->datas_buff[1] = machine.components->imu->acc_mps2[1];
            context->datas_buff[2] = machine.components->imu->acc_mps2[2];
            context->datas_buff[3] = machine.components->imu->gyr_rps[0];
            context->datas_buff[4] = machine.components->imu->gyr_rps[1];
            context->datas_buff[5] = machine.components->imu->gyr_rps[2];
            context->datas_buff[6] = (float)current_tick;

            TASK *task = add_task(scheduler, ASYNC_fs_write_floats);
            ASYNC_fs_read_write_floats_init(task, machine.flash_stream,
                                           context->datas_buff, 7);
            task->is_done = &(context->save_done);
            
            context->state = ASYNC_save_IMU_WAIT_SAVE;
        }
        break; }
    case ASYNC_save_IMU_WAIT_SAVE: {
        if (context->save_done) {
            if (current_tick - context->first_time >= context->max_delay) {
                context->state = ASYNC_save_IMU_END_UPDATE;
            } else {
                context->state = ASYNC_save_IMU_WAIT_UPDATE;
            }
        }
        break; }
    case ASYNC_save_IMU_END_UPDATE: {
        ASYNC_update_IMU_CONTEXT* context_update_imu = (ASYNC_update_IMU_CONTEXT*)(context->task_imu->context);
        context_update_imu->continue_update = false; // stop updating IMU
        context->state = ASYNC_save_IMU_START_LOAD;

        TASK *task = add_task(scheduler, ASYNC_play_note);
        ASYNC_play_note_init(task, machine.components->buzzer,
                             song_bank.beep_2_freqs,
                             song_bank.beep_2_durations,
                             BEEP_SONG_SIZE);
        break; }
    case ASYNC_save_IMU_START_LOAD: {
        context->save_done = false;

        TASK *task = add_task(scheduler, ASYNC_fs_read_floats);
        ASYNC_fs_read_write_floats_init(task, machine.flash_stream,
                                       context->datas_buff, 7);
        task->is_done = &(context->save_done);

        context->state = ASYNC_save_IMU_WAIT_LOAD;
        break; }
    case ASYNC_save_IMU_WAIT_LOAD: {
        if (context->save_done) {
            context->state = ASYNC_save_IMU_START_SEND;
        }
        break; }
    case ASYNC_save_IMU_START_SEND: {
        // uint8_t rx_buff[28*100];
        // W25Q_ReadData(machine.flash_stream->flash_chip, rx_buff, 28*100, 28*100);

        context->next_time = current_tick + 2;
        char buff[256];
        char acc_x[10], acc_y[10], acc_z[10];
        char gyr_x[10], gyr_y[10], gyr_z[10];
        char time[10];

        float_format(acc_x, context->datas_buff[0], 5, 10);
        float_format(acc_y, context->datas_buff[1], 5, 10);
        float_format(acc_z, context->datas_buff[2], 5, 10);

        float_format(gyr_x, context->datas_buff[3], 5, 10);
        float_format(gyr_y, context->datas_buff[4], 5, 10);
        float_format(gyr_z, context->datas_buff[5], 5, 10);

        float_format(time, context->datas_buff[6], 1, 10);

        sprintf(buff, "%s, %s, %s, %s, %s, %s, %s\r\n", acc_x, acc_y, acc_z,
                                                        gyr_x, gyr_y, gyr_z, time);
        CDC_Transmit_FS((uint8_t*)buff, strlen(buff));

        context->state = ASYNC_save_IMU_WAIT_SEND;
        break; }
    case ASYNC_save_IMU_WAIT_SEND: {
        if (current_tick >= context->next_time) {
            if (machine.flash_stream->read_ptr < machine.flash_stream->write_ptr) {
                context->state =  ASYNC_save_IMU_START_LOAD;
            } else {
                context->state = ASYNC_save_IMU_END_SEND;
            }
        }
        break; }
    case ASYNC_save_IMU_END_SEND: {
        kill_task(scheduler, self);
        break; }
    }
}


void ASYNC_test_w25q_2_init(TASK *task, W25Q_Chip *flash_chip) {
    ASYNC_test_w25q_2_CONTEXT *context = (ASYNC_test_w25q_2_CONTEXT*)task->context;

    W25Q_EarseSector(flash_chip, 0);

    context->flash_chip = flash_chip;

    context->tx_buffer = (uint8_t*)malloc(256 * 4 * sizeof(uint8_t));
    context->rx_buffer = (uint8_t*)malloc(256 * 5 * sizeof(uint8_t));

    uint8_t a = (uint8_t)'a';

    for (int i = 0; i < 4; i++) {
        memset(context->tx_buffer + 256 * i, a + i, 256);
    }

    context->state = ASYNC_test_w25q_2_START;
}

void ASYNC_test_w25q_2(SCHEDULER *scheduler, TASK *self) {
    ASYNC_test_w25q_2_CONTEXT *context = (ASYNC_test_w25q_2_CONTEXT*)self->context;

    switch (context->state) {
    case ASYNC_test_w25q_2_START: {
        context->is_done = false;
        TASK *task = add_task(scheduler, ASYNC_W25Q_WriteData);
        ASYNC_W25Q_WriteData_init(task, context->flash_chip, context->tx_buffer, 30, 256 * 4);
        task->is_done = &(context->is_done);
        context->state = ASYNC_test_w25q_2_WAIT;
        break; }
    case ASYNC_test_w25q_2_WAIT: {
        if (context->is_done) {
            uint32_t addr = 0;
            for (uint8_t i = 0; i < 5; i++) {
                addr = i * 256;
                W25Q_ReadData(context->flash_chip,
                    context->rx_buffer + addr, addr, 256);
            }
            context->state = ASYNC_test_w25q_2_END;
        }
        break; }
    case ASYNC_test_w25q_2_END: {
        free(context->tx_buffer);
        free(context->rx_buffer);
        kill_task(scheduler, self);
        break; }
    }
}


// =============================================


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
       /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
