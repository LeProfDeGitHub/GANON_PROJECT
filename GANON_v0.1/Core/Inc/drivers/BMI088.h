#ifndef BMI088_IMU_H
#define BMI088_IMU_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "scheduler.h"
#include "tools.h"

/* Register defines */
#define BMI_ACC_CHIP_ID 		0x00
#define BMI_ACC_DATA 			0x12
#define BMI_TEMP_DATA 			0x22
#define BMI_ACC_CONF 			0x40
#define BMI_ACC_RANGE 			0x41
#define BMI_INT1_IO_CONF 	   	0x53
#define BMI_INT1_INT2_MAP_DATA 	0x58
#define BMI_ACC_PWR_CONF 		0x7C
#define BMI_ACC_PWR_CTRL 		0x7D
#define BMI_ACC_SOFTRESET 		0x7E

#define BMI_GYR_CHIP_ID			0x00
#define BMI_GYR_DATA			0x02
#define	BMI_GYR_RANGE			0x0F
#define	BMI_GYR_BANDWIDTH		0x10
#define	BMI_GYR_SOFTRESET		0x14
#define	BMI_GYR_INT_CTRL		0x15
#define	BMI_INT3_INT4_IO_CONF	0x16
#define BMI_INT3_INT4_IO_MAP	0x18

typedef struct {

	/* SPI */
	SPI_HandleTypeDef_flag *spiHandle_flag;
	SPI_HandleTypeDef	   *spiHandle;
	GPIO_TypeDef 	  	   *csAccPinBank;
	GPIO_TypeDef 	  	   *csGyrPinBank;
	uint16_t 		  	    csAccPin;
	uint16_t 		  	    csGyrPin;

	/* DMA */
	// bool use_acc_spi;
	// bool use_gyr_spi;
	// bool *dma_complete;

	/* Conversion constants (raw to m/s^2 and raw to rad/s) */
	float accConversion;
	float gyrConversion;

	/* x-y-z measurements */
	float acc_mps2[3];
	float gyr_rps[3];

	bool new_acc_data;
	bool new_gyr_data;

	bool ASYNC_busy;
} BMI088;


typedef enum ASYNC_BMI088_ReadSensorDMA_State {
	ASYNC_BMI088_ReadSensorDMA_WAIT_BMI088,
	ASYNC_BMI088_ReadSensorDMA_START,
	ASYNC_BMI088_ReadSensorDMA_WAIT_DMA,
	ASYNC_BMI088_ReadSensorDMA_END,
} ASYNC_BMI088_ReadSensorDMA_State;

typedef struct {
	BMI088 *imu;

	ASYNC_BMI088_ReadSensorDMA_State state;
	bool dma_complete;

	uint8_t txBuf[8];
	uint8_t rxBuf[8];
} ASYNC_BMI088_ReadSensorDMA_CONTEXT;


/*
 *
 * INITIALISATION
 *
 */
uint8_t BMI088_Init(BMI088 *imu,
				    SPI_HandleTypeDef_flag *spiHandle_flag,
				    GPIO_TypeDef *csAccPinBank, uint16_t csAccPin,
				    GPIO_TypeDef *csGyrPinBank, uint16_t csGyrPin);

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */
uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);
uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);

uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);
uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);

/*
 *
 * POLLING
 *
 */
uint8_t BMI088_ReadAccelerometer(BMI088 *imu);
uint8_t BMI088_ReadGyroscope(BMI088 *imu);

/*
 *
 * DMA
 *
 */
void ASYNC_BMI088_ReadSensorDMA_init(TASK *task, BMI088 *imu);
void ASYNC_BMI088_ReadAccelerometerDMA(SCHEDULER *scheduler, TASK *self);
void ASYNC_BMI088_ReadGyroscopeDMA(SCHEDULER *scheduler, TASK *self);

#endif