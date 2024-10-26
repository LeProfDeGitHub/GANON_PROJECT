#include "drivers/BMI088.h"
#include <string.h>

/*
 *
 * INITIALISATION
 *
 */
uint8_t BMI088_Init(BMI088 *imu,
				    SPI_HandleTypeDef_flag *spiHandle_flag,
				    GPIO_TypeDef *csAccPinBank, uint16_t csAccPin,
				    GPIO_TypeDef *csGyrPinBank, uint16_t csGyrPin) {

	/* Store interface parameters in struct */
	imu->spiHandle_flag = spiHandle_flag;
	imu->spiHandle 		= spiHandle_flag->hspi;
	imu->csAccPinBank 	= csAccPinBank;
	imu->csAccPin 		= csAccPin;
	imu->csGyrPinBank 	= csGyrPinBank;
	imu->csGyrPin 		= csGyrPin;

	imu->new_acc_data = false;
	imu->new_gyr_data = false;

	imu->ASYNC_busy = false;

	uint8_t status = 0;

	/*
	 *
	 * ACCELEROMETER
	 *
	 */

	/* Accelerometer requires rising edge on CSB at start-up to activate SPI */
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
	HAL_Delay(50);

	/* Perform accelerometer soft reset */
	status += BMI088_WriteAccRegister(imu, BMI_ACC_SOFTRESET, 0xB6);
	HAL_Delay(50);

	/* Check chip ID */
	uint8_t chipID;
	status += BMI088_ReadAccRegister(imu, BMI_ACC_CHIP_ID, &chipID);

	if (chipID != 0x1E) {

	//	return 0;

	}
	HAL_Delay(10);

	/* Configure accelerometer  */
	status += BMI088_WriteAccRegister(imu, BMI_ACC_CONF, 0xA8); /* (no oversampling, ODR = 100 Hz, BW = 40 Hz) */
	HAL_Delay(10);

	status += BMI088_WriteAccRegister(imu, BMI_ACC_RANGE, 0x00); /* +- 3g range */
	HAL_Delay(10);

	// /* Enable accelerometer data ready interrupt */
	// status += BMI088_WriteAccRegister(imu, BMI_INT1_IO_CONF, 0x0A); /* INT1 = push-pull output, active high */
	// HAL_Delay(10);

	// status += BMI088_WriteAccRegister(imu, BMI_INT1_INT2_MAP_DATA, 0x04);
	// HAL_Delay(10);

	/* Put accelerometer into active mode */
	status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CONF, 0x00);
	HAL_Delay(10);

	/* Turn accelerometer on */
	status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CTRL, 0x04);
	HAL_Delay(10);

	/* Pre-compute accelerometer conversion constant (raw to m/s^2) */
	imu->accConversion = 9.81f / 32768.0f * 2.0f * 1.5f; /* Datasheet page 27 */


	/*
	 *
	 * GYROSCOPE
	 *
	 */
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	/* Perform gyro soft reset */
	status += BMI088_WriteGyrRegister(imu, BMI_GYR_SOFTRESET, 0xB6);
	HAL_Delay(250);

	/* Check chip ID */
	status += BMI088_ReadGyrRegister(imu, BMI_GYR_CHIP_ID, &chipID);

	if (chipID != 0x0F) {

		//return 0;

	}
	HAL_Delay(10);

	/* Configure gyroscope */
	status += BMI088_WriteGyrRegister(imu, BMI_GYR_RANGE, 0x01); /* +- 1000 deg/s */
	HAL_Delay(10);

	status += BMI088_WriteGyrRegister(imu, BMI_GYR_BANDWIDTH, 0x07); /* ODR = 100 Hz, Filter bandwidth = 32 Hz */
	HAL_Delay(10);

	// /* Enable gyroscope data ready interrupt */
	// status += BMI088_WriteGyrRegister(imu, BMI_GYR_INT_CTRL, 0x80); /* New data interrupt enabled */
	// HAL_Delay(10);

	// status += BMI088_WriteGyrRegister(imu, BMI_INT3_INT4_IO_CONF, 0x01); /* INT3 = push-pull, active high */
	// HAL_Delay(10);

	// status += BMI088_WriteGyrRegister(imu, BMI_INT3_INT4_IO_MAP, 0x01); /* Data ready interrupt mapped to INT3 pin */
	// HAL_Delay(10);

	/* Pre-compute gyroscope conversion constant (raw to rad/s) */
	imu->gyrConversion = 0.01745329251f * 1000.0f / 32768.0f; /* Datasheet page 39 */

	return status;

}

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */

/* ACCELEROMETER READS ARE DIFFERENT TO GYROSCOPE READS. SEND ONE BYTE ADDRESS, READ ONE DUMMY BYTE, READ TRUE DATA !!! */
uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[3] = {regAddr | 0x80, 0x00, 0x00};
	uint8_t rxBuf[3];

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 3, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	if (status == 1) {

		*data = rxBuf[2];

	}

	return status;

}

uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr | 0x80, 0x00};
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	if (status == 1) {

		*data = rxBuf[1];

	}

	return status;

}

uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	return status;

}

uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	return status;

}



/*
 *
 * POLLING
 *
 */
uint8_t BMI088_ReadAccelerometer(BMI088 *imu) {

	/* Read raw accelerometer data */
	uint8_t txBuf[8] = {(BMI_ACC_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 1 byte dummy, 6 bytes data */
	uint8_t rxBuf[8];

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 8, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((rxBuf[3] << 8) | rxBuf[2]);
	int16_t accY = (int16_t) ((rxBuf[5] << 8) | rxBuf[4]);
	int16_t accZ = (int16_t) ((rxBuf[7] << 8) | rxBuf[6]);

	/* Convert to m/s^2 */
	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;

	return status;

}

uint8_t BMI088_ReadGyroscope(BMI088 *imu) {

	/* Read raw gyroscope data */
	uint8_t txBuf[7] = {(BMI_GYR_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 6 bytes data */
	uint8_t rxBuf[7];

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 7, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t gyrX = (int16_t) ((rxBuf[2] << 8) | rxBuf[1]);
	int16_t gyrY = (int16_t) ((rxBuf[4] << 8) | rxBuf[3]);
	int16_t gyrZ = (int16_t) ((rxBuf[6] << 8) | rxBuf[5]);

	/* Convert to rad/s */
	imu->gyr_rps[0] = imu->gyrConversion * gyrX;
	imu->gyr_rps[1] = imu->gyrConversion * gyrY;
	imu->gyr_rps[2] = imu->gyrConversion * gyrZ;

	return status;

}

/*
 *
 * DMA
 *
 */
void ASYNC_BMI088_ReadSensorDMA_init(TASK *task, BMI088 *imu) {
	ASYNC_BMI088_ReadSensorDMA_CONTEXT *context = (ASYNC_BMI088_ReadSensorDMA_CONTEXT*)task->context;

	context->imu = imu;
	context->state = ASYNC_BMI088_ReadSensorDMA_WAIT_BMI088;

	// Can't be a problem because size of txBuf is 8
	memset(context->txBuf, 0, 8);
}

void ASYNC_BMI088_ReadAccelerometerDMA(SCHEDULER *scheduler, TASK *self) {
	ASYNC_BMI088_ReadSensorDMA_CONTEXT *context = (ASYNC_BMI088_ReadSensorDMA_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_BMI088_ReadSensorDMA_WAIT_BMI088: {
		if (!(context->imu->ASYNC_busy)) {
			context->state = ASYNC_BMI088_ReadSensorDMA_START;
			context->imu->ASYNC_busy = true;
		}
		break;}
	case ASYNC_BMI088_ReadSensorDMA_START: {
		context->txBuf[0] = BMI_ACC_DATA | 0x80;
		context->dma_complete = false;
		TASK *task = add_task(scheduler, ASYNC_SPI_TxRx_DMA);
		ASYNC_SPI_TxRx_DMA_init(task,
		                        context->imu->spiHandle_flag,
								context->imu->csAccPinBank, context->imu->csAccPin,
								context->txBuf, context->rxBuf, 1, 7);
		task->is_done = &(context->dma_complete);
		context->state = ASYNC_BMI088_ReadSensorDMA_WAIT_DMA;
		break;}
	case ASYNC_BMI088_ReadSensorDMA_WAIT_DMA: {
		if (context->dma_complete) {
			context->state = ASYNC_BMI088_ReadSensorDMA_END;
		}
		break;}
	case ASYNC_BMI088_ReadSensorDMA_END: {
		BMI088_ReadAccelerometer(context->imu);	// FOR TESTING
		uint8_t *rate_buf = context->rxBuf + 1; // Skip dummy byte

		/* Form signed 16-bit integers */
		int16_t accX = (int16_t) ((rate_buf[1] << 8) | rate_buf[0]);
		int16_t accY = (int16_t) ((rate_buf[3] << 8) | rate_buf[2]);
		int16_t accZ = (int16_t) ((rate_buf[5] << 8) | rate_buf[4]);

		/* Convert to m/s^2 */
		context->imu->acc_mps2[0] = context->imu->accConversion * accX;
		context->imu->acc_mps2[1] = context->imu->accConversion * accY;
		context->imu->acc_mps2[2] = context->imu->accConversion * accZ;

		context->imu->new_acc_data = true;

		context->imu->ASYNC_busy = false;
		kill_task(scheduler, self); // End the task
		break;}
	}
}

void ASYNC_BMI088_ReadGyroscopeDMA(SCHEDULER *scheduler, TASK *self) {
	ASYNC_BMI088_ReadSensorDMA_CONTEXT *context = (ASYNC_BMI088_ReadSensorDMA_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_BMI088_ReadSensorDMA_WAIT_BMI088: {
		if (!(context->imu->ASYNC_busy)) {
			context->state = ASYNC_BMI088_ReadSensorDMA_START;
			context->imu->ASYNC_busy = true;
		}
		break;}
	case ASYNC_BMI088_ReadSensorDMA_START: {
		context->txBuf[0] = BMI_GYR_DATA | 0x80;
		context->dma_complete = false;
		TASK *task = add_task(scheduler, ASYNC_SPI_TxRx_DMA);
		ASYNC_SPI_TxRx_DMA_init(task,
		                        context->imu->spiHandle_flag,
								context->imu->csGyrPinBank, context->imu->csGyrPin,
								context->txBuf, context->rxBuf, 1, 6);
		task->is_done = &(context->dma_complete);
		context->state = ASYNC_BMI088_ReadSensorDMA_WAIT_DMA;
		break;}
	case ASYNC_BMI088_ReadSensorDMA_WAIT_DMA: {
		if (context->dma_complete) {
			context->state = ASYNC_BMI088_ReadSensorDMA_END;
		}
		break;}
	case ASYNC_BMI088_ReadSensorDMA_END: {
		uint8_t *rate_buf = context->rxBuf + 0; // No dummy byte

		/* Form signed 16-bit integers */
		int16_t gyrX = (int16_t) ((rate_buf[1] << 8) | rate_buf[0]);
		int16_t gyrY = (int16_t) ((rate_buf[3] << 8) | rate_buf[2]);
		int16_t gyrZ = (int16_t) ((rate_buf[5] << 8) | rate_buf[4]);

		/* Convert to rad/s */
		context->imu->gyr_rps[0] = context->imu->gyrConversion * gyrX;
		context->imu->gyr_rps[1] = context->imu->gyrConversion * gyrY;
		context->imu->gyr_rps[2] = context->imu->gyrConversion * gyrZ;

		context->imu->new_gyr_data = true;

		context->imu->ASYNC_busy = false;
		kill_task(scheduler, self); // End the task
		break;}
	}
}
