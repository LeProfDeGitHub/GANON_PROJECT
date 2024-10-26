/**
 *******************************************
 * @file    w25q_mem.c
 * @author  Dmitriy Semenov / Crazy_Geeks
 * @version 0.1b
 * @date    12-August-2021
 * @brief   Source file for W25Qxxx lib
 * @note    https://github.com/Crazy-Geeks/STM32-W25Q-QSPI
 *******************************************
 *
 * @note https://ru.mouser.com/datasheet/2/949/w25q256jv_spi_revg_08032017-1489574.pdf
 * @note https://www.st.com/resource/en/application_note/DM00227538-.pdf
 */

 /**
  * @addtogroup W25Q_Driver
  * @{
  */

#include "drivers/w25q_mem.h"
#include <stdlib.h>
#include <string.h>




void W25Q_Init(W25Q_Chip				*w25q_chip,
			   SPI_HandleTypeDef_flag	*hspi_flag,
			   GPIO_TypeDef				*csPinBank,
			   uint16_t              	 csPin,
			   uint32_t              	 id) {
	w25q_chip->hspi_flag = hspi_flag;
	w25q_chip->csPinBank = csPinBank;
	w25q_chip->csPin = csPin;
	w25q_chip->statue = W25Q_OK;

	w25q_chip->ASYNC_busy = false;

	for (uint8_t i = 0; i < 24; i++) {
		w25q_chip->status_bits[i] = false;
	}

	uint8_t id_buf[3];
	W25Q_ReadID(w25q_chip, id_buf);

	if (w25q_chip->statue == W25Q_OK) {
		if (id_buf[0] != W25Q_MANUFACTURER_ID) {
			w25q_chip->statue = W25Q_CHIP_ERR;
		} else if (id != (uint32_t)((id_buf[1] << 8) | id_buf[2])) {
			w25q_chip->statue = W25Q_PARAM_ERR;
		} else {
			W25Q_ReadStatusReg(w25q_chip);
    		W25Q_WriteStatuReg1(w25q_chip, 0x00);
		}
	}
}

bool W25Q_IsReady(W25Q_Chip *w25q_chip) {
	W25Q_ReadStatusReg(w25q_chip);
	return !w25q_chip->status_bits[W25Q_BUSY_BIT];
}

void W25Q_WaitForReady(W25Q_Chip *w25q_chip) {
	while (!W25Q_IsReady(w25q_chip)) {}
}

void W25Q_ReadID(W25Q_Chip *w25q_chip, uint8_t *id) {
	uint8_t txBuf[1] = { W25Q_READ_JEDEC_ID };
	uint8_t rxBuf[3];

	W25Q_TransmitReceive2(w25q_chip, txBuf, rxBuf, 1, 3);

	id[0] = rxBuf[0];
	id[1] = rxBuf[1];
	id[2] = rxBuf[2];
}

void W25Q_ReadStatusReg__(W25Q_Chip *w25q_chip) {

	uint8_t txBuf[3] = { W25Q_READ_SR1, W25Q_READ_SR2, W25Q_READ_SR3 };
	uint8_t rxBuf[3];

	W25Q_TransmitReceive2(w25q_chip, txBuf + 0, rxBuf + 0, 1, 1);
	W25Q_TransmitReceive2(w25q_chip, txBuf + 1, rxBuf + 1, 1, 1);
	W25Q_TransmitReceive2(w25q_chip, txBuf + 2, rxBuf + 2, 1, 1);

	for (uint8_t i = 0; i < 8; i++) {
		w25q_chip->status_bits[i + 0] = (rxBuf[0] >> i) & 0x01;
		w25q_chip->status_bits[i + 8] = (rxBuf[1] >> i) & 0x01;
		w25q_chip->status_bits[i + 16] = (rxBuf[2] >> i) & 0x01;
	}
}

void W25Q_ReadStatusReg(W25Q_Chip *w25q_chip) {

	uint8_t txBuf[3] = { W25Q_READ_SR1, W25Q_READ_SR2, W25Q_READ_SR3 };
	uint8_t rxBuf[6];

	W25Q_TransmitReceive(w25q_chip, txBuf + 0, rxBuf + 0, 1, 1);
	W25Q_TransmitReceive(w25q_chip, txBuf + 1, rxBuf + 2, 1, 1);
	W25Q_TransmitReceive(w25q_chip, txBuf + 2, rxBuf + 4, 1, 1);

	for (uint8_t i = 0; i < 8; i++) {
		w25q_chip->status_bits[i + 0] = (rxBuf[1] >> i) & 0x01;
		w25q_chip->status_bits[i + 8] = (rxBuf[3] >> i) & 0x01;
		w25q_chip->status_bits[i + 16] = (rxBuf[5] >> i) & 0x01;
	}
}

void W25Q_WriteStatuReg1(W25Q_Chip *w25q_chip, uint8_t data) {
	uint8_t txBuf[2] = { W25Q_WRITE_SR1, data };

	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 2, 0);	
}

void W25Q_WriteStatuReg2(W25Q_Chip *w25q_chip, uint8_t data) {
	uint8_t txBuf[2] = { W25Q_WRITE_SR2, data };

	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 2, 0);	
}

void W25Q_WriteStatuReg3(W25Q_Chip *w25q_chip, uint8_t data) {
	uint8_t txBuf[2] = { W25Q_WRITE_SR3, data };

	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 2, 0);
}

void W25Q_EarseSector(W25Q_Chip *w25q_chip, uint32_t addr) {
	W25Q_WaitForReady(w25q_chip);
	if (w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
		W25Q_WriteEnable(w25q_chip);
	}

	uint8_t txBuf[4] = { W25Q_SECTOR_ERASE,
						(uint8_t)(addr >> 16),
						(uint8_t)(addr >> 8),
						(uint8_t)(addr >> 0) };
	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 4, 0);
}

void W25Q_EarseAll(W25Q_Chip *w25q_chip) {
	W25Q_WaitForReady(w25q_chip);
	if (w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
		W25Q_WriteEnable(w25q_chip);
	}

	uint8_t txBuf[1] = { W25Q_CHIP_ERASE };
	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 1, 0);
}

void W25Q_WriteEnable(W25Q_Chip *w25q_chip) {
	uint8_t txBuf[1] = { W25Q_WRITE_ENABLE };
	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 1, 0);
}

void W25Q_WriteDisable(W25Q_Chip *w25q_chip) {
	uint8_t txBuf[1] = { W25Q_WRITE_DISABLE };
	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 1, 0);
}

void W25Q_PageProgram(W25Q_Chip *w25q_chip, uint8_t *data, uint32_t addr, uint16_t data_size) {
	W25Q_WaitForReady(w25q_chip);
	if (w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
		W25Q_WriteEnable(w25q_chip);
	}

	data_size = data_size > MEM_PAGE_SIZE ? MEM_PAGE_SIZE : data_size;
	uint16_t tx_size = data_size + 4;
	// uint8_t *tx_buf = malloc(sizeof(uint8_t) * tx_size);
	uint8_t tx_buf[256 + 4];
	tx_buf[0] = W25Q_PAGE_PROGRAM;		// Command
	tx_buf[1] = (uint8_t)(addr >> 16);	// Address
	tx_buf[2] = (uint8_t)(addr >> 8);	// Address
	tx_buf[3] = (uint8_t)(addr >> 0);	// Address

	// for (uint16_t i = 0; i < data_size; i++) {
	// 	tx_buf[i + 4] = data[i];
	// }
	memcpy(tx_buf + 4, data, data_size);

	W25Q_TransmitReceive(w25q_chip, tx_buf, NULL, tx_size, 0);

	// free(tx_buf);
}

void W25Q_WriteData(W25Q_Chip *w25q_chip, uint8_t *data, uint32_t addr, uint32_t data_size) {
	uint32_t flash_size = MEM_FLASH_SIZE * 1000000; // MEM_FLASH_SIZE MBytes
	uint32_t data_size_left = (data_size + addr) > flash_size ? flash_size - addr : data_size;
	uint32_t current_addr = addr;
	uint8_t *current_data = data;

	while (data_size_left > 0) {
		uint32_t relative_addr = current_addr % MEM_PAGE_SIZE;
		uint16_t data_size_page = (data_size_left + relative_addr) > MEM_PAGE_SIZE ? MEM_PAGE_SIZE - relative_addr : data_size_left;
		W25Q_PageProgram(w25q_chip, current_data, current_addr, data_size_page);

		data_size_left -= data_size_page;
		current_addr += data_size_page;
		current_data += data_size_page;
	}
}

void W25Q_ReadData(W25Q_Chip *w25q_chip, uint8_t *data_buf, uint32_t addr, uint32_t data_size) {
	W25Q_WaitForReady(w25q_chip);

	for (uint16_t i = 0; i < data_size; i++) {
		data_buf[i] = 0;
	}

	uint8_t tx_buf[4] = { W25Q_READ_DATA,		// Command
						(uint8_t)(addr >> 16), // Address
						(uint8_t)(addr >> 8),  // Address
						(uint8_t)addr };		// Address

	W25Q_TransmitReceive2(w25q_chip, tx_buf, data_buf, 4, data_size);
}

void W25Q_TransmitReceive2(W25Q_Chip *w25q_chip, uint8_t *tx_buf, uint8_t* rx_buf, uint16_t tx_len, uint16_t rx_len) {
	uint16_t len = tx_len + rx_len;
	uint8_t *tx_buf_full = malloc(sizeof(uint8_t) * len);
	uint8_t *rx_buf_full = malloc(sizeof(uint8_t) * len);

	memcpy(tx_buf_full, tx_buf, tx_len);

	HAL_GPIO_WritePin(w25q_chip->csPinBank, w25q_chip->csPin, GPIO_PIN_RESET);
	if (rx_buf) {
		HAL_SPI_TransmitReceive(w25q_chip->hspi_flag->hspi, tx_buf_full, rx_buf_full, len, HAL_MAX_DELAY);
	} else {
		HAL_SPI_Transmit(w25q_chip->hspi_flag->hspi, tx_buf_full, tx_len, HAL_MAX_DELAY);
	}
	HAL_GPIO_WritePin(w25q_chip->csPinBank, w25q_chip->csPin, GPIO_PIN_SET);

	memcpy(rx_buf, rx_buf_full + tx_len, rx_len);

	free(tx_buf_full);
	free(rx_buf_full);
}

void W25Q_TransmitReceive(W25Q_Chip *w25q_chip, uint8_t *tx_buf, uint8_t* rx_buf, uint16_t tx_len, uint16_t rx_len) {
	uint16_t len = tx_len + rx_len;
	HAL_GPIO_WritePin(w25q_chip->csPinBank, w25q_chip->csPin, GPIO_PIN_RESET);
	if (rx_buf) {
		HAL_SPI_TransmitReceive(w25q_chip->hspi_flag->hspi, tx_buf, rx_buf, len, HAL_MAX_DELAY);
	} else {
		HAL_SPI_Transmit(w25q_chip->hspi_flag->hspi, tx_buf, tx_len, HAL_MAX_DELAY);
	}
	HAL_GPIO_WritePin(w25q_chip->csPinBank, w25q_chip->csPin, GPIO_PIN_SET);
}



void ASYNC_W25Q_ReadStatusReg_init(TASK *task, W25Q_Chip *w25q_chip) {
	ASYNC_W25Q_ReadStatusReg_CONTEXT *context = (ASYNC_W25Q_ReadStatusReg_CONTEXT*)task->context;

	context->w25q_chip = w25q_chip;

	context->state = ASYNC_W25Q_START;
	// context->read_status_state = ASYNC_W25Q_ReadStatusReg_READ_SR1;
	// can skip the waiting state if the chip is busy because this
	// task is not going to change the internal state of the chip

	memset(context->dma_complete, false, sizeof(bool) * 3);

	context->tx_buf[0] = W25Q_READ_SR1;
	context->tx_buf[1] = W25Q_READ_SR2;
	context->tx_buf[2] = W25Q_READ_SR3;
}

void ASYNC_W25Q_ReadStatusReg(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_ReadStatusReg_CONTEXT* context = (ASYNC_W25Q_ReadStatusReg_CONTEXT*)self->context;

	// uint8_t idx = (uint8_t)(context->read_status_state);

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: {
		// what ever if w25q is taken by another task
		// this task don't change the internal state of the chip
		// 
		context->state = ASYNC_W25Q_START;
		break;}
	case ASYNC_W25Q_START: {	// if SR1, SR2 or SR3, then read
		TASK *task;
		for (uint8_t i = 0; i < 3; i++) {
			task = add_task(scheduler, ASYNC_SPI_TxRx_DMA);
			ASYNC_SPI_TxRx_DMA_init(task,
									context->w25q_chip->hspi_flag,
									context->w25q_chip->csPinBank, context->w25q_chip->csPin,
									context->tx_buf + i, context->rx_buf + i, 1, 1);
			task->is_done = context->dma_complete + i;
		}
		context->state = ASYNC_W25Q_WAIT;
		break;}
	case ASYNC_W25Q_WAIT: {
		if (context->dma_complete[0] &&
		    context->dma_complete[1] &&
			context->dma_complete[2]) {
			context->state = ASYNC_W25Q_END;
		}
		break;}
	case ASYNC_W25Q_END: {
		for (uint8_t i = 0; i < 8; i++) {
			context->w25q_chip->status_bits[i +  0] = (context->rx_buf[0] >> i) & 0x01; // SR1
			context->w25q_chip->status_bits[i +  8] = (context->rx_buf[1] >> i) & 0x01; // SR2
			context->w25q_chip->status_bits[i + 16] = (context->rx_buf[2] >> i) & 0x01; // SR3
		}
		kill_task(scheduler, self);
		break;}
	}
}


void ASYNC_W25Q_WriteEnable_init(TASK *task, W25Q_Chip *w25q_chip) {
	ASYNC_W25Q_WriteEnable_CONTEXT *context = (ASYNC_W25Q_WriteEnable_CONTEXT*)task->context;
	context->w25q_chip = w25q_chip;
}

void ASYNC_W25Q_WriteEnable(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_WriteEnable_CONTEXT* context = (ASYNC_W25Q_WriteEnable_CONTEXT*)self->context;

	uint8_t tx_buf[1] = { W25Q_WRITE_ENABLE };
	TASK *task = add_task(scheduler, ASYNC_SPI_TxRx_DMA);
	ASYNC_SPI_TxRx_DMA_init(task,
							context->w25q_chip->hspi_flag,
							context->w25q_chip->csPinBank, context->w25q_chip->csPin,
							tx_buf, NULL, 1, 0);
	task->is_done = self->is_done;
	self->is_done = NULL;
	kill_task(scheduler, self);
}


void ASYNC_W25Q_WaitForReady_init(TASK *task, W25Q_Chip *w25q_chip) {
	ASYNC_W25Q_WaitForReady_CONTEXT *context = (ASYNC_W25Q_WaitForReady_CONTEXT*)task->context;

	context->w25q_chip = w25q_chip;
	context->state = ASYNC_W25Q_START;
	// can skip the waiting state if the chip is busy because this
	// task is not going to change the internal state of the chip
}

void ASYNC_W25Q_WaitForReady(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_WaitForReady_CONTEXT* context = (ASYNC_W25Q_WaitForReady_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: {
		// what ever if w25q is taken by another task
		// this task don't change the internal state of the chip
		// 
		context->state = ASYNC_W25Q_START;
		break;}
	case ASYNC_W25Q_START: {
		context->read_reg_status_done = false;
		TASK *task = add_task(scheduler, ASYNC_W25Q_ReadStatusReg);
		ASYNC_W25Q_ReadStatusReg_init(task, context->w25q_chip);
		task->is_done = &(context->read_reg_status_done);
		context->state = ASYNC_W25Q_WAIT;
		break;}
	case ASYNC_W25Q_WAIT: {
		if (context->read_reg_status_done) {
			if (context->w25q_chip->status_bits[W25Q_BUSY_BIT] == 0) {
				context->state = ASYNC_W25Q_END;
			} else {
				context->state = ASYNC_W25Q_START;
			}
		}
		break;}
	case ASYNC_W25Q_END: {
		kill_task(scheduler, self);
		break;}
	}
}


void ASYNC_W25Q_EraseSector_init(TASK *task, W25Q_Chip *w25q_chip, uint32_t addr) {
	ASYNC_W25Q_EarseSector_CONTEXT *context = (ASYNC_W25Q_EarseSector_CONTEXT*)task->context;

	context->w25q_chip = w25q_chip;
	context->addr = addr;

	context->tx_buf[0] = W25Q_SECTOR_ERASE;
	context->tx_buf[1] = (uint8_t)(context->addr >> 16);
	context->tx_buf[2] = (uint8_t)(context->addr >>  8);
	context->tx_buf[3] = (uint8_t)(context->addr >>  0);

	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
}

void ASYNC_W25Q_EraseSector(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_EarseSector_CONTEXT* context = (ASYNC_W25Q_EarseSector_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WaitAndProceed_WAIT_W25Q: {
		if (!(context->w25q_chip->ASYNC_busy)) {
			context->w25q_chip->ASYNC_busy = true;
			context->state = ASYNC_W25Q_WaitAndProceed_START;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_START: {
		context->is_ready = false;
		TASK *task = add_task(scheduler, ASYNC_W25Q_WaitForReady);
		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
		if (context->is_ready) {
			if (context->w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
				context->is_ready = false;
				TASK *task = add_task(scheduler, ASYNC_W25Q_WriteEnable);
				ASYNC_W25Q_WriteEnable_init(task, context->w25q_chip);
				task->is_done = &(context->is_ready);
				context->state = ASYNC_W25Q_WaitAndProceed_WAIT_WEL;
			} else {
				context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
			}
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {
		if (context->is_ready) {
			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_TxRx: {
		context->is_ready = false;
		TASK *task = add_task(scheduler, ASYNC_SPI_TxRx_DMA);
		ASYNC_SPI_TxRx_DMA_init(task,
								context->w25q_chip->hspi_flag,
								context->w25q_chip->csPinBank, context->w25q_chip->csPin,
								context->tx_buf, NULL, 4, 0);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_END;
		break;}
	case ASYNC_W25Q_WaitAndProceed_END: {
		if (context->is_ready) {
			context->w25q_chip->ASYNC_busy = false;
			kill_task(scheduler, self);
		}
		break;}
	}
}


void ASYNC_W25Q_EraseAll_init(TASK *task, W25Q_Chip *w25q_chip) {
	ASYNC_W25Q_EarseAll_CONTEXT *context = (ASYNC_W25Q_EarseAll_CONTEXT*)task->context;

	context->w25q_chip = w25q_chip;

	context->tx_buf[0] = W25Q_CHIP_ERASE;

	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
}

void ASYNC_W25Q_EraseAll(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_EarseAll_CONTEXT* context = (ASYNC_W25Q_EarseAll_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WaitAndProceed_WAIT_W25Q: {
		if (!(context->w25q_chip->ASYNC_busy)) {
			context->w25q_chip->ASYNC_busy = true;
			context->state = ASYNC_W25Q_WaitAndProceed_START;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_START: {
		context->is_ready = false;
		TASK *task = add_task(scheduler, ASYNC_W25Q_WaitForReady);
		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
		if (context->is_ready) {
			if (context->w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
				context->is_ready = false;
				TASK *task = add_task(scheduler, ASYNC_W25Q_WriteEnable);
				ASYNC_W25Q_WriteEnable_init(task, context->w25q_chip);
				task->is_done = &(context->is_ready);
				context->state = ASYNC_W25Q_WaitAndProceed_WAIT_WEL;
			} else {
				context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
			}
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {
		if (context->is_ready) {
			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_TxRx: {
		context->is_ready = false;
		TASK *task = add_task(scheduler, ASYNC_SPI_TxRx_DMA);
		ASYNC_SPI_TxRx_DMA_init(task,
								context->w25q_chip->hspi_flag,
								context->w25q_chip->csPinBank, context->w25q_chip->csPin,
								context->tx_buf, NULL, 1, 0);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_END;
		break;}
	case ASYNC_W25Q_WaitAndProceed_END: {
		if (context->is_ready) {
			context->w25q_chip->ASYNC_busy = false;
			kill_task(scheduler, self);
		}
		break;}
	}
}


void ASYNC_W25Q_ReadData_init(TASK 		*task,
							  W25Q_Chip *w25q_chip,
							  uint8_t   *data,
							  uint32_t   addr,
							  uint32_t   data_size) {
	ASYNC_W25Q_ReadData_CONTEXT *context = (ASYNC_W25Q_ReadData_CONTEXT*)task->context;

	context->w25q_chip = w25q_chip;

	context->data = data;
	context->addr = addr;
	context->data_size = data_size;

	context->w25q_chip->tx_buf[0] = W25Q_READ_DATA;
	context->w25q_chip->tx_buf[1] = (uint8_t)(context->addr >> 16);	// Address
	context->w25q_chip->tx_buf[2] = (uint8_t)(context->addr >>  8);	// Address
	context->w25q_chip->tx_buf[3] = (uint8_t)(context->addr >>  0);	// Address

	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
}

void ASYNC_W25Q_ReadData(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_ReadData_CONTEXT* context = (ASYNC_W25Q_ReadData_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: {
		if (!(context->w25q_chip->ASYNC_busy)) {
			context->w25q_chip->ASYNC_busy = true;
			context->state = ASYNC_W25Q_WaitAndProceed_START;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_START: {
		context->is_ready = false;
		TASK *task = add_task(scheduler, ASYNC_W25Q_WaitForReady);
		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
		if (context->is_ready) {
			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {break;} // case never happens
	case ASYNC_W25Q_WaitAndProceed_TxRx: {
		context->is_ready = false;
		TASK *task = add_task(scheduler, ASYNC_SPI_TxRx_DMA);
		ASYNC_SPI_TxRx_DMA_init(task,
								context->w25q_chip->hspi_flag,
								context->w25q_chip->csPinBank, context->w25q_chip->csPin,
								context->w25q_chip->tx_buf, context->data, 4, context->data_size);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_END;
		break;}
	case ASYNC_W25Q_WaitAndProceed_END: {
		if (context->is_ready) {
			context->w25q_chip->ASYNC_busy = false;
			kill_task(scheduler, self);
		}
		break;}
	}
}


void ASYNC_W25Q_PageProgram_init(TASK		*task,
							     W25Q_Chip  *w25q_chip,
							     uint8_t    *data,
							     uint32_t    addr,
							     uint16_t    data_size) {
	ASYNC_W25Q_PageProgram_CONTEXT *context = (ASYNC_W25Q_PageProgram_CONTEXT*)task->context;
	
	context->w25q_chip = w25q_chip;

	context->data = data;
	context->addr = addr;
	context->data_size = data_size > MEM_PAGE_SIZE ? MEM_PAGE_SIZE : data_size;

	context->tx_buf = malloc(sizeof(uint8_t) * (context->data_size + 4));

	context->tx_buf[0] = W25Q_PAGE_PROGRAM;
	context->tx_buf[1] = (uint8_t)(context->addr >> 16);	// Address
	context->tx_buf[2] = (uint8_t)(context->addr >>  8);	// Address
	context->tx_buf[3] = (uint8_t)(context->addr >>  0);	// Address

	memcpy(context->tx_buf + 4, context->data, context->data_size);

	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
}

void ASYNC_W25Q_PageProgram(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_PageProgram_CONTEXT* context = (ASYNC_W25Q_PageProgram_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WaitAndProceed_WAIT_W25Q: {
		if (!context->w25q_chip->ASYNC_busy) {
			context->state = ASYNC_W25Q_WaitAndProceed_START;
			context->w25q_chip->ASYNC_busy = true;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_START: {
		context->is_ready = false;
		TASK *task = add_task(scheduler, ASYNC_W25Q_WaitForReady);
		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
		if (context->is_ready) {
			if (context->w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
				context->is_ready = false;
				TASK *task = add_task(scheduler, ASYNC_W25Q_WriteEnable);
				ASYNC_W25Q_WriteEnable_init(task, context->w25q_chip);
				task->is_done = &(context->is_ready);
				context->state = ASYNC_W25Q_WaitAndProceed_WAIT_WEL;
			} else {
				context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
			}
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {
		if (context->is_ready) {
			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_TxRx: {
		context->is_ready = false;
		TASK *task = add_task(scheduler, ASYNC_SPI_TxRx_DMA);
		ASYNC_SPI_TxRx_DMA_init(task,
								context->w25q_chip->hspi_flag,
								context->w25q_chip->csPinBank,
								context->w25q_chip->csPin,
								context->tx_buf, NULL, context->data_size + 4, 0);
		free(context->tx_buf); // tx_buf is copied to TxRx_DMA task
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_END;
		break;}
	case ASYNC_W25Q_WaitAndProceed_END: {
		if (context->is_ready) {
			context->w25q_chip->ASYNC_busy = false;
			kill_task(scheduler, self);
		}
		break;}
	}
}


void ASYNC_W25Q_WriteData_init(TASK		  *task,
							   W25Q_Chip  *w25q_chip,
							   uint8_t    *data,
							   uint32_t    addr,
							   uint32_t    data_size) {
	ASYNC_W25Q_WriteData_CONTEXT *context = (ASYNC_W25Q_WriteData_CONTEXT*)task->context;

	context->w25q_chip = w25q_chip;

	context->n_data_blocks = data_size / MEM_PAGE_SIZE + 1;

	context->data_blocks = malloc(sizeof(uint8_t*) * context->n_data_blocks);
	context->addrs       = malloc(sizeof(uint32_t) * context->n_data_blocks);
	context->data_sizes  = malloc(sizeof(uint8_t ) * context->n_data_blocks);

	uint32_t flash_size = MEM_FLASH_SIZE * 1000000; // MEM_FLASH_SIZE MBytes
	uint32_t data_size_left = (data_size + addr) > flash_size ? flash_size - addr : data_size;
	uint32_t current_addr = addr;
	uint8_t *current_data = data;

	context->i = 0;

	while (data_size_left > 0) {
		uint32_t relative_addr = current_addr % MEM_PAGE_SIZE;
		uint16_t data_size_page = (data_size_left + relative_addr) > MEM_PAGE_SIZE ? MEM_PAGE_SIZE - relative_addr : data_size_left;
		
		context->data_blocks[context->i] = (uint8_t*)malloc(sizeof(uint8_t) * data_size_page);
		context->addrs[context->i] = current_addr;
		context->data_sizes[context->i] = data_size_page;

		memcpy(context->data_blocks[context->i], current_data, data_size_page);

		data_size_left -= data_size_page;
		current_addr += data_size_page;
		current_data += data_size_page;
		(context->i)++;
	}
	context->i = 0;
	context->state = ASYNC_W25Q_START;
}

void ASYNC_W25Q_WriteData(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_WriteData_CONTEXT* context = (ASYNC_W25Q_WriteData_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: {
		context->state = ASYNC_W25Q_START;
		break;} // case never happens
	case ASYNC_W25Q_START: {
		context->is_ready = false;
		TASK *task = add_task(scheduler, ASYNC_W25Q_PageProgram);
		ASYNC_W25Q_PageProgram_init(task,
									context->w25q_chip,
									context->data_blocks[context->i],
									context->addrs[context->i],
									context->data_sizes[context->i]);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WAIT;
		break;}
	case ASYNC_W25Q_WAIT: {
		if (context->is_ready) {
			free(context->data_blocks[context->i]);
			context->data_blocks[context->i] = NULL;
			context->i++;
			if (context->i < context->n_data_blocks) {
				context->state = ASYNC_W25Q_START;
			} else {
				context->state = ASYNC_W25Q_END;
			}
		}
		break;}
	case ASYNC_W25Q_END: {
		free(context->data_blocks);
		free(context->addrs);
		free(context->data_sizes);
		kill_task(scheduler, self);
		break;}
	}
}


