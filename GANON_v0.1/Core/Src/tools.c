
#include "tools.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

void float_format(char *buff, float num, int precision, int width) {
    float num_abs = num < 0 ? -num : num;

    long int num_up = (int)num_abs;
    long int num_dw = (int)((num_abs - num_up) * pow(10, precision));

    int num_up_width = width - precision - 2; // 1 for the dot and 1 for the sign
    long int max_up = pow(10, num_up_width) - 1;

    num_up = num_up > max_up ? max_up : num_up;

    if (num < 0) {
        sprintf(buff, "-%u.%u", num_up, num_dw);
    } else {
        sprintf(buff, "+%u.%u", num_up, num_dw);
    }
}


int count_nbr_elems(char buffer[], char sep) {
    int i = 1;
    for (int j = 0; buffer[j]; ++j) {
        if (buffer[j] == sep)
            ++i;
    }
    return i;
}

void set_elems_from_csv(char **elems, char buffer[], char sep, int nbr_elems) {
    int i = 0;
    int count = 0;

    buffer[strlen(buffer) - 1] = '\0';
    while (buffer[i] != '\0' && count < nbr_elems) {
        elems[count] = buffer + i;
        ++count;

        while (buffer[i] != sep && buffer[i] != '\0')
            ++i;

        if (buffer[i] == sep) {
            buffer[i] = '\0';
        }
        ++i;
    }
} 

void set_line_to_csv(char **elems, char buffer[], char sep, int nbr_elems) {
    char str_sep[2] = {sep, '\0'}; 
    buffer[0] = '\0';
    for (int i = 0; i < nbr_elems; ++i) {
        strcat(buffer, elems[i]);
        if (i < nbr_elems - 1)
            strcat(buffer, str_sep);
    }
    strcat(buffer, "\n");
}

void SPI_HandleTypeDef_flag_init(SPI_HandleTypeDef_flag *hspi_flag, SPI_HandleTypeDef* hspi) {
    hspi_flag->hspi = hspi;
    hspi_flag->is_used = false;
}

void ASYNC_SPI_TxRx_DMA_init(TASK                       *task,
                             SPI_HandleTypeDef_flag     *hspi_flag,
                             GPIO_TypeDef               *csPinBank,
                             uint16_t                    csPin,
                             uint8_t                    *tx_buf,
                             uint8_t                    *rx_buf,
                             uint16_t                    tx_size,
                             uint16_t                    rx_size) {
    ASYNC_SPI_TxRx_DMA_CONTEXT *context = (ASYNC_SPI_TxRx_DMA_CONTEXT*)task->context;

    context->hspi_flag = hspi_flag;
    context->csPinBank = csPinBank;
    context->csPin = csPin;

    context->has_started = false;

    context->tx_buf = tx_buf;
    context->rx_buf = rx_buf;
    context->tx_size = tx_size;
    context->rx_size = rx_size;

    uint16_t size = context->tx_size + context->rx_size;

    context->__tx_buf_full = (uint8_t*)malloc(sizeof(uint8_t) * size);
    context->__rx_buf_full = (uint8_t*)malloc(sizeof(uint8_t) * size);

    // Can't be a problem because tx_size < size
    memcpy(context->__tx_buf_full,
           context->tx_buf,
           context->tx_size);
}

void ASYNC_SPI_TxRx_DMA(SCHEDULER* scheduler, TASK* self) {
    ASYNC_SPI_TxRx_DMA_CONTEXT* context = (ASYNC_SPI_TxRx_DMA_CONTEXT*)self->context;

    uint16_t size = context->tx_size + context->rx_size;

    if (context->hspi_flag->hspi->State == HAL_SPI_STATE_READY) {
        if ((!context->has_started) && (!context->hspi_flag->is_used)) {
            
            context->has_started = true;
            context->hspi_flag->is_used = true;     // Lock SPI bus
            HAL_GPIO_WritePin(context->csPinBank, context->csPin, GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive_DMA(context->hspi_flag->hspi,
                                        context->__tx_buf_full,
                                        context->__rx_buf_full,
                                        size);
        } else if ((context->has_started) && (context->hspi_flag->is_used)) {
            HAL_GPIO_WritePin(context->csPinBank, context->csPin, GPIO_PIN_SET);
            context->hspi_flag->is_used = false;    // Realease SPI bus

            if (context->rx_buf) {                  // rx_buf can be NULL
                // Can't be a problem because length of __rx_buf_full + tx_size = size
                memcpy(context->rx_buf,
                       context->__rx_buf_full + context->tx_size,
                       context->rx_size);
            }
            free(context->__tx_buf_full);
            free(context->__rx_buf_full);
            kill_task(scheduler, self);
        }
    }
}
