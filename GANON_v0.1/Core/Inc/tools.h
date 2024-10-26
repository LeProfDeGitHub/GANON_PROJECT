
#ifndef TOOLS_H
#define TOOLS_H

#include "stm32f4xx_hal.h"
#include "scheduler.h"

typedef union FLOAT_U32_UNION {
    float    float_type;
    uint32_t uint32_type;
} FLOAT_U32_UNION;

typedef struct SPI_HandleTypeDef_flag {
    SPI_HandleTypeDef   *hspi;
    bool                 is_used;
} SPI_HandleTypeDef_flag;

typedef enum ASYNC_WAITER_STATE {
	ASYNC_START,
	ASYNC_WAIT,
	ASYNC_END
} ASYNC_WAITER_STATE;

typedef struct ASYNC_SPI_TxRx_DMA_CONTEXT {
    SPI_HandleTypeDef_flag* hspi_flag;
    GPIO_TypeDef *csPinBank;
    uint16_t csPin;

    bool has_started;

    uint8_t *tx_buf;
    uint8_t *rx_buf;

    uint8_t *__tx_buf_full;
    uint8_t *__rx_buf_full;

    uint16_t tx_size;
    uint16_t rx_size;

    uint32_t next_time;

#ifdef DEBUG_SCHEDULER
    TASK *owner_task;
#endif // DEBUG_SCHEDULER
} ASYNC_SPI_TxRx_DMA_CONTEXT;



void float_format(char* buff, float num, int precision, int width);


// Renvoie le nombre de champ d'une ligne type csv.
int count_nbr_elems(char buffer[], char sep);

// Modifie un [buffer] caracterisant la ligne d'un fichier de csv afin de decouper les differentes chaines
// de caracteres. Remplie un tableau de pointeur pointant vers ces differentes chaines de caracteres.
void set_elems_from_csv(char **elems, char buffer[], char sep, int nbr_elems);

// Remplie un [buffer] caracterisant la ligne d'un fichier de csv a partir d'un tableau de
// pointeur pointant vers les differentes chaines de caracteres.
void set_line_to_csv(char **elems, char buffer[], char sep, int nbr_elems);

void SPI_HandleTypeDef_flag_init(SPI_HandleTypeDef_flag *hspi_flag, SPI_HandleTypeDef* hspi);


void ASYNC_SPI_TxRx_DMA_init(TASK                       *task,
                             SPI_HandleTypeDef_flag     *hspi_flag,
                             GPIO_TypeDef               *csPinBank,
                             uint16_t                    csPin,
                             uint8_t                    *tx_buf,
                             uint8_t                    *rx_buf,
                             uint16_t                    tx_size,
                             uint16_t                    rx_size);

void ASYNC_SPI_TxRx_DMA(SCHEDULER *scheduler, TASK *self);

#endif // TOOLS_H