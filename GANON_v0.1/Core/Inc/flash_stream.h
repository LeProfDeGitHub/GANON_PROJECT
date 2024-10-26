
#include "stm32f4xx_hal.h"
#include "drivers/w25q_mem.h"


#define DATA_NUMBER             11 // 3 accel + 3 gyro + 1 temp + 1 pressure + 1 longitude + 1 latitude + 1 time
#define DATA_SIZE               DATA_NUMBER * sizeof(float)

#define LAST_SECTOR_ADDR        0xFFF000

#define SECTOR_SIZE             0x1000
#define INFO_SIZE               2*sizeof(uint32_t) + sizeof(uint8_t)

#define NUM_INFO                ((int)SECTOR_SIZE) / ((int)INFO_SIZE)
#define LAST_INFO_ADDR          LAST_SECTOR_ADDR + ((int)NUM_INFO - 1) * ((int)INFO_SIZE)


typedef struct FLASH_STREAM {
    W25Q_Chip   *flash_chip;

    uint32_t     write_ptr;
    uint32_t     read_ptr;

    uint32_t     last_info_ptr;
} FLASH_STREAM;

typedef struct ASYNC_fs_read_write_CONTEXT {
    FLASH_STREAM    *stream;
    uint8_t         *data;
    uint16_t         len;
} ASYNC_fs_read_write_CONTEXT;

typedef struct ASYNC_fs_read_write_float_CONTEXT {
    FLASH_STREAM       *stream;
    float              *data;
    uint16_t            len;
    uint8_t            *__buffer;

    bool                dma_complete;
    ASYNC_WAITER_STATE  state;
} ASYNC_fs_read_write_float_CONTEXT;

void flash_stream_init(FLASH_STREAM* stream, W25Q_Chip* flash_chip);

void flash_stream_write(FLASH_STREAM* stream, uint8_t* data, uint16_t len);
void flash_stream_read(FLASH_STREAM* stream, uint8_t* data, uint16_t len);
void flash_stream_write_float(FLASH_STREAM* stream, float data);
void flash_stream_read_float(FLASH_STREAM* stream, float* data);
void flash_stream_write_floats(FLASH_STREAM* stream, float* data, uint16_t len);
void flash_stream_read_floats(FLASH_STREAM* stream, float* data, uint16_t len);

void ASYNC_fs_read_write_init(TASK *task, FLASH_STREAM *stream,
                              uint8_t *data, uint16_t len);
void ASYNC_fs_write(SCHEDULER *scheduler, TASK *self);
void ASYNC_fs_read(SCHEDULER *scheduler, TASK *self);

void ASYNC_fs_read_write_floats_init(TASK *task, FLASH_STREAM *stream,
                                    float *data, uint16_t len);
void ASYNC_fs_write_floats(SCHEDULER *scheduler, TASK *self);
void ASYNC_fs_read_floats(SCHEDULER *scheduler, TASK *self);
