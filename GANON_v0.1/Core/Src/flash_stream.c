#include "flash_stream.h"
#include <stdlib.h>


void flash_stream_init(FLASH_STREAM* stream, W25Q_Chip* flash_chip) {
    stream->flash_chip = flash_chip;
    stream->write_ptr = 0;
    stream->read_ptr = 0;

    // flash_stream_find_last_info_ptr(stream);
}


void flash_stream_write(FLASH_STREAM* stream, uint8_t* data, uint16_t len) {
    if (stream->write_ptr + DATA_SIZE < LAST_SECTOR_ADDR) {
        W25Q_PageProgram(stream->flash_chip, data, stream->write_ptr, len);
        stream->write_ptr += len;
    }
}

void flash_stream_read(FLASH_STREAM* stream, uint8_t* data, uint16_t len) {
    W25Q_ReadData(stream->flash_chip, data, stream->read_ptr, len);
    stream->read_ptr += len;
}

void flash_stream_write_float(FLASH_STREAM* stream, float data) {
    FLOAT_U32_UNION data_float_u32 = { .float_type = data };
    uint8_t flash_buffer[4] = {data_float_u32.uint32_type >> 24,
                               data_float_u32.uint32_type >> 16,
                               data_float_u32.uint32_type >> 8,
                               data_float_u32.uint32_type >> 0 };
    flash_stream_write(stream, flash_buffer, 4);
}

void flash_stream_read_float(FLASH_STREAM* stream, float* data) {
    uint8_t flash_buffer[4];
    FLOAT_U32_UNION data_float_u32;
    flash_stream_read(stream, flash_buffer, 4);
    data_float_u32.uint32_type = (flash_buffer[0] << 24 |
                                  flash_buffer[1] << 16 |
                                  flash_buffer[2] << 8 |
                                  flash_buffer[3] << 0);
    *data = data_float_u32.float_type;
}

void flash_stream_write_floats(FLASH_STREAM* stream, float* data, uint16_t len) {
    uint8_t flash_buffer[256];

    for (int i = 0; i < len; i++) {
        FLOAT_U32_UNION data_float_u32 = { .float_type = data[i] };
        flash_buffer[i * 4 + 0] = data_float_u32.uint32_type >> 24;
        flash_buffer[i * 4 + 1] = data_float_u32.uint32_type >> 16;
        flash_buffer[i * 4 + 2] = data_float_u32.uint32_type >> 8;
        flash_buffer[i * 4 + 3] = data_float_u32.uint32_type >> 0;
    }

    flash_stream_write(stream, flash_buffer, len * 4);
}

void flash_stream_read_floats(FLASH_STREAM* stream, float* data, uint16_t len) {
    uint8_t flash_buffer[256];

    flash_stream_read(stream, flash_buffer, len * 4);

    for (int i = 0; i < len; i++) {
        FLOAT_U32_UNION data_float_u32;
        data_float_u32.uint32_type = (flash_buffer[i * 4 + 0] << 24 |
                                      flash_buffer[i * 4 + 1] << 16 |
                                      flash_buffer[i * 4 + 2] << 8 |
                                      flash_buffer[i * 4 + 3] << 0);
        data[i] = data_float_u32.float_type;
    }
}


void ASYNC_fs_read_write_init(TASK *task, FLASH_STREAM *stream,
                              uint8_t *data, uint16_t len) {
    ASYNC_fs_read_write_CONTEXT *context = (ASYNC_fs_read_write_CONTEXT*)task->context;
    
    uint32_t flash_size = MEM_FLASH_SIZE * 1000000;

    context->len = (context->stream->write_ptr + context->len < flash_size) ?
                   context->len : flash_size - context->stream->write_ptr;

    context->stream = stream;
    context->data = data;
    context->len = len;
}

void ASYNC_fs_write(SCHEDULER *scheduler, TASK *self) {
    ASYNC_fs_read_write_CONTEXT* context = (ASYNC_fs_read_write_CONTEXT*)self->context;

    TASK *task = add_task(scheduler, ASYNC_W25Q_WriteData);
    ASYNC_W25Q_WriteData_init(task,
                              context->stream->flash_chip,
                              context->data,
                              context->stream->write_ptr,
                              context->len);
    task->is_done = self->is_done;
    self->is_done = NULL;

    context->stream->write_ptr += context->len;

    kill_task(scheduler, self);
}

void ASYNC_fs_read(SCHEDULER *scheduler, TASK *self) {
    ASYNC_fs_read_write_CONTEXT *context = (ASYNC_fs_read_write_CONTEXT*)self->context;

    TASK *task = add_task(scheduler, ASYNC_W25Q_ReadData);
    ASYNC_W25Q_ReadData_init(task,
                                context->stream->flash_chip,
                                context->data,
                                context->stream->read_ptr,
                                context->len);
    task->is_done = self->is_done;
    self->is_done = NULL;

    context->stream->read_ptr += context->len;

    kill_task(scheduler, self);
}


void ASYNC_fs_read_write_floats_init(TASK *task, FLASH_STREAM *stream,
                                    float *data, uint16_t len) {
    ASYNC_fs_read_write_float_CONTEXT *context = (ASYNC_fs_read_write_float_CONTEXT*)task->context;
    
    context->stream = stream;
    context->data = data;
    context->len = len;

    context->__buffer = (uint8_t*)malloc(sizeof(uint8_t) * len * 4);

    context->dma_complete = false;
    context->state = ASYNC_START;
}

void ASYNC_fs_write_floats(SCHEDULER *scheduler, TASK *self) {
    ASYNC_fs_read_write_float_CONTEXT *context = (ASYNC_fs_read_write_float_CONTEXT*)self->context;

    FLOAT_U32_UNION data_float_u32;

    switch (context->state) {
    case ASYNC_START: {
        for (int i = 0; i < context->len; i++) {
            data_float_u32.float_type = context->data[i];

            context->__buffer[i * 4 + 0] = data_float_u32.uint32_type >> 24;
            context->__buffer[i * 4 + 1] = data_float_u32.uint32_type >> 16;
            context->__buffer[i * 4 + 2] = data_float_u32.uint32_type >>  8;
            context->__buffer[i * 4 + 3] = data_float_u32.uint32_type >>  0;
        }
        TASK *task = add_task(scheduler, ASYNC_fs_write);
        ASYNC_fs_read_write_init(task,
                                 context->stream,
                                 context->__buffer,
                                 context->len * 4);
        task->is_done = &(context->dma_complete);
        context->state = ASYNC_WAIT;
        break;}
    case ASYNC_WAIT: {
        if (context->dma_complete) {
            context->state = ASYNC_END;
        }
        break;}
    case ASYNC_END: {
        free(context->__buffer);
        kill_task(scheduler, self);
        break;}
    }
}

void ASYNC_fs_read_floats(SCHEDULER *scheduler, TASK *self) {
    ASYNC_fs_read_write_float_CONTEXT* context = (ASYNC_fs_read_write_float_CONTEXT*)self->context;

    FLOAT_U32_UNION data_float_u32;

    switch (context->state) {
    case ASYNC_START: {
        TASK *task = add_task(scheduler, ASYNC_fs_read);
        ASYNC_fs_read_write_init(task,
                                 context->stream,
                                 context->__buffer,
                                 context->len * 4);
        task->is_done = &(context->dma_complete);
        context->state = ASYNC_WAIT;
        break;}
    case ASYNC_WAIT: {
        if (context->dma_complete) {
            context->state = ASYNC_END;
        }
        break;}
    case ASYNC_END: {
        for (int i = 0; i < context->len; i++) {
            data_float_u32.uint32_type = (context->__buffer[i * 4 + 0] << 24 |
                                          context->__buffer[i * 4 + 1] << 16 |
                                          context->__buffer[i * 4 + 2] <<  8 |
                                          context->__buffer[i * 4 + 3] <<  0);
            context->data[i] = data_float_u32.float_type;
        }
        free(context->__buffer);
        kill_task(scheduler, self);
        break;}
    }
}

