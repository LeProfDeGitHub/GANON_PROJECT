
#include "drivers/buzzer.h"
#include "scheduler.h"

#include <string.h>
#include <math.h>

void BUZZER_Init(BUZZER *buzzer, TIM_HandleTypeDef *htim, uint32_t channel) {
    buzzer->htim = htim;
    buzzer->channel = channel;
    buzzer->ASYNC_busy = false;
}

float get_note_freq(note_t note, octave_t octave) {
    if (note == NOTE_NOP) {
        return 0;
    }
    return NOTE_A4 * pow(2.0, ((float)note / (float)12) + (float)octave);
}

void set_prescaler_by_freq(TIM_HandleTypeDef* htim, float freq) {
    uint32_t timer_freq = HAL_RCC_GetPCLK1Freq();
    uint32_t periode = htim->Init.Period;
    uint32_t prescaler;
    if (freq == 0) {
        prescaler = 0;
    } else {
        prescaler = timer_freq / ((periode * (uint32_t)freq) - 1);
    }
    __HAL_TIM_SET_PRESCALER(htim, prescaler);
}

void play_note(BUZZER *buzzer, float* freqs, uint32_t* duration, int size) {
    set_prescaler_by_freq(buzzer->htim, 0);
    HAL_TIM_PWM_Start(buzzer->htim, buzzer->channel);
    for (int i = 0; i < size; i++) {
        set_prescaler_by_freq(buzzer->htim, freqs[i]);
        HAL_Delay(duration[i]);
    }
    set_prescaler_by_freq(buzzer->htim, 0);
    HAL_TIM_PWM_Stop(buzzer->htim, buzzer->channel);
}

void ASYNC_play_note_init(TASK *task, BUZZER *buzzer, float *freqs, uint32_t *durations, int size) {
    ASYNC_play_note_CONTEXT *context = (ASYNC_play_note_CONTEXT *)(task->context);
    
    context->buzzer = buzzer;
    context->freqs = freqs;
    context->durations = durations;
    context->size = size;
    context->index = 0;
    context->next_time = 0;
    context->state = ASYNC_play_note_WAIT_BUZZER;
}

#ifdef BUZZER_ACTIVE

void ASYNC_play_note(SCHEDULER *scheduler, TASK *self) {
    uint32_t current_time = HAL_GetTick();
    ASYNC_play_note_CONTEXT *context = (ASYNC_play_note_CONTEXT *)(self->context);

    switch (context->state) {
    case ASYNC_play_note_WAIT_BUZZER: {
        if (!(context->buzzer->ASYNC_busy) && (context->buzzer->htim->State == HAL_TIM_STATE_READY)) {
            context->state = ASYNC_play_note_START;
            context->buzzer->ASYNC_busy = true;
        }
        break; }
    case ASYNC_play_note_START: {
        set_prescaler_by_freq(context->buzzer->htim, 0);
        HAL_TIM_PWM_Start(context->buzzer->htim, context->buzzer->channel);
        context->state = ASYNC_play_note_PLAY;
        break; }
    case ASYNC_play_note_PLAY: {
        if (current_time >= context->next_time) {
            if (context->index < context->size) {
                set_prescaler_by_freq(context->buzzer->htim, context->freqs[context->index]);
                context->next_time = HAL_GetTick() + context->durations[context->index];
                context->index++;
            } else {
                set_prescaler_by_freq(context->buzzer->htim, 0);
                HAL_TIM_PWM_Stop(context->buzzer->htim, context->buzzer->channel);
                context->state = ASYNC_play_note_END;
            }
        }
        break; }
    case ASYNC_play_note_END: {
        context->buzzer->ASYNC_busy = false;
        kill_task(scheduler, self);
        break; }
    }
}

#else

void ASYNC_play_note(SCHEDULER *scheduler, TASK *self) {
    kill_task(scheduler, self);
}

#endif // BUZZER_ACTIVE


void set_song_bank(SONG_BANK *song_bank) {
    set_nggyu_song(song_bank->nggyu_freqs, song_bank->nggyu_durations);
    set_cccp_song(song_bank->cccp_freqs, song_bank->cccp_durations);
    set_beep_0_song(song_bank->beep_0_freqs, song_bank->beep_0_durations);
    set_beep_1_song(song_bank->beep_1_freqs, song_bank->beep_1_durations);
    set_beep_2_song(song_bank->beep_2_freqs, song_bank->beep_2_durations);
    set_beep_3_song(song_bank->beep_3_freqs, song_bank->beep_3_durations);
}


void set_nggyu_song(float *freqs, uint32_t *durations) {
    float _freqs[NGGYU_SONG_SIZE] = {
        get_note_freq(NOTE_C, OCTAVE_4),
        get_note_freq(NOTE_D, OCTAVE_4),
        get_note_freq(NOTE_G, OCTAVE_3),
        get_note_freq(NOTE_D, OCTAVE_4),
        get_note_freq(NOTE_E, OCTAVE_4),
        get_note_freq(NOTE_G, OCTAVE_4),
        get_note_freq(NOTE_F, OCTAVE_4),
        get_note_freq(NOTE_E, OCTAVE_4),
        get_note_freq(NOTE_C, OCTAVE_4),
        get_note_freq(NOTE_D, OCTAVE_4),
        get_note_freq(NOTE_G, OCTAVE_3),
        get_note_freq(NOTE_NOP, 0),
        get_note_freq(NOTE_G, OCTAVE_3),
        get_note_freq(NOTE_A, OCTAVE_3),

        get_note_freq(NOTE_C, OCTAVE_4),
        get_note_freq(NOTE_D, OCTAVE_4),
        get_note_freq(NOTE_G, OCTAVE_3),
        get_note_freq(NOTE_D, OCTAVE_4),
        get_note_freq(NOTE_E, OCTAVE_4),
    };

    uint32_t _durations[NGGYU_SONG_SIZE] = {
        1000,
        1000,
         500,
        1000,
        1000,
         250,
         250,
         250,
        1000,
        1000,
         500,
        1000,
         250,
         250,

        1000,
        1000,
         500,
        1000,
        1000,
    };

    for (int i = 0; i < NGGYU_SONG_SIZE; i++) {
        freqs[i] = _freqs[i];
        durations[i] = _durations[i];
    }
}

void set_cccp_song(float *freqs, uint32_t *durations) {
    float _freqs[CCCP_SONG_SIZE] = {
        get_note_freq(NOTE_As, OCTAVE_3),
        get_note_freq(NOTE_Ds, OCTAVE_4),
        get_note_freq(NOTE_As, OCTAVE_3),
        get_note_freq(NOTE_C , OCTAVE_4),
        get_note_freq(NOTE_D , OCTAVE_4),
        get_note_freq(NOTE_G , OCTAVE_3),
        0,
        get_note_freq(NOTE_G , OCTAVE_3),
        get_note_freq(NOTE_C , OCTAVE_4),
        get_note_freq(NOTE_As, OCTAVE_3),
        get_note_freq(NOTE_Gs, OCTAVE_3),
        get_note_freq(NOTE_As, OCTAVE_3),
        get_note_freq(NOTE_Ds, OCTAVE_3),
        get_note_freq(NOTE_F , OCTAVE_3),
        0,
        get_note_freq(NOTE_F , OCTAVE_3),
        get_note_freq(NOTE_G , OCTAVE_3),
        get_note_freq(NOTE_Gs, OCTAVE_3),
        0,
        get_note_freq(NOTE_Gs, OCTAVE_3),
        get_note_freq(NOTE_As, OCTAVE_3),
        get_note_freq(NOTE_C , OCTAVE_4),
        0,
        get_note_freq(NOTE_D , OCTAVE_4),
        get_note_freq(NOTE_Ds, OCTAVE_4),
        get_note_freq(NOTE_F , OCTAVE_4),
    };

    uint32_t _durations[CCCP_SONG_SIZE] = {
         500,
        1000,
         750,
         250,
         750,
         475,
          50,
         475,
         750,
         750,
         250,
         750,
         750,
         725,
          50,
         475,
         500,
         725,
          50,
         475,
         500,
         725,
          50,
         475,
         500,
         750,
    };

    for (int i = 0; i < CCCP_SONG_SIZE; i++) {
        freqs[i] = _freqs[i];
        durations[i] = _durations[i];
    }
}

void set_beep_0_song(float *freqs, uint32_t *durations) {
    float _freqs[BEEP_SONG_SIZE] = {
        get_note_freq(NOTE_C, OCTAVE_4),
        get_note_freq(NOTE_E, OCTAVE_4),
        get_note_freq(NOTE_G, OCTAVE_4),
    };

    uint32_t _durations[BEEP_SONG_SIZE] = {
        500,
        300,
        500,
    };

    for (int i = 0; i < BEEP_SONG_SIZE; i++) {
        freqs[i] = _freqs[i];
        durations[i] = _durations[i];
    }
}

void set_beep_1_song(float *freqs, uint32_t *durations) {
    float _freqs[BEEP_SONG_SIZE] = {
        get_note_freq(NOTE_E, OCTAVE_4),
        0,
        get_note_freq(NOTE_E, OCTAVE_4),
    };

    uint32_t _durations[BEEP_SONG_SIZE] = {
        250,
        250,
        250,
    };

    for (int i = 0; i < BEEP_SONG_SIZE; i++) {
        freqs[i] = _freqs[i];
        durations[i] = _durations[i];
    }
}

void set_beep_2_song(float *freqs, uint32_t *durations) {
    float _freqs[BEEP_SONG_SIZE] = {
        get_note_freq(NOTE_E, OCTAVE_4),
        get_note_freq(NOTE_G, OCTAVE_4),
        get_note_freq(NOTE_G, OCTAVE_4),
    };

    uint32_t _durations[BEEP_SONG_SIZE] = {
        250,
        250,
        250,
    };

    for (int i = 0; i < BEEP_SONG_SIZE; i++) {
        freqs[i] = _freqs[i];
        durations[i] = _durations[i];
    }
}

void set_beep_3_song(float *freqs, uint32_t *durations) {
    float _freqs[BEEP_SONG_SIZE] = {
        get_note_freq(NOTE_G, OCTAVE_5),
        0,
        get_note_freq(NOTE_G, OCTAVE_5),
    };

    uint32_t _durations[BEEP_SONG_SIZE] = {
        250,
        250,
        250,
    };

    for (int i = 0; i < BEEP_SONG_SIZE; i++) {
        freqs[i] = _freqs[i];
        durations[i] = _durations[i];
    }
}
