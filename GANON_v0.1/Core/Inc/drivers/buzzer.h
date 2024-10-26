#ifndef BUZZER_H
#define BUZZER_H

#include "stm32f4xx_hal.h"
#include "scheduler.h"

#define NOTE_A4 440

#define NGGYU_SONG_SIZE 19
#define CCCP_SONG_SIZE 26
#define BEEP_SONG_SIZE 3


// #define BUZZER_ACTIVE 1


typedef enum {
    NOTE_NOP = -10,
    NOTE_C   = -9,
    NOTE_Cs  = -8,
    NOTE_D   = -7,
    NOTE_Ds  = -6,
    NOTE_E   = -5,
    NOTE_F   = -4,
    NOTE_Fs  = -3,
    NOTE_G   = -2,
    NOTE_Gs  = -1,
    NOTE_A   =  0,   // Central note (A4 = 440 Hz)
    NOTE_As  = +1,
    NOTE_B   = +2
} note_t;

typedef enum {
    OCTAVE_0 = -4,
    OCTAVE_1 = -3,
    OCTAVE_2 = -2,
    OCTAVE_3 = -1,
    OCTAVE_4 =  0,  // Central octave (A4 = 440 Hz)
    OCTAVE_5 = +1,
    OCTAVE_6 = +2,
    OCTAVE_7 = +3,
    OCTAVE_8 = +4
} octave_t;

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;

    bool ASYNC_busy;
} BUZZER;

typedef struct {
    float nggyu_freqs[NGGYU_SONG_SIZE];
    uint32_t nggyu_durations[NGGYU_SONG_SIZE];

    float cccp_freqs[CCCP_SONG_SIZE];
    uint32_t cccp_durations[CCCP_SONG_SIZE];

    float beep_0_freqs[BEEP_SONG_SIZE];
    uint32_t beep_0_durations[BEEP_SONG_SIZE];

    float beep_1_freqs[BEEP_SONG_SIZE];
    uint32_t beep_1_durations[BEEP_SONG_SIZE];

    float beep_2_freqs[BEEP_SONG_SIZE];
    uint32_t beep_2_durations[BEEP_SONG_SIZE];

    float beep_3_freqs[BEEP_SONG_SIZE];
    uint32_t beep_3_durations[BEEP_SONG_SIZE];
} SONG_BANK;

typedef enum ASYNC_play_note_STATE {
    ASYNC_play_note_WAIT_BUZZER,
    ASYNC_play_note_START,
    ASYNC_play_note_PLAY,
    ASYNC_play_note_END,
} ASYNC_play_note_STATE;

typedef struct {
    BUZZER   *buzzer;
    float    *freqs;
    uint32_t *durations;
    size_t    size;
    size_t    index;
    uint32_t  next_time;

    ASYNC_play_note_STATE state;
} ASYNC_play_note_CONTEXT;

void BUZZER_Init(BUZZER *buzzer, TIM_HandleTypeDef *htim, uint32_t channel);

float get_note_freq(note_t note, octave_t octave);

void set_prescaler_by_freq(TIM_HandleTypeDef *htim, float freq);

void play_note(BUZZER *buzzer, float *freqs, uint32_t *duration, int size);

void ASYNC_play_note_init(TASK *task, BUZZER *buzzer, float* freqs, uint32_t* durations, int size);
void ASYNC_play_note(SCHEDULER *scheduler, TASK *self);

void set_song_bank(SONG_BANK *song_bank);

void set_nggyu_song(float *freqs, uint32_t *durations);
void set_cccp_song(float *freqs, uint32_t *durations);
void set_beep_0_song(float *freqs, uint32_t *durations);
void set_beep_1_song(float *freqs, uint32_t *durations);
void set_beep_2_song(float *freqs, uint32_t *durations);
void set_beep_3_song(float *freqs, uint32_t *durations);

#endif /* BUZZER_H */