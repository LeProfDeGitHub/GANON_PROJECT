#ifndef SCHEDULER_H
#define SCHEDULER_H

// #define CHANGE_DATA_TYPE

#include <stdbool.h>

#include "stm32f4xx_hal.h"
// #include "tools.h"


#define DEBUG_SCHEDULER 1


// !!! ATTENTION, A METTRE A JOUR !!!
#define ASYNC_CONTEXT_DEFAULT_BYTES_SIZE 64 // ATTENTION, A METTRE A JOUR !!!
#define MAX_TASKS_NBR 50 // !!! ATTENTION, A METTRE A JOUR !!!
// !!! ATTENTION, A METTRE A JOUR !!!

// fonction a executer (avec le scheduler et le contexte)
typedef void(*__task_func_t)(void*, void*);

typedef struct {
    __task_func_t func;     // fonction a executer (avec le scheduler et sa tache référente)
    size_t idx;             // index de la tache
    bool *is_done;          // pointeur vers un booléen pour transmettre l'etat de la tache
    uint8_t context[ASYNC_CONTEXT_DEFAULT_BYTES_SIZE]; // contexte de la tache
} TASK;

typedef struct {
    size_t length;
    size_t current_idx;
    size_t nbr_has_run_task;
    TASK  *tasks[MAX_TASKS_NBR];
    bool   running[MAX_TASKS_NBR];
    size_t nbr_cycles;
} SCHEDULER;

typedef void(*task_func_t)(SCHEDULER*, TASK*);


void init_scheduler(SCHEDULER *scheduler);

TASK *add_task(SCHEDULER *scheduler, task_func_t func);

void kill_task(SCHEDULER *scheduler, TASK *task);

void run_task(SCHEDULER *scheduler, TASK *task);

void run_scheduler(SCHEDULER *scheduler);

#endif