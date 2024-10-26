
#include <stdlib.h>
#include <string.h>

#include "scheduler.h"

void init_scheduler(SCHEDULER *scheduler) {
    scheduler->length = 0;
    scheduler->current_idx = 0;
    scheduler->nbr_has_run_task = 0;
    scheduler->tasks[0] = (TASK*)malloc(MAX_TASKS_NBR * sizeof(TASK));
    uint32_t tasks_size = MAX_TASKS_NBR * sizeof(TASK);
    for (int i = 0; i < MAX_TASKS_NBR; i++) {
        scheduler->running[i] = false;
        // fill the array with the right addresses
        scheduler->tasks[i] = scheduler->tasks[0] + i;
    }
    scheduler->nbr_cycles = 0;
}

TASK *add_task(SCHEDULER *scheduler, task_func_t func) {
    TASK *task = NULL;
    for (int i = 0; i < MAX_TASKS_NBR; i++) {
        if (!scheduler->running[i]) {   // empty slot found
            if (scheduler->length == 20) {
                __NOP(); // debug breakpoint
            }
            scheduler->running[i] = true;
            task = scheduler->tasks[i];
            // initialize the task
            task->func = (__task_func_t)func;
            task->idx = i;
            task->is_done = NULL;
            // don't forget to increment the number of
            // active tasks in the scheduler
            scheduler->length++;
            break;
        }
    }
    return task;
}

void kill_task(SCHEDULER *scheduler, TASK *task) {
    scheduler->running[task->idx] = false;
    if (task->is_done) {        // if is_done ptr is set (not NULL)
        *task->is_done = true;  // then share the result with the owner
                                // of *is_done
    }
    // don't forget to decrement the number of
    // active tasks in the scheduler
    scheduler->length--;
}

void run_task(SCHEDULER *scheduler, TASK *task) {
    task->func(scheduler, task);
}

void run_scheduler(SCHEDULER *scheduler) {
    if (scheduler->length > 0) {                                // if there are active tasks
        if (scheduler->nbr_has_run_task >= scheduler->length) { // if all tasks have run
            scheduler->current_idx = 0;
            scheduler->nbr_has_run_task = 0;
            scheduler->nbr_cycles++;
        }
        for (; scheduler->current_idx < MAX_TASKS_NBR; scheduler->current_idx++) {
            if (scheduler->running[scheduler->current_idx]) {
                break;
            }
        }
        if (scheduler->current_idx >= MAX_TASKS_NBR) {      // if no active task found (should not happen ???)
            scheduler->current_idx = 0;
            scheduler->nbr_cycles++;
        }
        TASK *task = scheduler->tasks[scheduler->current_idx];
        run_task(scheduler, task);
        scheduler->current_idx++;
        scheduler->nbr_has_run_task++;
    }
}
