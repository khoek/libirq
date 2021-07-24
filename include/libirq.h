#ifndef __LIB_LIBIRQ_H
#define __LIB_LIBIRQ_H

#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <stdint.h>

typedef struct irq_task irq_task_t;
typedef irq_task_t* irq_task_handle_t;

typedef void (*handle_irq_fn_t)(void* private);

esp_err_t libirq_create(gpio_num_t pin_irq, bool trigger_high, handle_irq_fn_t handle_irq, void* private,
                        BaseType_t core_id, uint32_t task_stack_size, irq_task_handle_t* out_handle);

// Wakes the associated task, causing `handle_irq()` to be called despite the IRQ not actually being
// asserted. If an actual IRQ also arrives while the wake message is being dispatched, there is no
// guarentee that `handle_irq()` will be called twice.
void libirq_wake(irq_task_handle_t task);

// This function may be called at most once. It may not be called from the provided `handle_irq()`
// function (this will result in a deadlock).
void libirq_destroy(irq_task_handle_t task);

#endif
