#pragma once

#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <libtask.h>
#include <stdint.h>

typedef struct libirq_source libirq_source_t;
typedef libirq_source_t* libirq_source_handle_t;

typedef struct libirq_waiter {
    libirq_source_handle_t __internal;
} libirq_waiter_t;

// Creates an IRQ source on the given GPIO pin and with the specified level,
// which is processed on the specified core. In order to use the generated
// `irq_source_handle_t` you must create one or more waiters using
// `libirq_waiter_sleep_until_active()`. Each waiter behaves identically, but
// generally cannot be shared between threads---if a call to
// `libirq_waiter_sleep_until_active()` ever returns `false` it may never be
// used again (this only happens when the underlying IRQ source has been
// destroyed), and therefore it is unsafe to share a single waiter between
// threads where one thread could pass the waiter to
// `libirq_waiter_sleep_until_active()` after the same function call has
// returned `false` on another thread.
esp_err_t libirq_source_create(gpio_num_t pin_irq, bool trigger_high,
                               BaseType_t core_id,
                               libirq_source_handle_t* out_handle);

// This function may be called at most once, and when it returns it guarentees
// that any associated `irq_waiter_t` handles will return false upon a
// subsequent call to `libirq_waiter_sleep_until_active()`. (Thus the waiter
// handles will also then be destroyed.)
//
// It is illegal to use `handle` after a call to this function.
void libirq_source_destroy(libirq_source_handle_t handle);

// Triggers an IRQ event for the given IRQ soruce, exactly as if a real IRQ had
// occured (thereby waking all currently sleeping waiters).
void libirq_source_trigger(libirq_source_handle_t handle);

// Creates a waiter for the given `handle`. Used to wait for the IRQ assigned to
// the source to become active. See the description of `libirq_source_create()`
// for the purpose of waiters.
libirq_waiter_t libirq_waiter_create(libirq_source_handle_t handle);

// Explicitly destroys the given waiter. The waiter subsequently cannot be used
// ever again.
void libirq_waiter_destroy(libirq_waiter_t waiter);

// Returns `true` if we woke because the assigned IRQ became active, else
// returns `false` if the IRQ source has been destroyed.
//
// DANGER: If this function returns false it is illegal to use the waiter ever
// again (including passing the waiter to `libirq_waiter_destroy()`).
bool libirq_waiter_sleep_until_active(libirq_waiter_t waiter);

// Convenience method to create a libtask loop task from the given IRQ source
// waiter, which continuously waits for an active IRQ and calls `do_once()` each
// time it occurs.
//
// It is illegal to use the given `waiter` after it is passed to this task.
esp_err_t libirq_managed_loop_spawn(libirq_waiter_t waiter,
                                    libtask_do_once_fn_t do_once, void* private,
                                    const char* name, uint32_t task_stack_size,
                                    UBaseType_t task_priority,
                                    libtask_loop_handle_t* out_handle);
