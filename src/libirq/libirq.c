#include "libirq.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <libesp.h>

static const char* TAG = "libirq";

#define IRQ_TASK_STACK_SIZE 2048
#define IRQ_TASK_PRIORITY 15

struct libirq_source {
    // A copy of this pin number must be stored here for the ISR to access.
    gpio_num_t pin_irq;
    // FIXME relocate this?
    // Whether to trigger the ISR handler on a low pin level or high pin level.
    bool trigger_high;

    // Used to wait for a signal from the IRQ ISR, and to instruct the ISR
    // setup/teardown task to stop.
    EventGroupHandle_t events;
#define IRQ_EVENT_GOT_IRQ (1ULL << 0)
#define IRQ_EVENT_SHOULD_TEARDOWN (1ULL << 1)
#define IRQ_EVENT_TEARDOWN_FINISHED (1ULL << 2)

    SemaphoreHandle_t refcount_lock;
    uint32_t refcount;
};

static libirq_source_t* alloc_irq_source(gpio_num_t pin_irq, bool trigger_high) {
    libirq_source_t* src = malloc(sizeof(libirq_source_t));
    src->pin_irq = pin_irq;
    src->trigger_high = trigger_high;
    src->events = xEventGroupCreate();

    src->refcount_lock = xSemaphoreCreateMutex();
    src->refcount = 1;

    return src;
}

static void free_irq_source(libirq_source_t* src) {
    vEventGroupDelete(src->events);
    free(src);
}

static void inc_refcount_irq_source(libirq_source_t* src) {
    while (xSemaphoreTake(src->refcount_lock, portMAX_DELAY) != pdTRUE)
        ;

    src->refcount++;
    assert(src->refcount);

    xSemaphoreGive(src->refcount_lock);
}

static void dec_refcount_irq_source(libirq_source_t* src) {
    while (xSemaphoreTake(src->refcount_lock, portMAX_DELAY) != pdTRUE)
        ;

    src->refcount--;
    bool should_free = !src->refcount;

    xSemaphoreGive(src->refcount_lock);

    if (should_free) {
        free_irq_source(src);
    }
}

static void isr_irq(void* arg) {
    libirq_source_t* src = arg;

    // First, since this interrupt is level-triggered (so we don't miss any) disable further
    // interrupts. If the waiting task happens to enable this interrupt again asynchronously
    // with this, we don't mind, since `sleep_until_irq_active()` clears spurious pending
    // messages.
    gpio_intr_disable(src->pin_irq);

    // Only now give the semaphore, which will ensure that the `irq_loop()` will always
    // eventually take the semaphore and call `gpio_intr_enable()` for us. (We don't care
    // about the return value, i.e. whether we were able to.)
    xEventGroupSetBits(src->events, IRQ_EVENT_GOT_IRQ);
}

// Returns `true` if the sleep ended because the IRQ became active,
// else returns `false` if the sleep was interrupted by a call for the underlying IRQ source
// to destroy itself.
static bool sleep_until_irq_active(const libirq_source_t* src) {
    // Clear a potential previous IRQ message from the IRQ ISR. Note that this call
    // is safe to make even if other tasks are currently waiting on this bit.
    //
    // It is important that this bit is never cleared after a call to `gpio_intr_enable`(),
    // e.g. upon its reciept below, because this can result in a race condition
    // (as explained below).
    xEventGroupClearBits(src->events, IRQ_EVENT_GOT_IRQ);

    // First, enable the IRQ ISR. This is safe to call if another task has already
    // done this and is currently waiting (but in principle could cause a "race" where
    // both the we and the other task observe the "first" IRQ to arrive, while the ISR
    // triggers again due to this call and sets the `IRQ_EVENT_GOT_IRQ` bit again
    // "supriously"). This corner-case is dealt with by the bit clear performed above---
    // it can only occur when we make this `gpio_intr_enable()` call after the ISR
    // has already triggered and disabled itself, but before the other waiting task has
    // recieved the `IRQ_EVENT_GOT_IRQ` message and cleared the pending bit.
    gpio_intr_enable(src->pin_irq);

    // Now wait until the ISR tells us IRQ has gone active again.
    //
    // Note that it could have happened that after this `gpio_intr_enable()` call, because
    // of another wait operation in progress on another thread the ISR had already triggered,
    // disabled itself, and then sent the `IRQ_EVENT_GOT_IRQ` bit. It is therefore imperative
    // that the other waiting task does not automatically clear the `IRQ_EVENT_GOT_IRQ` bit
    // upon its reciept. Else this could occur before we call `xEventGroupWaitBits()` ourselves,
    // thereby missing the setting of `IRQ_EVENT_GOT_IRQ` which we wanted to observe. This would
    // then lock up this thread until another thread calls `sleep_until_irq_active()` again.
    //
    // As a solution, we simply don't ask FreeRTOS to automatically clear the `IRQ_EVENT_GOT_IRQ`
    // bit upon its reciept. Instead we perform the relevant bit clear above, before the call to
    // `gpio_intr_enable()`, which solves all of our propblems.
    EventBits_t bits;
    do {
        bits = xEventGroupWaitBits(src->events,
                                   (IRQ_EVENT_GOT_IRQ | IRQ_EVENT_SHOULD_TEARDOWN),
                                   false, false, portMAX_DELAY);
    } while (!(bits & (IRQ_EVENT_GOT_IRQ | IRQ_EVENT_SHOULD_TEARDOWN)));

    return !(bits & IRQ_EVENT_SHOULD_TEARDOWN);
}

static void irq_task(void* arg) {
    libirq_source_t* src = arg;

    gpio_num_t pin_irq = src->pin_irq;

    gpio_config_t io_conf = {
        .intr_type = src->trigger_high ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pin_irq),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Prevent the level trigger interrupt on the IRQ pin
    // from hanging up the core when we enable interrupts.
    gpio_intr_disable(pin_irq);

    // TODO Install in a way which doesn't collide with other code.
    // (Note that we need `gpio_install_isr_service()` to have been
    // called without the flag which requires IRAM ISRs.)
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin_irq, isr_irq, src);

    EventBits_t bits;
    do {
        bits = xEventGroupWaitBits(src->events,
                                   IRQ_EVENT_SHOULD_TEARDOWN,
                                   false, false, portMAX_DELAY);
    } while (!(bits & IRQ_EVENT_SHOULD_TEARDOWN));

    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_isr_handler_remove(pin_irq);
    gpio_uninstall_isr_service();

    gpio_intr_enable(pin_irq);

    ESP_LOGD(TAG, "irq task stopped");

    xEventGroupSetBits(src->events, IRQ_EVENT_TEARDOWN_FINISHED);
}

esp_err_t libirq_source_create(gpio_num_t pin_irq, bool trigger_high, BaseType_t core_id,
                               libirq_source_handle_t* out_handle) {
    libirq_source_t* src = alloc_irq_source(pin_irq, trigger_high);

    // Note that this task will allocate an interrupt, and therefore must happen on a well-defined core
    // (hence `xTaskCreatePinnedToCore()`), since interrupts must be deallocated on the same core they
    // were allocated on.
    //
    // After allocating the interrupt the task will put itself to sleep, only to awake when it is time
    // for the interrupt to be deallocated.
    BaseType_t result = xTaskCreatePinnedToCore(&irq_task, "libirq_task", IRQ_TASK_STACK_SIZE, (void*) src,
                                                IRQ_TASK_PRIORITY, NULL, core_id);
    if (result != pdPASS) {
        free_irq_source(src);

        ESP_LOGE(TAG, "failed to create irq task! (0x%X)", result);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "%s: %p", __func__, src);

    *out_handle = src;
    return ESP_OK;
}

void libirq_source_trigger(libirq_source_handle_t handle) {
    xEventGroupSetBits(handle->events, IRQ_EVENT_GOT_IRQ);
}

void libirq_source_destroy(libirq_source_handle_t handle) {
    ESP_LOGD(TAG, "%s: %p", __func__, handle);

    xEventGroupSetBits(handle->events, IRQ_EVENT_SHOULD_TEARDOWN);

    EventBits_t bits;
    do {
        bits = xEventGroupWaitBits(handle->events,
                                   IRQ_EVENT_SHOULD_TEARDOWN,
                                   false, false, portMAX_DELAY);
    } while (!(bits & IRQ_EVENT_TEARDOWN_FINISHED));

    dec_refcount_irq_source(handle);
}

libirq_waiter_t libirq_waiter_create(libirq_source_handle_t handle) {
    // It is a race condition to create waiters after the source has been destroyed.
    // Also, this is pointless.
    assert(!(xEventGroupGetBits(handle->events) & IRQ_EVENT_SHOULD_TEARDOWN));

    libirq_waiter_t waiter = {
        .__internal = handle,
    };
    inc_refcount_irq_source(handle);
    return waiter;
}

void libirq_waiter_destroy(libirq_waiter_t waiter) {
    dec_refcount_irq_source(waiter.__internal);
}

bool libirq_waiter_sleep_until_active(libirq_waiter_t waiter) {
    bool should_destroy = !sleep_until_irq_active(waiter.__internal);

    if (should_destroy) {
        libirq_waiter_destroy(waiter);
    }

    return !should_destroy;
}

typedef struct waiter_loop_data {
    libirq_waiter_t waiter;

    libtask_do_once_fn_t do_once;
    void* private;
} waiter_loop_data_t;

static libtask_disposition_t libirq_do_once_wrapper(waiter_loop_data_t* data) {
    if (!libirq_waiter_sleep_until_active(data->waiter)) {
        free(data);
        return LIBTASK_DISPOSITION_STOP;
    }

    libtask_disposition_t disp = data->do_once(data->private);

    if (disp == LIBTASK_DISPOSITION_STOP) {
        libirq_waiter_destroy(data->waiter);
        free(data);
    }

    return disp;
}

esp_err_t libirq_managed_loop_spawn(libirq_waiter_t waiter,
                                    libtask_do_once_fn_t do_once, void* private,
                                    const char* name, uint32_t task_stack_size, UBaseType_t task_priority,
                                    libtask_loop_handle_t* out_handle) {
    waiter_loop_data_t* data = malloc(sizeof(waiter_loop_data_t));
    data->waiter = waiter;
    data->do_once = do_once;
    data->private = private;

    esp_err_t ret = libtask_loop_spawn((libtask_do_once_fn_t) libirq_do_once_wrapper, data, name, task_stack_size, task_priority, out_handle);

    if (ret != ESP_OK) {
        libirq_waiter_destroy(waiter);
        free(data);
    }

    return ret;
}
