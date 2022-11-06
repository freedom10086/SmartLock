#include <esp_sleep.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "zw800.h"
#include "key.h"
#include "common.h"
#include "motor.h"

static const char *TAG = "main";

static void
key_click_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "rev key click event %ld", event_id);
    switch (event_id) {
        case KEY_1_SHORT_CLICK:
            zw800_add_finger(0);
            break;
        case KEY_1_LONG_CLICK:
            zw800_clear_all_finger();
            break;
        case FINGER_REC_OK:
            open_lock();
            break;
    }
}

void app_main(void) {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        ESP_LOGI(TAG, "wake up by cause  %d", cause);
    }

    // Create the event loops
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // key click event
    esp_event_handler_instance_register(
            SMART_LOCK_EVENT, ESP_EVENT_ANY_ID,
            key_click_event_handler, NULL, NULL);
    key_init();

    // zw800 finger print sensor
    zw800_config_t zw800_config = ZW800_CONFIG_DEFAULT();
    zw800_init(&zw800_config);

    motor_init();
}
