#ifndef COMMON_H
#define COMMON_H

ESP_EVENT_DECLARE_BASE(SMART_LOCK_EVENT);

/**
 * key click event
 */
typedef enum {
    KEY_1_SHORT_CLICK,
    KEY_1_LONG_CLICK,
    KEY_2_SHORT_CLICK,
    KEY_2_LONG_CLICK,
    FINGER_REC_OK,
    FINGER_REC_FAILED
} smart_lock_event_id_t;

#endif