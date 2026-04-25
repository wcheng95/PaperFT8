#pragma once

#include <stdint.h>
#include "host_mocks.h"

static inline int64_t esp_timer_get_time(void) {
    return rtc_now_ms() * 1000;
}
