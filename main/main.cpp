#include <cstdio>
#include <M5Unified.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr int kLineCount = 8;
constexpr int kLineHeightPx = 67;
constexpr int kTextSize = 4;
constexpr int kTextLeftPx = 24;

int g_y_offset = 0;
bool g_line_inverted[kLineCount] = {false};

void draw_line(int line_idx) {
    if (line_idx < 0 || line_idx >= kLineCount) {
        return;
    }

    const int display_w = M5.Display.width();
    const int row_y = g_y_offset + line_idx * kLineHeightPx;
    const bool inverted = g_line_inverted[line_idx];
    const uint16_t bg = inverted ? TFT_BLACK : TFT_WHITE;
    const uint16_t fg = inverted ? TFT_WHITE : TFT_BLACK;

    M5.Display.fillRect(0, row_y + 1, display_w, kLineHeightPx - 1, bg);
    M5.Display.drawFastHLine(0, row_y, display_w, TFT_BLACK);
    if (line_idx == kLineCount - 1) {
        M5.Display.drawFastHLine(0, row_y + kLineHeightPx, display_w, TFT_BLACK);
    }

    M5.Display.setTextColor(fg, bg);
    M5.Display.setTextDatum(middle_left);
    M5.Display.setTextSize(kTextSize);

    char text[16];
    std::snprintf(text, sizeof(text), "Line %d", line_idx + 1);
    M5.Display.drawString(text, kTextLeftPx, row_y + (kLineHeightPx / 2));
}

int hit_test_line(int x, int y) {
    if (x < 0 || x >= M5.Display.width()) {
        return -1;
    }
    if (y < g_y_offset) {
        return -1;
    }

    const int relative_y = y - g_y_offset;
    if (relative_y >= kLineCount * kLineHeightPx) {
        return -1;
    }

    return relative_y / kLineHeightPx;
}

void draw_lines() {
    M5.Display.setEpdMode(epd_mode_t::epd_text);
    M5.Display.fillScreen(TFT_WHITE);
    const int display_h = M5.Display.height();
    const int block_h = kLineCount * kLineHeightPx;
    g_y_offset = (display_h > block_h) ? ((display_h - block_h) / 2) : 0;

    for (int i = 0; i < kLineCount; ++i) {
        draw_line(i);
    }

    M5.Display.display();
}

}  // namespace

extern "C" void app_main(void) {
    auto cfg = M5.config();
    cfg.output_power = true;
    cfg.external_rtc = false;
    M5.begin(cfg);

    // Keep the user-selected orientation.
    M5.Display.setRotation(1);
    draw_lines();

    while (true) {
        M5.update();

        auto touch = M5.Touch.getDetail();
        if (touch.wasPressed()) {
            const int line_idx = hit_test_line(touch.x, touch.y);
            if (line_idx >= 0) {
                g_line_inverted[line_idx] = !g_line_inverted[line_idx];
                draw_line(line_idx);
                M5.Display.display();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
