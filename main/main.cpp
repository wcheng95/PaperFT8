#include <cstdio>
#include <M5Unified.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr int kLineCount = 8;
constexpr int kLineHeightPx = 67;
constexpr int kTextSize = 4;
constexpr int kTextLeftPx = 24;
constexpr int kBlankLineIdx = 0;
constexpr int kTextFirstLineIdx = 1;
constexpr int kTextLineCount = 6;
constexpr int kCommandLineIdx = 7;
constexpr int kCommandButtonCount = 12;

const char* kFakeRxLines[kTextLineCount] = {
    "1009   0 CQ AI5YH EM32",
    "1641   0 CQ AC4ZH EM78",
    " 709   8 EA8B AB6ME 73",
    "1394   0 EA8B KF0BAA DM79",
    "2434   5 <...> K8KWT EN72",
    "1222   6 LU7ART KF0OUQ EM36"
};

const char* kCommandLabels[kCommandButtonCount] = {
    "12", "ESC", "S", "R", "T", "Q", "M", "B", "C", "D", "^", "v"
};

int g_y_offset = 0;
bool g_text_line_inverted[kTextLineCount] = {false};
bool g_button_inverted[kCommandButtonCount] = {false};
bool g_touch_latched = false;

int line_y(int line_idx) {
    return g_y_offset + line_idx * kLineHeightPx;
}

void draw_blank_line() {
    const int row_y = line_y(kBlankLineIdx);
    const int display_w = M5.Display.width();
    M5.Display.fillRect(0, row_y, display_w, kLineHeightPx, TFT_WHITE);
    M5.Display.drawFastHLine(0, row_y, display_w, TFT_BLACK);
    M5.Display.drawFastHLine(0, row_y + kLineHeightPx - 1, display_w, TFT_BLACK);
}

void draw_text_line(int text_idx) {
    if (text_idx < 0 || text_idx >= kTextLineCount) {
        return;
    }

    const int screen_line_idx = kTextFirstLineIdx + text_idx;
    const int row_y = line_y(screen_line_idx);
    const int display_w = M5.Display.width();
    const bool inverted = g_text_line_inverted[text_idx];
    const uint16_t bg = inverted ? TFT_BLACK : TFT_WHITE;
    const uint16_t fg = inverted ? TFT_WHITE : TFT_BLACK;

    M5.Display.fillRect(0, row_y, display_w, kLineHeightPx, bg);
    M5.Display.drawFastHLine(0, row_y, display_w, TFT_BLACK);
    M5.Display.drawFastHLine(0, row_y + kLineHeightPx - 1, display_w, TFT_BLACK);

    M5.Display.setTextColor(fg, bg);
    M5.Display.setTextDatum(middle_left);
    M5.Display.setTextSize(kTextSize);

    M5.Display.drawString(kFakeRxLines[text_idx], kTextLeftPx, row_y + (kLineHeightPx / 2));
}

void draw_command_button(int button_idx) {
    if (button_idx < 0 || button_idx >= kCommandButtonCount) return;

    const int display_w = M5.Display.width();
    const int row_y = line_y(kCommandLineIdx);
    const int x0 = (button_idx * display_w) / kCommandButtonCount;
    const int x1 = ((button_idx + 1) * display_w) / kCommandButtonCount;
    const int w = x1 - x0;
    const bool inverted = g_button_inverted[button_idx];
    const uint16_t bg = inverted ? TFT_BLACK : TFT_WHITE;
    const uint16_t fg = inverted ? TFT_WHITE : TFT_BLACK;

    M5.Display.fillRect(x0, row_y, w, kLineHeightPx, bg);
    M5.Display.drawRect(x0, row_y, w, kLineHeightPx, TFT_BLACK);

    M5.Display.setTextColor(fg, bg);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextSize(3);
    M5.Display.drawString(kCommandLabels[button_idx], x0 + (w / 2), row_y + (kLineHeightPx / 2));
}

void draw_command_line() {
    for (int i = 0; i < kCommandButtonCount; ++i) {
        draw_command_button(i);
    }
}

int hit_test_text_line(int x, int y) {
    if (x < 0 || x >= M5.Display.width()) return -1;
    if (y < g_y_offset) return -1;

    const int line_idx = (y - g_y_offset) / kLineHeightPx;
    if (line_idx < kTextFirstLineIdx || line_idx >= (kTextFirstLineIdx + kTextLineCount)) {
        return -1;
    }

    return line_idx - kTextFirstLineIdx;
}

int hit_test_command_button(int x, int y) {
    const int row_y = line_y(kCommandLineIdx);
    if (x < 0 || x >= M5.Display.width()) return -1;
    if (y < row_y || y >= row_y + kLineHeightPx) return -1;
    return (x * kCommandButtonCount) / M5.Display.width();
}

void handle_touch() {
    if (M5.Touch.getCount() == 0) {
        g_touch_latched = false;
        return;
    }

    const auto touch = M5.Touch.getDetail();
    if (!touch.isPressed()) {
        g_touch_latched = false;
        return;
    }

    if (g_touch_latched) {
        return;
    }
    g_touch_latched = true;

    bool needs_display = false;
    const int button_idx = hit_test_command_button(touch.x, touch.y);
    if (button_idx >= 0) {
        g_button_inverted[button_idx] = !g_button_inverted[button_idx];
        draw_command_button(button_idx);
        needs_display = true;
    } else {
        const int text_idx = hit_test_text_line(touch.x, touch.y);
        if (text_idx >= 0) {
            g_text_line_inverted[text_idx] = !g_text_line_inverted[text_idx];
            draw_text_line(text_idx);
            needs_display = true;
        }
    }

    if (needs_display) {
        M5.Display.display();
    }
}

void draw_layout() {
    M5.Display.setEpdMode(epd_mode_t::epd_text);
    M5.Display.fillScreen(TFT_WHITE);

    const int display_h = M5.Display.height();
    const int block_h = kLineCount * kLineHeightPx;
    g_y_offset = (display_h > block_h) ? ((display_h - block_h) / 2) : 0;

    for (int i = 0; i < kTextLineCount; ++i) {
        g_text_line_inverted[i] = false;
    }
    for (int i = 0; i < kCommandButtonCount; ++i) {
        g_button_inverted[i] = false;
    }

    draw_blank_line();
    for (int i = 0; i < kTextLineCount; ++i) {
        draw_text_line(i);
    }
    draw_command_line();

    M5.Display.display();
}

}  // namespace

extern "C" void app_main(void) {
    auto cfg = M5.config();
    cfg.output_power = true;
    cfg.external_rtc = false;
    M5.begin(cfg);

    M5.Display.setRotation(1);
    draw_layout();

    while (true) {
        M5.update();
        handle_touch();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
