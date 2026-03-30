#include "ui.h"
#include <M5Unified.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include "freertos/semphr.h"

static constexpr int RX_LINES = 6;

static constexpr int kLineCount = 8;
static constexpr int kLineHeightPx = 67;
static constexpr int kTopLineIdx = 0;
static constexpr int kTextFirstLineIdx = 1;
static constexpr int kCommandLineIdx = 7;
static constexpr int kCommandButtons = 12;

static constexpr int kModeTextSize = 2;
static constexpr int kBodyTextSize = 4;
static constexpr int kCommandTextSize = 3;
static constexpr int kTextLeftPx = 24;

static constexpr uint16_t UI_BG = TFT_WHITE;
static constexpr uint16_t UI_FG = TFT_BLACK;

static const char* kCommandLabels[kCommandButtons] = {
    "12", "ESC", "S", "R", "T", "Q", "M", "B", "C", "D", "^", "v"
};

static bool ui_paused = false;
static UiLayout g_layout;
static int g_y_offset = 0;

static std::vector<UiRxLine> rx_lines;
static int rx_page = 0;
static int rx_selected = -1;
static std::vector<UiRxLine> last_drawn_lines;
static int last_page = -1;

static SemaphoreHandle_t g_disp_mutex = nullptr;

static void disp_lock() {
    if (g_disp_mutex) {
        xSemaphoreTake(g_disp_mutex, portMAX_DELAY);
    }
}

static void disp_unlock() {
    if (g_disp_mutex) {
        xSemaphoreGive(g_disp_mutex);
    }
}

struct DispGuard {
    DispGuard() { disp_lock(); }
    ~DispGuard() { disp_unlock(); }
};

static int line_y(int line_idx) {
    return g_y_offset + line_idx * kLineHeightPx;
}

static void draw_row_frame(int line_idx) {
    const int row_y = line_y(line_idx);
    M5.Display.fillRect(0, row_y, g_layout.screen_w, kLineHeightPx, UI_BG);
    M5.Display.drawFastHLine(0, row_y, g_layout.screen_w, UI_FG);
    M5.Display.drawFastHLine(0, row_y + kLineHeightPx - 1, g_layout.screen_w, UI_FG);
}

static void draw_mode_row(const char* text) {
    draw_row_frame(kTopLineIdx);
    M5.Display.setTextColor(UI_FG, UI_BG);
    M5.Display.setTextDatum(middle_left);
    M5.Display.setTextSize(kModeTextSize);
    M5.Display.drawString(text ? text : "", 8, line_y(kTopLineIdx) + (kLineHeightPx / 2));
}

static void draw_text_row(int row_idx, const char* text, bool outline) {
    if (row_idx < 0 || row_idx >= RX_LINES) return;
    const int screen_line = kTextFirstLineIdx + row_idx;
    const int row_y = line_y(screen_line);

    draw_row_frame(screen_line);
    M5.Display.setTextColor(UI_FG, UI_BG);
    M5.Display.setTextDatum(middle_left);
    M5.Display.setTextSize(kBodyTextSize);
    M5.Display.drawString(text ? text : "", kTextLeftPx, row_y + (kLineHeightPx / 2));

    if (outline) {
        M5.Display.drawRect(0, row_y, g_layout.screen_w, kLineHeightPx, UI_FG);
    }
}

static void draw_command_button_locked(int button_idx) {
    if (button_idx < 0 || button_idx >= kCommandButtons) return;

    const int x0 = (button_idx * g_layout.screen_w) / kCommandButtons;
    const int x1 = ((button_idx + 1) * g_layout.screen_w) / kCommandButtons;
    const int w = x1 - x0;
    const int row_y = line_y(kCommandLineIdx);

    M5.Display.fillRect(x0, row_y, w, kLineHeightPx, UI_BG);
    M5.Display.drawRect(x0, row_y, w, kLineHeightPx, UI_FG);

    M5.Display.setTextColor(UI_FG, UI_BG);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextSize(kCommandTextSize);
    M5.Display.drawString(kCommandLabels[button_idx], x0 + (w / 2), row_y + (kLineHeightPx / 2));
}

static void draw_command_bar_locked() {
    for (int i = 0; i < kCommandButtons; ++i) {
        draw_command_button_locked(i);
    }
}

const UiLayout& ui_layout() { return g_layout; }

void ui_set_paused(bool paused) { ui_paused = paused; }
bool ui_is_paused() { return ui_paused; }

bool ui_waterfall_dirty() { return false; }
void ui_draw_waterfall_if_dirty() {}

void ui_init() {
    g_disp_mutex = xSemaphoreCreateMutex();

    auto cfg = M5.config();
    cfg.output_power = true;
    cfg.external_rtc = false;
    M5.begin(cfg);

    M5.Display.setRotation(1);
    M5.Display.setEpdMode(epd_mode_t::epd_text);
    M5.Display.fillScreen(UI_BG);

    g_layout.screen_w = M5.Display.width();
    g_layout.screen_h = M5.Display.height();

    const int block_h = kLineCount * kLineHeightPx;
    g_y_offset = (g_layout.screen_h > block_h) ? ((g_layout.screen_h - block_h) / 2) : 0;

    g_layout.mode_box = {0, line_y(kTopLineIdx), g_layout.screen_w, kLineHeightPx};
    g_layout.text_area = {0, line_y(kTextFirstLineIdx), g_layout.screen_w, kLineHeightPx * RX_LINES};
    g_layout.command_bar = {0, line_y(kCommandLineIdx), g_layout.screen_w, kLineHeightPx};
    g_layout.waterfall = {0, 0, 0, 0};
    g_layout.countdown = {0, 0, 0, 0};
    g_layout.line_h = kLineHeightPx;

    draw_mode_row("");
    for (int i = 0; i < RX_LINES; ++i) {
        draw_text_row(i, "", false);
    }
    draw_command_bar_locked();
    M5.Display.display();
}

void ui_set_waterfall_row(int row, const uint8_t* bins, int len) {
    (void)row;
    (void)bins;
    (void)len;
}

void ui_push_waterfall_row(const uint8_t* bins, int len) {
    (void)bins;
    (void)len;
}

void ui_clear_waterfall() {}

void ui_draw_waterfall() {}

void ui_draw_countdown(float fraction, bool even_slot, int offset_hz) {
    (void)fraction;
    (void)even_slot;
    (void)offset_hz;
}

void ui_set_rx_list(const std::vector<UiRxLine>& lines) {
    rx_lines = lines;
    rx_page = 0;
    rx_selected = -1;
    last_drawn_lines.clear();
    last_page = -1;
}

void ui_force_redraw_rx() {
    last_drawn_lines.clear();
    last_page = -1;
}

static void draw_rx_line(int row_idx, const UiRxLine& l, int line_no, bool selected) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%d %s", line_no, l.text.c_str());
    draw_text_row(row_idx, buf, selected);
}

void ui_draw_rx(int flash_index) {
    if (!rx_lines.empty() && flash_index < 0) {
        if (rx_page == last_page && last_drawn_lines.size() == rx_lines.size()) {
            bool same = true;
            for (size_t i = 0; i < rx_lines.size(); ++i) {
                if (rx_lines[i].text != last_drawn_lines[i].text ||
                    rx_lines[i].is_cq != last_drawn_lines[i].is_cq ||
                    rx_lines[i].is_to_me != last_drawn_lines[i].is_to_me) {
                    same = false;
                    break;
                }
            }
            if (same) return;
        }
    }

    DispGuard guard;
    const int start = rx_page * RX_LINES;
    for (int i = 0; i < RX_LINES; ++i) {
        const int idx = start + i;
        if (idx < (int)rx_lines.size()) {
            const bool selected = (idx == flash_index);
            draw_rx_line(i, rx_lines[idx], i + 1, selected);
        } else {
            draw_text_row(i, "", false);
        }
    }
    M5.Display.display();

    if (flash_index < 0) {
        last_page = rx_page;
        last_drawn_lines = rx_lines;
    } else {
        last_page = -1;
        last_drawn_lines.clear();
    }
}

int ui_handle_rx_key(char c) {
    int selected_idx = -1;
    if (c == 0) return selected_idx;

    if (c == ';') {
        if (rx_page > 0) {
            rx_page--;
            ui_draw_rx();
        }
    } else if (c == '.') {
        if ((rx_page + 1) * RX_LINES < (int)rx_lines.size()) {
            rx_page++;
            ui_draw_rx();
        }
    } else if (c >= '1' && c <= '6') {
        int line = c - '1';
        int idx = rx_page * RX_LINES + line;
        if (idx >= 0 && idx < (int)rx_lines.size()) {
            rx_selected = idx;
            ui_draw_rx();
            selected_idx = idx;
        }
    }

    return selected_idx;
}

void ui_draw_list(const std::vector<std::string>& lines, int page, int highlight_abs) {
    DispGuard guard;

    for (int i = 0; i < RX_LINES; ++i) {
        int idx = page * RX_LINES + i;
        if (idx < (int)lines.size()) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%d %s", i + 1, lines[idx].c_str());
            draw_text_row(i, buf, idx == highlight_abs);
        } else {
            draw_text_row(i, "", idx == highlight_abs);
        }
    }

    M5.Display.display();
}

void ui_draw_debug(const std::vector<std::string>& lines, int page) {
    DispGuard guard;

    for (int i = 0; i < RX_LINES; ++i) {
        int idx = page * RX_LINES + i;
        if (idx < (int)lines.size()) {
            draw_text_row(i, lines[idx].c_str(), false);
        } else {
            draw_text_row(i, "", false);
        }
    }

    M5.Display.display();
}

void ui_draw_mode_box(const char* mode_label) {
    DispGuard guard;
    draw_mode_row(mode_label ? mode_label : "");
    M5.Display.display();
}

int ui_command_hit_test(int x, int y) {
    const UiRect& bar = g_layout.command_bar;
    if (!bar.contains(x, y)) return -1;
    return (x * kCommandButtons) / g_layout.screen_w;
}

void ui_draw_command_bar() {
    DispGuard guard;
    draw_command_bar_locked();
    M5.Display.display();
}

void ui_draw_tx(const std::string& next,
                const std::vector<std::string>& queue,
                int page,
                int selected,
                const std::vector<bool>& mark_delete,
                const std::vector<int>& slot_colors) {
    (void)slot_colors;

    DispGuard guard;

    {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "1 %s", next.c_str());
        draw_text_row(0, buf, false);
    }

    int start_idx = page * 5;
    for (int row = 1; row < RX_LINES; ++row) {
        int idx = start_idx + (row - 1);
        if (idx < (int)queue.size()) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%d %s", row + 1, queue[idx].c_str());
            bool outline = (idx == selected) ||
                           (idx < (int)mark_delete.size() && mark_delete[idx]);
            draw_text_row(row, buf, outline);
        } else {
            draw_text_row(row, "", false);
        }
    }

    M5.Display.display();
}

int ui_rx_hit_test(int x, int y) {
    if (!g_layout.text_area.contains(x, y)) return -1;
    int line = (y - g_layout.text_area.y) / g_layout.line_h;
    if (line < 0 || line >= RX_LINES) return -1;
    int idx = rx_page * RX_LINES + line;
    if (idx < 0 || idx >= (int)rx_lines.size()) return -1;
    return idx;
}

int ui_text_line_hit_test(int x, int y) {
    if (!g_layout.text_area.contains(x, y)) return -1;
    int line = (y - g_layout.text_area.y) / g_layout.line_h;
    if (line < 0 || line >= RX_LINES) return -1;
    return line;
}

void ui_rx_scroll(int delta) {
    if (delta < 0) {
        if (rx_page > 0) {
            rx_page--;
            ui_draw_rx();
        }
    } else if (delta > 0) {
        if ((rx_page + 1) * RX_LINES < (int)rx_lines.size()) {
            rx_page++;
            ui_draw_rx();
        }
    }
}
