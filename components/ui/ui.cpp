#include "ui.h"
#include <M5Unified.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <cmath>
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static constexpr int RX_LINES = 6;

static constexpr int kLineCount = 8;
static constexpr int kLineHeightPx = 67;
static constexpr int kTopLineIdx = 0;
static constexpr int kTextFirstLineIdx = 1;
static constexpr int kCommandLineIdx = 7;
static constexpr int kCommandButtons = 12;

static constexpr int kBodyTextSize = 4;
static constexpr int kCommandTextSize = 3;
static constexpr int kTextLeftPx = 24;
static constexpr int kWaterfallCols = 240;
static constexpr int kWaterfallRows = 24;
static constexpr int64_t kWaterfallDrawMinMs = 700;
static constexpr int kWaterfallFreqMinHz = 200;
static constexpr int kWaterfallFreqMaxHz = 3000;
static constexpr int kWaterfallMarksHz[] = {500, 1000, 1500, 2000, 2500};

static constexpr uint16_t UI_BG = TFT_WHITE;
static constexpr uint16_t UI_FG = TFT_BLACK;

static const char* kCommandLabels[kCommandButtons] = {
    "12", "ESC", "S", "R", "T", "Q", "M", "B", "C", "D", "^", "v"
};
static constexpr int kCmdIdxS = 2;
static constexpr int kCmdIdxR = 3;
static constexpr int kCmdIdxT = 4;
static constexpr int kCmdIdxQ = 5;
static constexpr int kCmdIdxM = 6;
static constexpr int kCmdIdxB = 7;
static constexpr int kCmdIdxC = 8;
static constexpr int kCmdIdxD = 9;

static bool ui_paused = false;
static UiLayout g_layout;
static int g_y_offset = 0;
static int g_active_mode_button = -1;
static bool g_command_enabled[kCommandButtons] = {
    true, true, true, true, true, true, true, true, true, true, true, true
};
static bool g_header_widgets_enabled = true;
static bool g_countdown_enabled = false;  // runtime toggle via command button 0

static bool g_countdown_even = true;
static int g_countdown_second = 0;  // 0..12 within 15s slot
static bool g_countdown_visual_initialized = false;
static bool g_countdown_visual_odd = false;

static uint8_t g_waterfall_ring[kWaterfallRows][kWaterfallCols] = {};
static int g_waterfall_head = 0;  // next write row
static bool g_waterfall_has_data = false;
static bool g_waterfall_dirty_flag = false;
static int64_t g_waterfall_last_draw_ms = 0;

static std::vector<UiRxLine> rx_lines;
static int rx_page = 0;
static int rx_selected = -1;
static std::vector<UiRxLine> last_drawn_lines;
static int last_page = -1;

static SemaphoreHandle_t g_disp_mutex = nullptr;
static SemaphoreHandle_t g_wf_mutex = nullptr;
static TaskHandle_t g_disp_task = nullptr;
static volatile bool g_display_pending = false;

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

static int64_t now_ms() {
    return esp_timer_get_time() / 1000;
}

static int command_index_from_key(char key) {
    char k = static_cast<char>(std::toupper(static_cast<unsigned char>(key)));
    switch (k) {
        case 'S': return kCmdIdxS;
        case 'R': return kCmdIdxR;
        case 'T': return kCmdIdxT;
        case 'Q': return kCmdIdxQ;
        case 'M': return kCmdIdxM;
        case 'B': return kCmdIdxB;
        case 'C': return kCmdIdxC;
        case 'D': return kCmdIdxD;
        default: return -1;
    }
}

static void request_display_flush() {
    if (!g_disp_task) {
        M5.Display.display();
        return;
    }
    g_display_pending = true;
    xTaskNotifyGive(g_disp_task);
}

static void ui_display_task(void*) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (g_display_pending) {
            g_display_pending = false;
            DispGuard guard;
            M5.Display.display();
        }
    }
}

static int line_y(int line_idx) {
    return g_y_offset + line_idx * kLineHeightPx;
}

static void draw_row_frame(int line_idx) {
    const int row_y = line_y(line_idx);
    M5.Display.fillRect(0, row_y, g_layout.screen_w, kLineHeightPx, UI_BG);
    M5.Display.drawFastHLine(0, row_y, g_layout.screen_w, UI_FG);
    M5.Display.drawFastHLine(0, row_y + kLineHeightPx - 1, g_layout.screen_w, UI_FG);
}

static void draw_waterfall_freq_marks_locked() {
    const UiRect& wf = g_layout.waterfall;
    if (wf.w <= 0 || wf.h <= 0) return;
    if (kWaterfallFreqMaxHz <= kWaterfallFreqMinHz) return;

    const int row_y = line_y(kTopLineIdx);
    const int scale_top = row_y + 2;
    const int scale_bottom = wf.y - 2;
    if (scale_bottom <= scale_top) return;

    const int axis_y = std::max(scale_top + 10, scale_bottom - 8);

    M5.Display.setTextColor(UI_FG, UI_BG);
    M5.Display.setTextDatum(top_center);
    M5.Display.setTextSize(2);

    auto draw_thick_tick = [&](int x, int y, int h, int thickness) {
        if (h <= 0 || thickness <= 0) return;
        int left = x - ((thickness - 1) / 2);
        int right = left + thickness - 1;
        for (int xx = left; xx <= right; ++xx) {
            if (xx < wf.x || xx >= (wf.x + wf.w)) continue;
            M5.Display.drawFastVLine(xx, y, h, UI_FG);
        }
    };

    // Minor ticks every 100 Hz (no labels); skip major-tick positions.
    for (int hz = kWaterfallFreqMinHz; hz <= kWaterfallFreqMaxHz; hz += 100) {
        if ((hz % 500) == 0) continue;
        const float t = float(hz - kWaterfallFreqMinHz) / float(kWaterfallFreqMaxHz - kWaterfallFreqMinHz);
        if (t < 0.0f || t > 1.0f) continue;
        const int x = wf.x + (int)std::lround(t * float(wf.w - 1));
        draw_thick_tick(x, axis_y - 2, 5, 2);
    }

    for (int hz : kWaterfallMarksHz) {
        const float t = float(hz - kWaterfallFreqMinHz) / float(kWaterfallFreqMaxHz - kWaterfallFreqMinHz);
        if (t < 0.0f || t > 1.0f) continue;
        const int x = wf.x + (int)std::lround(t * float(wf.w - 1));
        draw_thick_tick(x, axis_y - 5, 11, 3);

        char label[8];
        std::snprintf(label, sizeof(label), "%d", hz);
        M5.Display.drawString(label, x, scale_top);
    }
}

static void draw_mode_row(const char* text) {
    (void)text;
    draw_row_frame(kTopLineIdx);
    draw_waterfall_freq_marks_locked();
}

static void draw_countdown_locked() {
    // Countdown is rendered in command button 0.
}

static void scale_bins_to_waterfall_cols(uint8_t* dst, const uint8_t* src, int len) {
    if (!dst) return;
    std::memset(dst, 0, kWaterfallCols);
    if (!src || len <= 0) return;
    if (len == kWaterfallCols) {
        std::memcpy(dst, src, kWaterfallCols);
        return;
    }
    for (int x = 0; x < kWaterfallCols; ++x) {
        int start = (int)((int64_t)x * len / kWaterfallCols);
        int end = (int)((int64_t)(x + 1) * len / kWaterfallCols);
        if (end <= start) end = start + 1;
        uint8_t v = 0;
        for (int i = start; i < end && i < len; ++i) {
            if (src[i] > v) v = src[i];
        }
        dst[x] = v;
    }
}

static void draw_waterfall_locked(const uint8_t rows[kWaterfallRows][kWaterfallCols],
                                  int head,
                                  bool has_data) {
    if (!g_header_widgets_enabled) return;
    const UiRect& r = g_layout.waterfall;
    if (r.w <= 0 || r.h <= 0) return;

    M5.Display.fillRect(r.x, r.y, r.w, r.h, UI_BG);
    if (!has_data) return;

    const int inner_w = r.w;
    const int inner_h = r.h;
    if (inner_w <= 0 || inner_h <= 0) return;

    for (int y = 0; y < inner_h; ++y) {
        int y_from_bottom = inner_h - 1 - y;
        int src_row_back = (int)((int64_t)y_from_bottom * kWaterfallRows / inner_h);
        int src_row = (head - 1 - src_row_back + kWaterfallRows) % kWaterfallRows;
        for (int x = 0; x < inner_w; ++x) {
            int src_col = (int)((int64_t)x * kWaterfallCols / inner_w);
            uint8_t v = rows[src_row][src_col];
            bool on = false;
            if (v >= 180) on = true;
            else if (v >= 145) on = (((x + y) & 1) == 0);
            else if (v >= 115) on = (((x ^ y) & 3) == 0);
            if (on) {
                M5.Display.drawPixel(r.x + x, r.y + y, UI_FG);
            }
        }
    }
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

    if (button_idx == 0 && g_countdown_enabled) {
        const bool odd = !g_countdown_even;
        const uint16_t bg = UI_BG;
        const uint16_t fg = UI_FG;
        const bool full_redraw = !g_countdown_visual_initialized || (g_countdown_visual_odd != odd);
        if (full_redraw) {
            M5.Display.fillRect(x0, row_y, w, kLineHeightPx, bg);
            M5.Display.drawRect(x0, row_y, w, kLineHeightPx, UI_FG);
            if (odd) {
                // Odd slot: use a heavier inner frame while keeping black-on-white text/background.
                for (int inset = 1; inset <= 3; ++inset) {
                    const int rw = w - (inset * 2);
                    const int rh = kLineHeightPx - (inset * 2);
                    if (rw <= 0 || rh <= 0) break;
                    M5.Display.drawRect(x0 + inset, row_y + inset, rw, rh, UI_FG);
                }
            }
            g_countdown_visual_initialized = true;
            g_countdown_visual_odd = odd;
        } else {
            // Keep button background stable and only refresh the center label area.
            const int inner_pad_x = 6;
            const int inner_pad_y = 6;
            const int inner_w = std::max(0, w - (inner_pad_x * 2));
            const int inner_h = std::max(0, kLineHeightPx - (inner_pad_y * 2));
            if (inner_w > 0 && inner_h > 0) {
                M5.Display.fillRect(x0 + inner_pad_x, row_y + inner_pad_y, inner_w, inner_h, bg);
            }
        }
        M5.Display.setTextColor(fg, bg);
        M5.Display.setTextDatum(middle_center);
        M5.Display.setTextSize(kCommandTextSize);
        char label[4];
        std::snprintf(label, sizeof(label), "%d", g_countdown_second);
        M5.Display.drawString(label, x0 + (w / 2), row_y + (kLineHeightPx / 2));
        return;
    }

    if (button_idx == 0) {
        g_countdown_visual_initialized = false;
    }

    const bool active = (button_idx == g_active_mode_button);
    const bool enabled = g_command_enabled[button_idx];
    const uint16_t bg = active ? UI_FG : UI_BG;
    const uint16_t fg = active ? UI_BG : UI_FG;

    M5.Display.fillRect(x0, row_y, w, kLineHeightPx, bg);
    M5.Display.drawRect(x0, row_y, w, kLineHeightPx, UI_FG);

    M5.Display.setTextColor(fg, bg);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextSize(kCommandTextSize);
    M5.Display.drawString(kCommandLabels[button_idx], x0 + (w / 2), row_y + (kLineHeightPx / 2));
    if (!enabled) {
        const int slash_y = row_y + (kLineHeightPx / 2);
        const int slash_w = std::max(0, w - 8);
        if (slash_w > 0) {
            M5.Display.drawFastHLine(x0 + 4, slash_y, slash_w, UI_FG);
        }
    }
}

static void draw_command_bar_locked() {
    g_countdown_visual_initialized = false;
    for (int i = 0; i < kCommandButtons; ++i) {
        draw_command_button_locked(i);
    }
}

const UiLayout& ui_layout() { return g_layout; }

void ui_set_paused(bool paused) { ui_paused = paused; }
bool ui_is_paused() { return ui_paused; }

void ui_init() {
    g_disp_mutex = xSemaphoreCreateMutex();
    g_wf_mutex = xSemaphoreCreateMutex();
    BaseType_t ret = xTaskCreatePinnedToCore(ui_display_task, "ui_display", 3072, nullptr, 1, &g_disp_task, 0);
    if (ret != pdPASS) {
        g_disp_task = nullptr;
    }

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
    {
        const int row_y = line_y(kTopLineIdx);
        const int cd_w = std::max(190, g_layout.screen_w / 4);
        g_layout.countdown = {g_layout.screen_w - cd_w - 8, row_y + 4, cd_w, 26};
        const int wf_x = 0;
        const int wf_y = row_y + 34;
        int wf_w = g_layout.screen_w;
        if (wf_w < 16) wf_w = 16;
        int wf_h = (row_y + kLineHeightPx) - wf_y - 4;
        if (wf_h < 8) wf_h = 8;
        g_layout.waterfall = {wf_x, wf_y, wf_w, wf_h};
    }
    g_layout.line_h = kLineHeightPx;

    draw_mode_row("");
    draw_countdown_locked();
    for (int i = 0; i < RX_LINES; ++i) {
        draw_text_row(i, "", false);
    }
    draw_command_bar_locked();
    request_display_flush();
}

void ui_set_waterfall_row(int row, const uint8_t* bins, int len) {
    if (row < 0 || !bins || len <= 0 || !g_wf_mutex) return;
    if (xSemaphoreTake(g_wf_mutex, 0) != pdTRUE) return;
    const int idx = row % kWaterfallRows;
    scale_bins_to_waterfall_cols(g_waterfall_ring[idx], bins, len);
    g_waterfall_has_data = true;
    g_waterfall_dirty_flag = true;
    xSemaphoreGive(g_wf_mutex);
}

void ui_push_waterfall_row(const uint8_t* bins, int len) {
    if (!bins || len <= 0 || !g_wf_mutex) return;
    if (xSemaphoreTake(g_wf_mutex, 0) != pdTRUE) return;
    scale_bins_to_waterfall_cols(g_waterfall_ring[g_waterfall_head], bins, len);
    g_waterfall_head = (g_waterfall_head + 1) % kWaterfallRows;
    g_waterfall_has_data = true;
    g_waterfall_dirty_flag = true;
    xSemaphoreGive(g_wf_mutex);
}

void ui_clear_waterfall() {
    if (!g_wf_mutex) return;
    if (xSemaphoreTake(g_wf_mutex, portMAX_DELAY) == pdTRUE) {
        std::memset(g_waterfall_ring, 0, sizeof(g_waterfall_ring));
        g_waterfall_head = 0;
        g_waterfall_has_data = false;
        g_waterfall_dirty_flag = false;
        xSemaphoreGive(g_wf_mutex);
    }
    if (!g_header_widgets_enabled) return;
    DispGuard guard;
    draw_waterfall_locked(g_waterfall_ring, g_waterfall_head, false);
    request_display_flush();
}

void ui_draw_waterfall() {
    if (!g_header_widgets_enabled || !g_wf_mutex) return;
    static uint8_t snapshot[kWaterfallRows][kWaterfallCols];
    int head = 0;
    bool has_data = false;
    if (xSemaphoreTake(g_wf_mutex, 0) != pdTRUE) return;
    std::memcpy(snapshot, g_waterfall_ring, sizeof(snapshot));
    head = g_waterfall_head;
    has_data = g_waterfall_has_data;
    g_waterfall_dirty_flag = false;
    g_waterfall_last_draw_ms = now_ms();
    xSemaphoreGive(g_wf_mutex);

    DispGuard guard;
    draw_waterfall_locked(snapshot, head, has_data);
    request_display_flush();
}

void ui_draw_countdown(float fraction, bool even_slot, int offset_hz) {
    (void)offset_hz;
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;

    int sec = static_cast<int>(fraction * 15.0f);
    if (sec < 0) sec = 0;
    if (sec > 12) sec = 12;

    const bool changed = (g_countdown_even != even_slot) ||
                         (g_countdown_second != sec);
    if (!changed) return;

    g_countdown_even = even_slot;
    g_countdown_second = sec;

    if (!g_countdown_enabled) return;
    DispGuard guard;
    draw_command_button_locked(0);
    request_display_flush();
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
    // Format: "<line> <freq(4)> <snr(3)> <message>"
    // freq is right-aligned/padded to 4 chars; SNR is always signed 3 chars.
    std::snprintf(buf, sizeof(buf), "%d %4d %+03d %s",
                  line_no, l.offset_hz, l.snr, l.text.c_str());
    draw_text_row(row_idx, buf, selected);
}

void ui_draw_rx(int flash_index) {
    if (!rx_lines.empty() && flash_index < 0) {
        if (rx_page == last_page && last_drawn_lines.size() == rx_lines.size()) {
            bool same = true;
            for (size_t i = 0; i < rx_lines.size(); ++i) {
                if (rx_lines[i].text != last_drawn_lines[i].text ||
                    rx_lines[i].snr != last_drawn_lines[i].snr ||
                    rx_lines[i].offset_hz != last_drawn_lines[i].offset_hz ||
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
    request_display_flush();

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

    request_display_flush();
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

    request_display_flush();
}

void ui_draw_mode_box(const char* mode_label) {
    (void)mode_label;
    DispGuard guard;
    g_header_widgets_enabled = true;
    draw_mode_row("");
    if (g_header_widgets_enabled) {
        draw_countdown_locked();
    }
    request_display_flush();
}

int ui_command_hit_test(int x, int y) {
    const UiRect& bar = g_layout.command_bar;
    if (!bar.contains(x, y)) return -1;
    int idx = (x * kCommandButtons) / g_layout.screen_w;
    if (idx < 0 || idx >= kCommandButtons) return -1;
    if (!g_command_enabled[idx]) return -1;
    return idx;
}

void ui_draw_command_bar() {
    DispGuard guard;
    draw_command_bar_locked();
    request_display_flush();
}

void ui_set_active_mode_button(char mode_key) {
    int idx = command_index_from_key(mode_key);

    if (idx == g_active_mode_button) return;
    g_active_mode_button = idx;

    DispGuard guard;
    draw_command_bar_locked();
    request_display_flush();
}

void ui_set_countdown_enabled(bool enabled) {
    if (g_countdown_enabled == enabled) return;
    g_countdown_enabled = enabled;
    DispGuard guard;
    draw_command_button_locked(0);
    request_display_flush();
}

void ui_toggle_countdown_enabled() {
    ui_set_countdown_enabled(!g_countdown_enabled);
}

bool ui_is_countdown_enabled() {
    return g_countdown_enabled;
}

void ui_set_command_enabled(char command_key, bool enabled) {
    int idx = command_index_from_key(command_key);
    if (idx < 0 || idx >= kCommandButtons) return;
    if (g_command_enabled[idx] == enabled) return;
    g_command_enabled[idx] = enabled;
    if (!enabled && g_active_mode_button == idx) {
        g_active_mode_button = -1;
    }

    DispGuard guard;
    draw_command_bar_locked();
    request_display_flush();
}

bool ui_waterfall_dirty() {
    return g_waterfall_dirty_flag;
}

void ui_draw_waterfall_if_dirty() {
    if (!g_header_widgets_enabled) return;
    if (!g_waterfall_dirty_flag) return;
    const int64_t t = now_ms();
    if ((t - g_waterfall_last_draw_ms) < kWaterfallDrawMinMs) return;
    ui_draw_waterfall();
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

    request_display_flush();
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

