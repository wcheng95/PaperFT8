#include "ui.h"
#include <M5Unified.h>
#include <cstring>
#include "freertos/semphr.h"

static constexpr int RX_LINES = 6;
static constexpr int MODE_BOX_W = 102;
static constexpr int MODE_BOX_H = 102;
static constexpr int COUNTDOWN_H = 6;
static constexpr float WF_HZ_MIN = 198.4f;
static constexpr float WF_HZ_MAX = 3001.6f;
static constexpr float WF_HZ_PER_PX = 6.4f;
static bool ui_paused = false;      // waterfall updates paused

static std::vector<uint8_t> waterfall;
static int waterfall_w = 0;
static int waterfall_h = 0;
static int waterfall_head = 0;
static bool waterfall_dirty = false;
static UiLayout g_layout;

static std::vector<UiRxLine> rx_lines;
static int rx_page = 0;
static int rx_selected = -1;  // global index into rx_lines
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

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

const UiLayout& ui_layout() { return g_layout; }

void ui_set_paused(bool paused) { ui_paused = paused; }
bool ui_is_paused() { return ui_paused; }

bool ui_waterfall_dirty() { return waterfall_dirty; }
void ui_draw_waterfall_if_dirty() { if (waterfall_dirty) ui_draw_waterfall(); }

// Draw TX view: line 1 = next, lines 2-6 from queue/page
// slot_colors: optional per-line slot parity (0 even, 1 odd) for coloring text
void ui_draw_tx(const std::string& next, const std::vector<std::string>& queue, int page, int selected, const std::vector<bool>& mark_delete, const std::vector<int>& slot_colors) {
    const int line_h = g_layout.line_h;
    const int start_y = g_layout.text_area.y;

    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    // Line 1: next (always)
    M5.Display.fillRect(0, start_y, g_layout.screen_w, line_h, TFT_BLACK);
    uint16_t next_color = TFT_WHITE;
    if (slot_colors.size() >= 1) {
        next_color = (slot_colors[0] & 1) ? TFT_RED : TFT_GREEN;
    }
    M5.Display.setTextColor(next_color, TFT_BLACK);
    M5.Display.setCursor(0, start_y);
    M5.Display.printf("1 %s", next.c_str());

    // Lines 2-6: queue items based on page
    int start_idx = page * 5; // show up to 5 items after next
    for (int i = 0; i < 5; ++i) {
        int idx = start_idx + i;
        int y = start_y + (i + 1) * line_h;
        M5.Display.fillRect(0, y, g_layout.screen_w, line_h, TFT_BLACK);
        if (idx < (int)queue.size()) {
            bool del = (idx < (int)mark_delete.size() && mark_delete[idx]);
            bool sel = (idx == selected);
            uint16_t bg = sel ? rgb565(30, 30, 60) : (del ? rgb565(60, 30, 30) : TFT_BLACK);
            M5.Display.fillRect(0, y, g_layout.screen_w, line_h, bg);
            uint16_t fg = TFT_WHITE;
            if (slot_colors.size() > (size_t)(idx + 1)) {
                fg = (slot_colors[idx + 1] & 1) ? TFT_RED : TFT_GREEN;
            }
            M5.Display.setTextColor(fg, bg);
            M5.Display.setCursor(0, y);
            M5.Display.printf("%d %s", i + 2, queue[idx].c_str());
        }
    }
    M5.Display.endWrite();
}

void ui_init() {
    g_disp_mutex = xSemaphoreCreateMutex();
    auto cfg = M5.config();
    cfg.output_power = true;
    cfg.external_rtc = false;
    M5.begin(cfg);
    M5.Display.setRotation(1);
    if (M5.Display.width() < M5.Display.height()) {
        M5.Display.setRotation(3);
    }
    M5.Display.fillScreen(TFT_BLACK);
    g_layout.screen_w = M5.Display.width();
    g_layout.screen_h = M5.Display.height();
    g_layout.mode_box = {0, 0, MODE_BOX_W, MODE_BOX_H};
    g_layout.waterfall = {MODE_BOX_W, 0, g_layout.screen_w - MODE_BOX_W, MODE_BOX_H - COUNTDOWN_H};
    g_layout.countdown = {MODE_BOX_W, MODE_BOX_H - COUNTDOWN_H, g_layout.screen_w - MODE_BOX_W, COUNTDOWN_H};
    g_layout.text_area = {0, MODE_BOX_H, g_layout.screen_w, g_layout.screen_h - MODE_BOX_H};
    g_layout.line_h = g_layout.text_area.h / RX_LINES;
    int target_w = (int)((WF_HZ_MAX - WF_HZ_MIN) / WF_HZ_PER_PX);
    if (target_w < 1) target_w = g_layout.waterfall.w;
    waterfall_w = std::min(g_layout.waterfall.w, target_w);
    waterfall_h = g_layout.waterfall.h;
    waterfall.assign(waterfall_w * waterfall_h, 0);
    waterfall_head = 0;
    ui_draw_countdown(0.0f, true, 1500);
}

void ui_set_waterfall_row(int row, const uint8_t* bins, int len) {
    if (len > waterfall_w) len = waterfall_w;
    if (row < 0 || row >= waterfall_h) return;
    memcpy(&waterfall[row * waterfall_w], bins, len);
}

void ui_push_waterfall_row(const uint8_t* bins, int len) {
    if (ui_paused) return;
    if (len > waterfall_w) len = waterfall_w;
    memcpy(&waterfall[waterfall_head * waterfall_w], bins, len);
    if (len < waterfall_w) {
        memset(&waterfall[waterfall_head * waterfall_w] + len, 0, waterfall_w - len);
    }
    waterfall_head = (waterfall_head + 1) % waterfall_h;
    waterfall_dirty = true;
}

void ui_clear_waterfall() {
    std::fill(waterfall.begin(), waterfall.end(), 0);
    waterfall_head = 0;
    waterfall_dirty = true;
    ui_draw_waterfall();
}

void ui_draw_waterfall() {
    waterfall_dirty = false;
    if (ui_paused) return;
    DispGuard guard;
    // Clear the full waterfall area; we may draw a narrower buffer inside it.
    M5.Display.fillRect(g_layout.waterfall.x, g_layout.waterfall.y,
                        g_layout.waterfall.w, g_layout.waterfall.h, TFT_BLACK);
    M5.Display.startWrite();
    int dst_y = g_layout.waterfall.y;
    for (int i = 0; i < waterfall_h; ++i) {
        int src = (waterfall_head + i) % waterfall_h;
        const uint8_t* row = &waterfall[src * waterfall_w];
        for (int x = 0; x < waterfall_w; ++x) {
            uint8_t v = row[x];
            // Monochrome for e-paper
            uint16_t c = (v > 32) ? TFT_WHITE : TFT_BLACK;
            M5.Display.drawPixel(g_layout.waterfall.x + x, dst_y + i, c);
        }
    }
    M5.Display.endWrite();
}

static inline int hz_to_x(int hz) {
    float x = (hz - WF_HZ_MIN) / WF_HZ_PER_PX;
    int px = (int)x;
    if (px < 0) px = 0;
    if (px > waterfall_w - 1) px = waterfall_w - 1;
    return px;
}

static void ui_draw_offset_cursor_dot(int offset_hz) {
    const int y = g_layout.countdown.y;
    const int cy = y + (g_layout.countdown.h / 2);
    int cx = hz_to_x(offset_hz) + g_layout.countdown.x;

    // 3x3 dot centered at (cx, cy)
    int x0 = cx - 1;
    int y0 = cy - 1;

    // clamp so we don't draw outside
    if (x0 < g_layout.countdown.x) x0 = g_layout.countdown.x;
    if (y0 < y) y0 = y;
    if (x0 + 3 > g_layout.countdown.x + g_layout.countdown.w) x0 = g_layout.countdown.x + g_layout.countdown.w - 3;
    if (y0 + 3 > y + g_layout.countdown.h) y0 = y + g_layout.countdown.h - 3;

    if(offset_hz>=200 && offset_hz <=3000)
      M5.Display.fillRect(x0, y0, 5, 3, rgb565(0, 80, 160));   // blue cursor dot
}

void ui_draw_countdown(float fraction, bool even_slot, int offset_hz) {
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    int filled = (int)(fraction * g_layout.countdown.w);
    int y = g_layout.countdown.y;
    // Draw a faint background to make the bar visible even at 0%
    DispGuard guard;
    M5.Display.fillRect(g_layout.countdown.x, y, g_layout.countdown.w, g_layout.countdown.h, rgb565(20, 20, 40));
    if (filled > 0) {
        uint16_t color = even_slot ? rgb565(0, 180, 0) : rgb565(180, 0, 0);
        M5.Display.fillRect(g_layout.countdown.x, y, filled, g_layout.countdown.h, color);
    }
    // draw cursor last so countdown never overwrites it
    ui_draw_offset_cursor_dot(offset_hz);
}

void ui_set_rx_list(const std::vector<UiRxLine>& lines) {
    rx_lines = lines;
    rx_page = 0;       // reset to first page
    rx_selected = -1;  // clear selection
    last_drawn_lines.clear();
    last_page = -1;
}

void ui_force_redraw_rx() {
    last_drawn_lines.clear();
    last_page = -1;
}

static void draw_rx_line(int y, const UiRxLine& l, int line_no, bool selected, bool more_indicator) {
    uint16_t color = TFT_WHITE;
    if (more_indicator) {
        color = rgb565(0, 255, 255); // cyan to indicate more pages
    } else if (l.is_to_me) {
        color = rgb565(255, 0, 0);
    } else if (l.is_cq) {
        color = rgb565(0, 220, 0);
    }
    // Sticky line number in first column
    uint16_t bg = selected ? rgb565(30, 30, 60) : TFT_BLACK;
    M5.Display.fillRect(0, y, g_layout.text_area.w, g_layout.line_h, bg);
    M5.Display.setTextColor(TFT_WHITE, bg);
    M5.Display.setCursor(0, y);
    M5.Display.printf("%d ", line_no);
    M5.Display.setTextColor(color, bg);
    M5.Display.printf("%s", l.text.c_str());
}

void ui_draw_rx(int flash_index) {
    const int line_h = g_layout.line_h;
    const int start_y = g_layout.text_area.y;
    // Only redraw when page changes or content changes, but always draw if list is empty
    if (!(rx_lines.empty()) && flash_index < 0) {
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
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    int start = rx_page * RX_LINES;
    for (int i = 0; i < RX_LINES; ++i) {
        int idx = start + i;
        int y = start_y + i * line_h;
        M5.Display.fillRect(0, y, g_layout.text_area.w, line_h, TFT_BLACK);
        if (idx < (int)rx_lines.size()) {
            bool selected = (idx == flash_index);
            bool more = (rx_page == 0 && rx_lines.size() > RX_LINES && i == RX_LINES - 1);
            draw_rx_line(y, rx_lines[idx], i + 1, selected, more);
        }
    }
    M5.Display.endWrite();

    // cache drawn content
    if (flash_index < 0) {
        last_page = rx_page;
        last_drawn_lines = rx_lines;
    } else {
        last_page = -1;
        last_drawn_lines.clear();
    }
}

// Simple keyboard: dot/‘.’ scroll forward page, comma/‘,’ scroll back.
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

// Simple numbered list drawing helper (6 lines/page), optional highlight by absolute index
void ui_draw_list(const std::vector<std::string>& lines, int page, int highlight_abs) {
    const int line_h = g_layout.line_h;
    const int start_y = g_layout.text_area.y;
    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    for (int i = 0; i < RX_LINES; ++i) {
        int idx = page * RX_LINES + i;
        int y = start_y + i * line_h;
        uint16_t bg = (idx == highlight_abs) ? rgb565(30, 30, 60) : TFT_BLACK;
        M5.Display.fillRect(0, y, g_layout.text_area.w, line_h, bg);
        if (idx < (int)lines.size()) {
            M5.Display.setTextColor(TFT_WHITE, bg);
            M5.Display.setCursor(0, y);
            M5.Display.printf("%d %s", i + 1, lines[idx].c_str());
        }
    }
    M5.Display.endWrite();
}

void ui_draw_debug(const std::vector<std::string>& lines, int page) {
    const int line_h = 19;
    const int start_y = g_layout.text_area.y;
    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    for (int i = 0; i < RX_LINES; ++i) {
        int idx = page * RX_LINES + i;
        int y = start_y + i * line_h;
        M5.Display.fillRect(0, y, g_layout.text_area.w, line_h, TFT_BLACK);
        if (idx < (int)lines.size()) {
            M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
            M5.Display.setCursor(0, y);
            M5.Display.printf("%s", lines[idx].c_str());
        }
    }
    M5.Display.endWrite();
}

void ui_draw_mode_box(const char* mode_label) {
    DispGuard guard;
    M5.Display.fillRect(g_layout.mode_box.x, g_layout.mode_box.y, g_layout.mode_box.w, g_layout.mode_box.h, TFT_BLACK);
    M5.Display.drawRect(g_layout.mode_box.x, g_layout.mode_box.y, g_layout.mode_box.w, g_layout.mode_box.h, rgb565(80, 80, 80));
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setTextSize(2);
    int cx = g_layout.mode_box.x + g_layout.mode_box.w / 2;
    int cy = g_layout.mode_box.y + g_layout.mode_box.h / 2;
    M5.Display.setCursor(cx - 6, cy - 8);
    M5.Display.printf("%s", mode_label ? mode_label : "");
}

int ui_rx_hit_test(int x, int y) {
    if (!g_layout.text_area.contains(x, y)) return -1;
    int line = (y - g_layout.text_area.y) / g_layout.line_h;
    if (line < 0 || line >= RX_LINES) return -1;
    int idx = rx_page * RX_LINES + line;
    if (idx < 0 || idx >= (int)rx_lines.size()) return -1;
    return idx;
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


