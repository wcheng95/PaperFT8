#include "ui.h"
#include <M5Unified.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <cmath>
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ter_u20b_lvgl.h"

static constexpr int RX_LINES = 6;

static constexpr int kLineCount = 8;
static constexpr int kLineHeightPx = 67;
static constexpr int kTopLineIdx = 0;
static constexpr int kTextFirstLineIdx = 1;
static constexpr int kCommandLineIdx = 7;
static constexpr int kCommandButtons = 12;
static constexpr int kTextLeftPx = 24;
static constexpr int kBodyTextSize = 4;
static constexpr int kCommandTextSize = 3;
static constexpr int kTopScaleTextSize = 2;
static constexpr int kTextBottomCushionPx = 3;
// Use LVGL font for RX/Tx text rows.
static constexpr int kRxFontScale = 2;
static constexpr int kRxFontSpacing = 0;
static constexpr int kFontMaxScale = kRxFontScale;

/*
 * ePaper-friendly waterfall:
 * - Fixed 896x33 area, split into three 896x11 snapshot bands.
 * - One band update at ~4s, ~8s, ~12s in each 15s FT8 slot.
 * - Each update refreshes only the target 896x11 rectangle.
 * - Set kWaterfallSnapshotsEnabled=false to disable quickly.
 */
static constexpr bool kWaterfallSnapshotsEnabled = true;
static constexpr int kWaterfallWidth = 896;
static constexpr int kWaterfallBandHeight = 11;
static constexpr int kWaterfallBandCount = 3;
static constexpr int kWaterfallTotalHeight = kWaterfallBandHeight * kWaterfallBandCount;
static constexpr int kWaterfallRowBytes = kWaterfallWidth / 8;  // 112
static constexpr int kWaterfallBandBitmapBytes = kWaterfallRowBytes * kWaterfallBandHeight;  // 1232
static constexpr int kWaterfallSnapshotTopSec = 4;
static constexpr int kWaterfallSnapshotMidSec = 8;
static constexpr int kWaterfallSnapshotBotSec = 12;
static constexpr int kWaterfallRecentRows = 16;  // keep recent samples for one snapshot build

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
static constexpr int kCmdIdxEsc = 1;
static constexpr int kCmdIdxPrev = 10;
static constexpr int kCmdIdxNext = 11;

static bool ui_paused = false;
static UiLayout g_layout;
static int g_y_offset = 0;
static int g_active_mode_button = -1;
static bool g_command_enabled[kCommandButtons] = {
    true, true, true, true, true, true, true, true, true, true, true, true
};
static bool g_header_widgets_enabled = true;
static bool g_countdown_enabled = true;  // runtime toggle via command button 0
static bool g_nav_prev_available = false;
static bool g_nav_next_available = false;
static bool g_tx_indicator_active = false;

static bool g_countdown_even = true;
static int g_countdown_second = 0;  // 0..12 within 15s slot
static bool g_countdown_visual_initialized = false;
static bool g_countdown_visual_odd = false;

static uint8_t g_waterfall_recent[kWaterfallRecentRows][kWaterfallWidth] = {};
static int g_waterfall_recent_head = 0;   // next write row
static int g_waterfall_recent_count = 0;  // valid rows in recent ring
static int g_waterfall_slot_parity = -1;  // 0 even / 1 odd
static bool g_waterfall_band_done[kWaterfallBandCount] = {false, false, false};
static uint8_t g_waterfall_band_bitmap[kWaterfallBandBitmapBytes] = {};

static std::vector<UiRxLine> rx_lines;
static int rx_page = 0;
static int rx_selected = -1;
struct RxDrawCacheEntry {
    std::string text;
    int snr = 0;
    int offset_hz = 0;
    bool is_cq = false;
    bool is_to_me = false;
};
static std::vector<RxDrawCacheEntry> last_drawn_cache;
static int last_page = -1;
static std::string g_visible_rows[RX_LINES];

struct GlyphBitmapCache {
    bool ready = false;
    bool valid = false;
    int w = 0;
    int h = 0;
    int ofs_x = 0;
    int ofs_y = 0;
    int advance = 0;
    std::vector<uint8_t> bits;
};

static GlyphBitmapCache g_glyph_cache[kFontMaxScale + 1][128];

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

static int text_row_y(int row_idx) {
    return line_y(kTextFirstLineIdx + row_idx);
}

static void refresh_text_row_locked(int row_idx) {
    if (row_idx < 0 || row_idx >= RX_LINES) return;
    M5.Display.display(0, text_row_y(row_idx), g_layout.screen_w, kLineHeightPx);
}

static int glyph_advance_px(const lv_font_glyph_dsc_t& gd, int scale) {
    if (scale < 1) scale = 1;
    // LVGL adv_w is 4.4 fixed-point; apply scale before rounding.
    return (int)(((int32_t)gd.adv_w * scale + 8) >> 4);
}

static bool fetch_glyph(uint32_t ch, uint32_t next, lv_font_glyph_dsc_t& gd, const uint8_t*& bmp) {
    if (lv_font_get_glyph_dsc(&ter_u20b_lvgl, &gd, ch, next)) {
        bmp = lv_font_get_glyph_bitmap(&ter_u20b_lvgl, ch);
        if (bmp) return true;
    }
    if (ch != '?') {
        if (lv_font_get_glyph_dsc(&ter_u20b_lvgl, &gd, '?', 0)) {
            bmp = lv_font_get_glyph_bitmap(&ter_u20b_lvgl, '?');
            if (bmp) return true;
        }
    }
    return false;
}

static inline void set_bitmap_bit(uint8_t* bits, int row_bytes, int x, int y) {
    bits[y * row_bytes + (x >> 3)] |= static_cast<uint8_t>(0x80u >> (x & 7));
}

static const GlyphBitmapCache* get_cached_glyph(uint32_t ch, int scale) {
    if (scale < 1 || scale > kFontMaxScale) return nullptr;
    const uint32_t code = (ch >= 32 && ch <= 126) ? ch : static_cast<uint32_t>('?');
    auto& e = g_glyph_cache[scale][code];
    if (e.ready) {
        return e.valid ? &e : nullptr;
    }

    e.ready = true;
    e.valid = false;
    e.bits.clear();
    e.w = 0;
    e.h = 0;
    e.ofs_x = 0;
    e.ofs_y = 0;
    e.advance = 0;

    lv_font_glyph_dsc_t gd{};
    const uint8_t* bmp = nullptr;
    if (!fetch_glyph(code, 0, gd, bmp) || !bmp) {
        return nullptr;
    }

    e.advance = glyph_advance_px(gd, scale);
    e.ofs_x = gd.ofs_x * scale;
    e.ofs_y = gd.ofs_y * scale;

    if (gd.box_w <= 0 || gd.box_h <= 0) {
        e.valid = true;
        return &e;
    }

    if (scale == 1) {
        const int src_row_bytes = (gd.box_w + 7) >> 3;
        const int src_size = src_row_bytes * gd.box_h;
        e.w = gd.box_w;
        e.h = gd.box_h;
        e.bits.assign(bmp, bmp + src_size);
        e.valid = true;
        return &e;
    }

    const int src_row_bytes = (gd.box_w + 7) >> 3;
    const int out_w = gd.box_w * scale;
    const int out_h = gd.box_h * scale;
    const int out_row_bytes = (out_w + 7) >> 3;
    e.w = out_w;
    e.h = out_h;
    e.bits.assign(out_row_bytes * out_h, 0);

    for (int gy = 0; gy < gd.box_h; ++gy) {
        for (int gx = 0; gx < gd.box_w; ++gx) {
            const uint8_t byte = bmp[(gy * src_row_bytes) + (gx >> 3)];
            const bool on = (byte & (0x80u >> (gx & 7))) != 0;
            if (!on) continue;
            const int ox = gx * scale;
            const int oy = gy * scale;
            for (int sy = 0; sy < scale; ++sy) {
                for (int sx = 0; sx < scale; ++sx) {
                    set_bitmap_bit(e.bits.data(), out_row_bytes, ox + sx, oy + sy);
                }
            }
        }
    }

    e.valid = true;
    return &e;
}

static int glyph_step_px(const GlyphBitmapCache& g) {
    // Keep natural advance, but never step less than the glyph's drawn right edge.
    const int right = g.ofs_x + g.w;
    int step = g.advance;
    if ((right + 1) > step) {
        step = right + 1;
    }
    return (step > 0) ? step : 1;
}

static int lvgl_text_width_px(const char* text, int scale, int letter_spacing) {
    if (!text || scale <= 0) return 0;
    const size_t n = std::strlen(text);
    int width = 0;
    for (size_t i = 0; i < n; ++i) {
        const uint32_t ch = static_cast<uint8_t>(text[i]);
        const GlyphBitmapCache* g = get_cached_glyph(ch, scale);
        if (!g) continue;
        width += glyph_step_px(*g);
        if (i + 1 < n) width += letter_spacing;
    }
    return width;
}

static void draw_lvgl_text(const char* text, int x, int y_center, bool center_align, int scale, int letter_spacing) {
    if (!text || !text[0] || scale <= 0) return;
    if (letter_spacing < 0) letter_spacing = 0;

    int cursor_x = x;
    if (center_align) {
        cursor_x -= lvgl_text_width_px(text, scale, letter_spacing) / 2;
    }
    const int line_h = (int)ter_u20b_lvgl.line_height * scale;
    const int top_y = y_center - (line_h / 2);

    const size_t n = std::strlen(text);
    for (size_t i = 0; i < n; ++i) {
        const uint32_t ch = static_cast<uint8_t>(text[i]);
        const GlyphBitmapCache* g = get_cached_glyph(ch, scale);
        if (!g) continue;

        const int glyph_x = cursor_x + g->ofs_x;
        const int glyph_y = top_y + g->ofs_y;
        if (g->w > 0 && g->h > 0 && !g->bits.empty()) {
            // Draw foreground only; row background is cleared once before text render.
            M5.Display.drawBitmap(glyph_x, glyph_y, g->bits.data(), g->w, g->h, UI_FG);
        }

        cursor_x += glyph_step_px(*g);
        if (i + 1 < n) cursor_x += letter_spacing;
    }
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
    M5.Display.setTextSize(kTopScaleTextSize);

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
    const int row_y = line_y(kTopLineIdx);
    // Header row without frame lines.
    M5.Display.fillRect(0, row_y, g_layout.screen_w, kLineHeightPx, UI_BG);
    draw_waterfall_freq_marks_locked();
}

static void draw_countdown_locked() {
    // Countdown is rendered in command button 0.
}

static void scale_bins_to_waterfall_width(uint8_t* dst, const uint8_t* src, int len) {
    if (!dst) return;
    std::memset(dst, 0, kWaterfallWidth);
    if (!src || len <= 0) return;
    if (len == kWaterfallWidth) {
        std::memcpy(dst, src, kWaterfallWidth);
        return;
    }
    for (int x = 0; x < kWaterfallWidth; ++x) {
        int start = (int)((int64_t)x * len / kWaterfallWidth);
        int end = (int)((int64_t)(x + 1) * len / kWaterfallWidth);
        if (end <= start) end = start + 1;
        uint8_t v = 0;
        for (int i = start; i < end && i < len; ++i) {
            if (src[i] > v) v = src[i];
        }
        dst[x] = v;
    }
}

static bool waterfall_pixel_on(uint8_t v, int x, int y) {
    if (v >= 180) return true;
    if (v >= 145) return ((x + y) & 1) == 0;
    if (v >= 115) return ((x ^ y) & 3) == 0;
    return false;
}

static bool build_waterfall_band_bitmap_locked() {
    std::memset(g_waterfall_band_bitmap, 0, sizeof(g_waterfall_band_bitmap));
    if (!kWaterfallSnapshotsEnabled) return true;
    if (!g_wf_mutex) return false;
    if (xSemaphoreTake(g_wf_mutex, 0) != pdTRUE) return false;

    const int count = g_waterfall_recent_count;
    const int head = g_waterfall_recent_head;
    for (int y = 0; y < kWaterfallBandHeight; ++y) {
        const int back = (kWaterfallBandHeight - 1) - y;  // newest sample at bottom row
        if (back >= count) continue;
        const int src_idx = (head - 1 - back + kWaterfallRecentRows) % kWaterfallRecentRows;
        const uint8_t* src = g_waterfall_recent[src_idx];
        uint8_t* dst = g_waterfall_band_bitmap + (y * kWaterfallRowBytes);
        for (int x = 0; x < kWaterfallWidth; ++x) {
            if (waterfall_pixel_on(src[x], x, y)) {
                dst[x >> 3] |= static_cast<uint8_t>(0x80u >> (x & 7));
            }
        }
    }

    xSemaphoreGive(g_wf_mutex);
    return true;
}

static bool draw_waterfall_snapshot_band(int band_idx) {
    if (!kWaterfallSnapshotsEnabled) return false;
    if (!g_header_widgets_enabled) return false;
    if (band_idx < 0 || band_idx >= kWaterfallBandCount) return false;
    const UiRect& r = g_layout.waterfall;
    if (r.w != kWaterfallWidth || r.h != kWaterfallTotalHeight) return false;

    if (!build_waterfall_band_bitmap_locked()) return false;

    const int x = r.x;
    const int y = r.y + band_idx * kWaterfallBandHeight;
    DispGuard guard;
    M5.Display.drawBitmap(x, y, g_waterfall_band_bitmap, kWaterfallWidth, kWaterfallBandHeight, UI_FG, UI_BG);
    M5.Display.display(x, y, kWaterfallWidth, kWaterfallBandHeight);
    return true;
}

static void waterfall_snapshot_tick(int slot_sec, bool even_slot) {
    if (!kWaterfallSnapshotsEnabled) return;
    const int slot_parity = even_slot ? 0 : 1;
    if (slot_parity != g_waterfall_slot_parity) {
        g_waterfall_slot_parity = slot_parity;
        for (int i = 0; i < kWaterfallBandCount; ++i) {
            g_waterfall_band_done[i] = false;
        }
    }

    struct SnapDef {
        int sec;
        int band_idx;
    };
    static constexpr SnapDef kSnaps[kWaterfallBandCount] = {
        {kWaterfallSnapshotTopSec, 0},
        {kWaterfallSnapshotMidSec, 1},
        {kWaterfallSnapshotBotSec, 2},
    };

    for (const auto& s : kSnaps) {
        if (slot_sec < s.sec) continue;
        if (g_waterfall_band_done[s.band_idx]) continue;
        if (draw_waterfall_snapshot_band(s.band_idx)) {
            g_waterfall_band_done[s.band_idx] = true;
        }
    }
}

static void draw_text_row(int row_idx, const char* text, bool outline, bool use_rx_font) {
    if (row_idx < 0 || row_idx >= RX_LINES) return;
    const int screen_line = kTextFirstLineIdx + row_idx;
    const int row_y = line_y(screen_line);
    const int text_y = row_y + (kLineHeightPx / 2) - kTextBottomCushionPx;
    g_visible_rows[row_idx] = text ? text : "";

    // Text lines 1-6: clear background only (no inter-line borders).
    M5.Display.fillRect(0, row_y, g_layout.screen_w, kLineHeightPx, UI_BG);
    if (use_rx_font) {
        draw_lvgl_text(text ? text : "", kTextLeftPx, text_y, false, kRxFontScale, kRxFontSpacing);
    } else {
        M5.Display.setTextColor(UI_FG, UI_BG);
        M5.Display.setTextDatum(middle_left);
        M5.Display.setTextSize(kBodyTextSize);
        M5.Display.drawString(text ? text : "", kTextLeftPx, text_y);
    }

    if (outline) {
        // Non-line highlight marker (keeps selection visible without row frames).
        const int marker = 8;
        const int mx = 8;
        const int my = row_y + (kLineHeightPx - marker) / 2;
        M5.Display.fillRect(mx, my, marker, marker, UI_FG);
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
        const bool full_redraw = !g_countdown_visual_initialized || (g_countdown_visual_odd != odd);
        if (full_redraw) {
            M5.Display.fillRect(x0, row_y, w, kLineHeightPx, bg);
            M5.Display.drawRect(x0, row_y, w, kLineHeightPx, UI_FG);
            if (odd) {
                // Odd slot: 5px total frame (outer + 4 inner).
                for (int inset = 1; inset <= 4; ++inset) {
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
        char label[4];
        std::snprintf(label, sizeof(label), "%d", g_countdown_second);
        M5.Display.setTextColor(UI_FG, UI_BG);
        M5.Display.setTextDatum(middle_center);
        M5.Display.setTextSize(kCommandTextSize);
        M5.Display.drawString(label, x0 + (w / 2), row_y + (kLineHeightPx / 2));
        return;
    }

    if (button_idx == 0) {
        g_countdown_visual_initialized = false;
    }

    const bool active = (button_idx == g_active_mode_button);
    const bool enabled = g_command_enabled[button_idx];
    bool nav_available = false;
    if (button_idx == kCmdIdxPrev) {
        nav_available = g_nav_prev_available;
    } else if (button_idx == kCmdIdxNext) {
        nav_available = g_nav_next_available;
    }
    const bool nav_highlight = (button_idx == kCmdIdxPrev || button_idx == kCmdIdxNext) && nav_available;
    const bool highlight = active || nav_highlight;
    const uint16_t bg = UI_BG;
    const uint16_t fg = UI_FG;

    M5.Display.fillRect(x0, row_y, w, kLineHeightPx, bg);
    M5.Display.drawRect(x0, row_y, w, kLineHeightPx, UI_FG);
    if (button_idx == kCmdIdxEsc && g_tx_indicator_active) {
        // TX active indicator on ESC: 5px total frame (outer + 4 inner).
        for (int inset = 1; inset <= 4; ++inset) {
            const int rw = w - (inset * 2);
            const int rh = kLineHeightPx - (inset * 2);
            if (rw <= 0 || rh <= 0) break;
            M5.Display.drawRect(x0 + inset, row_y + inset, rw, rh, UI_FG);
        }
    } else if (highlight) {
        // 3px inner frame to indicate active/available without inversion.
        constexpr int kInnerFrameStart = 2;
        constexpr int kInnerFrameWidth = 3;
        for (int t = 0; t < kInnerFrameWidth; ++t) {
            const int inset = kInnerFrameStart + t;
            const int rw = w - (inset * 2);
            const int rh = kLineHeightPx - (inset * 2);
            if (rw <= 0 || rh <= 0) break;
            M5.Display.drawRect(x0 + inset, row_y + inset, rw, rh, UI_FG);
        }
    }

    M5.Display.setTextColor(fg, bg);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextSize(kCommandTextSize);
    M5.Display.drawString(kCommandLabels[button_idx], x0 + (w / 2), row_y + (kLineHeightPx / 2));
    if (!enabled) {
        const int slash_y = row_y + (kLineHeightPx / 2);
        const int slash_w = std::max(0, w - 8);
        if (slash_w > 0) {
            M5.Display.drawFastHLine(x0 + 4, slash_y, slash_w, fg);
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
        const int wf_w = (g_layout.screen_w >= kWaterfallWidth) ? kWaterfallWidth : (g_layout.screen_w & ~7);
        const int wf_h = kWaterfallTotalHeight;
        const int wf_x = (g_layout.screen_w - wf_w) / 2;
        const int wf_y = row_y + (kLineHeightPx - wf_h);
        g_layout.waterfall = {wf_x, wf_y, wf_w, wf_h};
    }
    g_layout.line_h = kLineHeightPx;

    draw_mode_row("");
    draw_countdown_locked();
    for (int i = 0; i < RX_LINES; ++i) {
        draw_text_row(i, "", false, false);
    }
    draw_command_bar_locked();
    request_display_flush();
}

void ui_set_waterfall_row(int row, const uint8_t* bins, int len) {
    (void)row;
    ui_push_waterfall_row(bins, len);
}

void ui_push_waterfall_row(const uint8_t* bins, int len) {
    if (!kWaterfallSnapshotsEnabled) return;
    if (!bins || len <= 0 || !g_wf_mutex) return;
    if (xSemaphoreTake(g_wf_mutex, 0) != pdTRUE) return;
    scale_bins_to_waterfall_width(g_waterfall_recent[g_waterfall_recent_head], bins, len);
    g_waterfall_recent_head = (g_waterfall_recent_head + 1) % kWaterfallRecentRows;
    if (g_waterfall_recent_count < kWaterfallRecentRows) {
        ++g_waterfall_recent_count;
    }
    xSemaphoreGive(g_wf_mutex);
}

void ui_clear_waterfall() {
    if (!g_wf_mutex) return;
    if (xSemaphoreTake(g_wf_mutex, portMAX_DELAY) == pdTRUE) {
        std::memset(g_waterfall_recent, 0, sizeof(g_waterfall_recent));
        g_waterfall_recent_head = 0;
        g_waterfall_recent_count = 0;
        g_waterfall_slot_parity = -1;
        for (int i = 0; i < kWaterfallBandCount; ++i) {
            g_waterfall_band_done[i] = false;
        }
        std::memset(g_waterfall_band_bitmap, 0, sizeof(g_waterfall_band_bitmap));
        xSemaphoreGive(g_wf_mutex);
    }
    if (!g_header_widgets_enabled) return;
    DispGuard guard;
    M5.Display.fillRect(g_layout.waterfall.x, g_layout.waterfall.y, g_layout.waterfall.w, g_layout.waterfall.h, UI_BG);
    M5.Display.display(g_layout.waterfall.x, g_layout.waterfall.y, g_layout.waterfall.w, g_layout.waterfall.h);
}

void ui_draw_waterfall() {
    // Legacy entry point: waterfall is now updated only by 3 snapshot events per slot.
}

void ui_draw_countdown(float fraction, bool even_slot, int offset_hz) {
    (void)offset_hz;
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;

    int slot_sec = static_cast<int>(fraction * 15.0f);
    if (slot_sec < 0) slot_sec = 0;
    if (slot_sec > 14) slot_sec = 14;
    waterfall_snapshot_tick(slot_sec, even_slot);

    int sec = slot_sec;
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
    last_drawn_cache.clear();
    last_page = -1;
}

void ui_force_redraw_rx() {
    last_drawn_cache.clear();
    last_page = -1;
}

static void draw_rx_line(int row_idx, const UiRxLine& l, int line_no, bool selected) {
    char buf[160];
    // Format: "<line> <freq(4)> <snr(3)> <message>"
    // freq is right-aligned/padded to 4 chars; SNR is always signed 3 chars.
    std::snprintf(buf, sizeof(buf), "%d %4d %+03d %s",
                  line_no, l.offset_hz, l.snr, l.text.c_str());
    draw_text_row(row_idx, buf, selected, true);
}

static bool draw_rx_abs_line_locked(int absolute_idx, bool highlight) {
    if (absolute_idx < 0 || absolute_idx >= (int)rx_lines.size()) return false;
    const int row_idx = absolute_idx - (rx_page * RX_LINES);
    if (row_idx < 0 || row_idx >= RX_LINES) return false;
    draw_rx_line(row_idx, rx_lines[absolute_idx], row_idx + 1, highlight);
    return true;
}

void ui_draw_rx(int flash_index) {
    if (!rx_lines.empty() && flash_index < 0) {
        if (rx_page == last_page && last_drawn_cache.size() == rx_lines.size()) {
            bool same = true;
            for (size_t i = 0; i < rx_lines.size(); ++i) {
                if (rx_lines[i].text != last_drawn_cache[i].text ||
                    rx_lines[i].snr != last_drawn_cache[i].snr ||
                    rx_lines[i].offset_hz != last_drawn_cache[i].offset_hz ||
                    rx_lines[i].is_cq != last_drawn_cache[i].is_cq ||
                    rx_lines[i].is_to_me != last_drawn_cache[i].is_to_me) {
                    same = false;
                    break;
                }
            }
            if (same) return;
        }
    }

    DispGuard guard;
    g_nav_prev_available = (rx_page > 0);
    g_nav_next_available = ((rx_page + 1) * RX_LINES) < (int)rx_lines.size();

    const int start = rx_page * RX_LINES;
    for (int i = 0; i < RX_LINES; ++i) {
        const int idx = start + i;
        if (idx < (int)rx_lines.size()) {
            const bool selected = (idx == flash_index);
            draw_rx_line(i, rx_lines[idx], i + 1, selected);
        } else {
            draw_text_row(i, "", false, true);
        }
    }
    // RX page navigation cues on ^ / v buttons.
    draw_command_button_locked(kCmdIdxPrev);
    draw_command_button_locked(kCmdIdxNext);
    request_display_flush();

    if (flash_index < 0) {
        last_page = rx_page;
        last_drawn_cache.resize(rx_lines.size());
        for (size_t i = 0; i < rx_lines.size(); ++i) {
            last_drawn_cache[i].text = rx_lines[i].text;
            last_drawn_cache[i].snr = rx_lines[i].snr;
            last_drawn_cache[i].offset_hz = rx_lines[i].offset_hz;
            last_drawn_cache[i].is_cq = rx_lines[i].is_cq;
            last_drawn_cache[i].is_to_me = rx_lines[i].is_to_me;
        }
    } else {
        last_page = -1;
        last_drawn_cache.clear();
    }
}

void ui_flash_rx_line(int absolute_idx, bool highlight) {
    DispGuard guard;
    if (!draw_rx_abs_line_locked(absolute_idx, highlight)) return;
    const int row_idx = absolute_idx - (rx_page * RX_LINES);
    refresh_text_row_locked(row_idx);
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
            selected_idx = idx;
        }
    }

    return selected_idx;
}

void ui_draw_list(const std::vector<std::string>& lines, int page, int highlight_abs, bool use_rx_font) {
    DispGuard guard;
    g_nav_prev_available = (page > 0);
    g_nav_next_available = ((page + 1) * RX_LINES) < (int)lines.size();

    for (int i = 0; i < RX_LINES; ++i) {
        int idx = page * RX_LINES + i;
        if (idx < (int)lines.size()) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%d %s", i + 1, lines[idx].c_str());
            draw_text_row(i, buf, idx == highlight_abs, use_rx_font);
        } else {
            draw_text_row(i, "", idx == highlight_abs, use_rx_font);
        }
    }

    draw_command_button_locked(kCmdIdxPrev);
    draw_command_button_locked(kCmdIdxNext);
    request_display_flush();
}

void ui_draw_debug(const std::vector<std::string>& lines, int page) {
    DispGuard guard;
    g_nav_prev_available = (page > 0);
    g_nav_next_available = ((page + 1) * RX_LINES) < (int)lines.size();

    for (int i = 0; i < RX_LINES; ++i) {
        int idx = page * RX_LINES + i;
        if (idx < (int)lines.size()) {
            draw_text_row(i, lines[idx].c_str(), false, false);
        } else {
            draw_text_row(i, "", false, false);
        }
    }

    draw_command_button_locked(kCmdIdxPrev);
    draw_command_button_locked(kCmdIdxNext);
    request_display_flush();
}

void ui_get_visible_text_lines(std::vector<std::string>& out) {
    out.clear();
    out.reserve(RX_LINES);
    for (int i = 0; i < RX_LINES; ++i) {
        out.push_back(g_visible_rows[i]);
    }
}

void ui_set_visible_text_line(int row_idx, const std::string& text) {
    if (row_idx < 0 || row_idx >= RX_LINES) return;
    g_visible_rows[row_idx] = text;
}

void ui_get_rx_page_info(int& current_page, int& total_pages) {
    total_pages = (int)rx_lines.size() <= 0 ? 1 : (((int)rx_lines.size() + RX_LINES - 1) / RX_LINES);
    if (total_pages < 1) total_pages = 1;
    current_page = rx_page + 1;
    if (current_page < 1) current_page = 1;
    if (current_page > total_pages) current_page = total_pages;
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
    g_nav_prev_available = false;
    g_nav_next_available = false;

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

void ui_set_tx_indicator(bool active) {
    if (g_tx_indicator_active == active) return;
    g_tx_indicator_active = active;
    DispGuard guard;
    draw_command_button_locked(kCmdIdxEsc);
    request_display_flush();
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
    return false;
}

void ui_draw_waterfall_if_dirty() {
    // Legacy entry point kept for call-site compatibility.
}

void ui_draw_tx(const std::string& next,
                const std::vector<std::string>& queue,
                int page,
                int selected,
                const std::vector<bool>& mark_delete,
                const std::vector<int>& slot_colors) {
    (void)slot_colors;

    DispGuard guard;
    const int start_idx = page * 5;
    g_nav_prev_available = (page > 0);
    g_nav_next_available = (start_idx + 5) < (int)queue.size();

    {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "1 %s", next.c_str());
        draw_text_row(0, buf, false, true);
    }

    for (int row = 1; row < RX_LINES; ++row) {
        int idx = start_idx + (row - 1);
        if (idx < (int)queue.size()) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%d %s", row + 1, queue[idx].c_str());
            bool outline = (idx == selected) ||
                           (idx < (int)mark_delete.size() && mark_delete[idx]);
            draw_text_row(row, buf, outline, true);
        } else {
            draw_text_row(row, "", false, true);
        }
    }

    draw_command_button_locked(kCmdIdxPrev);
    draw_command_button_locked(kCmdIdxNext);
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
