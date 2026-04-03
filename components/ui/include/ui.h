#pragma once
#include <vector>
#include <string>
#include <stdint.h>

struct UiRect {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
    bool contains(int px, int py) const {
        return px >= x && py >= y && px < (x + w) && py < (y + h);
    }
};

struct UiLayout {
    int screen_w = 0;
    int screen_h = 0;
    UiRect mode_box;
    UiRect waterfall;
    UiRect countdown;
    UiRect text_area;
    UiRect command_bar;
    int line_h = 0;
};

// A lightweight RX line format you can fill from your decoder
struct UiRxLine {
    std::string text;  // already formatted for display
    int snr = 0;
    int offset_hz = 0; // audio-bin offset in Hz relative to passband center
    int slot_id = 0;   // 0 = even slot (0/30s), 1 = odd slot (15/45s)
    std::string field1; // parsed token 1 (call/CQ marker)
    std::string field2; // parsed token 2
    std::string field3; // parsed token 3 (grid/report/etc)
    bool is_cq = false;
    bool is_to_me = false;
};

void ui_init();
void ui_set_waterfall_row(int row, const uint8_t* bins, int len);
// Push a new row into the waterfall ring buffer (advances head). UI task must flush.
void ui_push_waterfall_row(const uint8_t* bins, int len);
void ui_clear_waterfall();
void ui_draw_waterfall();
void ui_draw_waterfall_if_dirty();
bool ui_waterfall_dirty();
void ui_draw_countdown(float fraction, bool even_slot, int offset_hz);  // 0.0-1.0 fill of the countdown bar
void ui_set_rx_list(const std::vector<UiRxLine>& lines);
void ui_set_paused(bool paused);
bool ui_is_paused();
void ui_draw_rx(int flash_index = -1);
void ui_force_redraw_rx();
const UiLayout& ui_layout();
void ui_draw_mode_box(const char* mode_label);
int ui_rx_hit_test(int x, int y); // returns absolute index or -1
int ui_text_line_hit_test(int x, int y); // returns visible line index [0..5] or -1
void ui_rx_scroll(int delta);     // delta = -1 prev, +1 next
void ui_draw_command_bar();
// Highlight one mode command button (S/R/T/Q/M/B/C/D) as active using inverted colors.
// Pass 0 or unsupported key to clear highlight.
void ui_set_active_mode_button(char mode_key);
void ui_set_countdown_enabled(bool enabled);
void ui_toggle_countdown_enabled();
bool ui_is_countdown_enabled();
// ESC button TX indicator: false=1px frame, true=5px frame.
void ui_set_tx_indicator(bool active);
// Enable/disable a command button by key label (e.g. 'C', 'R', 'S', ...).
// Disabled buttons are not hit-testable.
void ui_set_command_enabled(char command_key, bool enabled);
int ui_command_hit_test(int x, int y); // returns command button index or -1
// Colors: pass same-length slot_colors (0 even->green, 1 odd->red) for next/queue
void ui_draw_tx(const std::string& next, const std::vector<std::string>& queue, int page, int selected, const std::vector<bool>& mark_delete, const std::vector<int>& slot_colors = {});
// Returns selected absolute index or -1 if none
int ui_handle_rx_key(char c);
// Generic list draw (6 lines per page)
void ui_draw_list(const std::vector<std::string>& lines, int page, int highlight_abs = -1);
void ui_draw_debug(const std::vector<std::string>& lines, int page);
