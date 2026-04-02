idf_component_register(
    SRCS "ui.cpp" "mic_input.cpp" "ter_u20b_lvgl.c"
    INCLUDE_DIRS "include"
    REQUIRES M5Unified M5GFX lvgl__lvgl
)
