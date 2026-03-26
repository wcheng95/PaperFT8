idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES M5Unified M5GFX
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)
