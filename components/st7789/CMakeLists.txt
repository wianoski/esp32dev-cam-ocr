FILE(GLOB_RECURSE app_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.*)

idf_component_register(srcs "main/st7789.c"
    "main/fontx.c"
    "main/decode_png.c"
    "main/pngle.c"
    )

# tjpgd library does not exist in ESP32-S2 ROM.
if(IDF_TARGET STREQUAL "esp32")
    list(APPEND srcs "decode_jpeg.c")
endif()

if(IDF_TARGET STREQUAL "esp32c3")
    list(APPEND srcs "decode_jpeg.c")
endif()

