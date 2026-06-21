# Proyecto Robot 2R Calibración
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS ${app_sources}
    REQUIRES nvs_flash esp_wifi esp_netif esp_event lwip
)
