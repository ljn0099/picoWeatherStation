if(NOT DEFINED WIFI_SSID OR WIFI_SSID STREQUAL "")
    message(FATAL_ERROR "You must define WIFI_SSID (e.g. -DWIFI_SSID=\"YourWiFiName\")")
endif()

if(NOT DEFINED WIFI_PASSWORD OR WIFI_PASSWORD STREQUAL "")
    message(FATAL_ERROR "You must define WIFI_PASSWORD (e.g. -DWIFI_PASSWORD=\"YourWiFiPassword\")")
endif()

add_executable(picoWeatherStation
    main.c
    queues.c
    ntp.c
)

target_compile_definitions(picoWeatherStation PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(picoWeatherStation PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(picoWeatherStation
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_gpio
    hardware_adc
    hardware_watchdog
    sensor_dps310
    sensor_hdc3022
    sensor_ltr390
    sensor_pcf8523
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_enable_stdio_usb(picoWeatherStation 0)
pico_enable_stdio_uart(picoWeatherStation 1)

pico_add_extra_outputs(picoWeatherStation)
