if(NOT DEFINED WIFI_SSID OR WIFI_SSID STREQUAL "")
    message(FATAL_ERROR "You must define WIFI_SSID (e.g. -DWIFI_SSID=\"YourWiFiName\")")
endif()

if(NOT DEFINED WIFI_PASSWORD OR WIFI_PASSWORD STREQUAL "")
    message(FATAL_ERROR "You must define WIFI_PASSWORD (e.g. -DWIFI_PASSWORD=\"YourWiFiPassword\")")
endif()

set(NANOPB_PLUGIN ${CMAKE_SOURCE_DIR}/nanopb/generator/protoc-gen-nanopb)

set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)

file(MAKE_DIRECTORY ${GENERATED_DIR})

set(WEATHER_PB_C ${GENERATED_DIR}/weather.pb.c)
set(WEATHER_PB_H ${GENERATED_DIR}/weather.pb.h)
set(WRAPPERS_PB_C ${GENERATED_DIR}/google/protobuf/wrappers.pb.c)
set(WRAPPERS_PB_H ${GENERATED_DIR}/google/protobuf/wrappers.pb.h)

add_custom_command(
    OUTPUT ${WEATHER_PB_C} ${WEATHER_PB_H} ${WRAPPERS_PB_C} ${WRAPPERS_PB_H}
    COMMAND protoc
            --plugin=protoc-gen-nanopb=${NANOPB_PLUGIN}
            --nanopb_out=${GENERATED_DIR}
            -I${CMAKE_CURRENT_SOURCE_DIR}
            -I${CMAKE_SOURCE_DIR}/nanopb/pb
            ${CMAKE_CURRENT_SOURCE_DIR}/weather.proto
            google/protobuf/wrappers.proto
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/weather.proto
    COMMENT "Generating C code from weather.proto with nanopb"
)

add_custom_target(generate_weather_pb
    DEPENDS ${WEATHER_PB_C} ${WEATHER_PB_H} ${WRAPPERS_PB_C} ${WRAPPERS_PB_H}
)

set(NANOPB_DIR "${CMAKE_SOURCE_DIR}/nanopb")

add_executable(picoWeatherStation
    main.c
    queues.c
    ntp.c
    weather_protocol.c
    ${WEATHER_PB_C}
    ${WRAPPERS_PB_C}
    ${NANOPB_DIR}/pb_common.c
    ${NANOPB_DIR}/pb_encode.c
    ${NANOPB_DIR}/pb_decode.c
)

add_dependencies(picoWeatherStation generate_weather_pb)

target_compile_definitions(picoWeatherStation PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(picoWeatherStation PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${GENERATED_DIR}
    ${NANOPB_DIR}
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
