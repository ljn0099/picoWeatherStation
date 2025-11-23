# Helper function to require variables
function(require_var VAR_NAME ERROR_MESSAGE)
    if(NOT DEFINED ${VAR_NAME} OR ${VAR_NAME} STREQUAL "")
        message(FATAL_ERROR "${ERROR_MESSAGE}")
    endif()
endfunction()

# Required WiFi configuration
require_var(WIFI_SSID     "You must define WIFI_SSID (e.g. -DWIFI_SSID=\"YourWiFiName\")")
require_var(WIFI_PASSWORD "You must define WIFI_PASSWORD (e.g. -DWIFI_PASSWORD=\"YourWiFiPassword\")")

# MQTT configuration (supports environment fallback)
foreach(VAR MQTT_SERVER MQTT_USERNAME MQTT_PASSWORD)
    if(NOT DEFINED ${VAR} AND DEFINED ENV{${VAR}})
        set(${VAR} $ENV{${VAR}})
        message(STATUS "Using ${VAR} from environment: '${${VAR}}'")
    endif()
    require_var(${VAR} "Error: ${VAR} is not defined")
endforeach()

# Certificate configuration
if(NOT MQTT_CERT_PATH)
    set(MQTT_CERT_PATH "${CMAKE_CURRENT_LIST_DIR}/certs/${MQTT_SERVER}")
endif()

if(NOT MQTT_CERT_INC)
    set(MQTT_CERT_INC "mqtt_client.inc")
endif()

set(MQTT_CERT_FULL_PATH "${MQTT_CERT_PATH}/${MQTT_CERT_INC}")

if(NOT EXISTS "${MQTT_CERT_FULL_PATH}")
    message(FATAL_ERROR
        "Missing MQTT certificate include file:\n"
        "  ${MQTT_CERT_FULL_PATH}\n"
        "Create the certificate file or specify MQTT_CERT_PATH/MQTT_CERT_INC."
    )
endif()

# Nanopb protobuf generation
set(NANOPB_DIR "${CMAKE_SOURCE_DIR}/nanopb")
set(NANOPB_PLUGIN "${NANOPB_DIR}/generator/protoc-gen-nanopb")

set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${GENERATED_DIR}")

set(WEATHER_PB_C  "${GENERATED_DIR}/weather.pb.c")
set(WEATHER_PB_H  "${GENERATED_DIR}/weather.pb.h")
set(WRAPPERS_PB_C "${GENERATED_DIR}/google/protobuf/wrappers.pb.c")
set(WRAPPERS_PB_H "${GENERATED_DIR}/google/protobuf/wrappers.pb.h")

add_custom_command(
    OUTPUT ${WEATHER_PB_C} ${WEATHER_PB_H} ${WRAPPERS_PB_C} ${WRAPPERS_PB_H}
    COMMAND protoc
        --plugin=protoc-gen-nanopb=${NANOPB_PLUGIN}
        --nanopb_out=${GENERATED_DIR}
        -I${CMAKE_CURRENT_SOURCE_DIR}
        -I${NANOPB_DIR}/pb
        ${CMAKE_CURRENT_SOURCE_DIR}/weather.proto
        google/protobuf/wrappers.proto
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/weather.proto
    COMMENT "Generating nanopb sources from .proto files"
)

add_custom_target(generate_weather_pb
    DEPENDS ${WEATHER_PB_C} ${WEATHER_PB_H} ${WRAPPERS_PB_C} ${WRAPPERS_PB_H}
)

# Target: picoWeatherStation
add_executable(picoWeatherStation
    main.c
    queues.c
    ntp.c
    mqtt.c
    weather_protocol.c
    ${WEATHER_PB_C}
    ${WRAPPERS_PB_C}
    ${NANOPB_DIR}/pb_common.c
    ${NANOPB_DIR}/pb_encode.c
    ${NANOPB_DIR}/pb_decode.c
)

add_dependencies(picoWeatherStation generate_weather_pb)

# Compile definitions and include paths
target_compile_definitions(picoWeatherStation PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    MQTT_USERNAME=\"${MQTT_USERNAME}\"
    MQTT_SERVER=\"${MQTT_SERVER}\"
    MQTT_PASSWORD=\"${MQTT_PASSWORD}\"
    MQTT_CERT_INC=\"${MQTT_CERT_INC}\"
    ALTCP_MBEDTLS_AUTHMODE=MBEDTLS_SSL_VERIFY_REQUIRED
)

target_include_directories(picoWeatherStation PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${GENERATED_DIR}
    ${NANOPB_DIR}
    ${MQTT_CERT_PATH}
)

# Libraries
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
    pico_lwip_mqtt
    pico_mbedtls
    pico_lwip_mbedtls
)

# Pico platform settings
pico_enable_stdio_usb(picoWeatherStation 0)
pico_enable_stdio_uart(picoWeatherStation 1)

pico_add_extra_outputs(picoWeatherStation)
