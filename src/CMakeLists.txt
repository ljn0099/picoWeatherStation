add_executable(picoWeatherStation
    main.c
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
)

pico_enable_stdio_usb(picoWeatherStation 0)
pico_enable_stdio_uart(picoWeatherStation 1)

pico_add_extra_outputs(picoWeatherStation)
