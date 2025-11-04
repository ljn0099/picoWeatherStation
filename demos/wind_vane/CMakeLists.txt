add_executable(wind_vane_demo
    main.c
)

target_link_libraries(wind_vane_demo
    pico_stdlib
    hardware_gpio
    hardware_adc
)

pico_enable_stdio_usb(wind_vane_demo 0)
pico_enable_stdio_uart(wind_vane_demo 1)

pico_add_extra_outputs(wind_vane_demo)
