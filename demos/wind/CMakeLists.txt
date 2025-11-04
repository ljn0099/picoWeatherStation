add_executable(wind_demo
    main.c
)

target_link_libraries(wind_demo
    pico_stdlib
    hardware_gpio
    hardware_adc
)

pico_enable_stdio_usb(wind_demo 0)
pico_enable_stdio_uart(wind_demo 1)

pico_add_extra_outputs(wind_demo)
