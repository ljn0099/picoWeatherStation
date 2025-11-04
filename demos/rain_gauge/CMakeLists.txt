add_executable(rain_gauge_demo
    main.c
)

target_link_libraries(rain_gauge_demo
    pico_stdlib
    hardware_gpio
)

pico_enable_stdio_usb(rain_gauge_demo 0)
pico_enable_stdio_uart(rain_gauge_demo 1)

pico_add_extra_outputs(rain_gauge_demo)
