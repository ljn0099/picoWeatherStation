add_executable(anemometer_demo
    main.c
)

target_link_libraries(anemometer_demo
    pico_stdlib
    hardware_gpio
)

pico_enable_stdio_usb(anemometer_demo 0)
pico_enable_stdio_uart(anemometer_demo 1)

pico_add_extra_outputs(anemometer_demo)
