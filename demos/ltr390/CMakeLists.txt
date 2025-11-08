add_executable(ltr390_demo
    main.c
)

target_link_libraries(ltr390_demo
    pico_stdlib
    hardware_i2c
    sensor_ltr390
)

pico_enable_stdio_usb(ltr390_demo 0)
pico_enable_stdio_uart(ltr390_demo 1)

pico_add_extra_outputs(ltr390_demo)
