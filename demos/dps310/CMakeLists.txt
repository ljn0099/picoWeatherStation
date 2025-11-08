add_executable(dps310_demo
    main.c
)

target_link_libraries(dps310_demo
    pico_stdlib
    hardware_i2c
    sensor_dps310
)

pico_enable_stdio_usb(dps310_demo 0)
pico_enable_stdio_uart(dps310_demo 1)

pico_add_extra_outputs(dps310_demo)
