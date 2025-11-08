add_executable(hdc3022_demo
    main.c
)

target_link_libraries(hdc3022_demo
    pico_stdlib
    hardware_i2c
    sensor_hdc3022
)

pico_enable_stdio_usb(hdc3022_demo 0)
pico_enable_stdio_uart(hdc3022_demo 1)

pico_add_extra_outputs(hdc3022_demo)
