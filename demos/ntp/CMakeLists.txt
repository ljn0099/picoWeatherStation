add_executable(ntp_demo
    main.c
    )
target_link_libraries(ntp_demo
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_aon_timer
    )
target_include_directories(ntp_demo PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    )
target_compile_definitions(ntp_demo PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    )

pico_add_extra_outputs(ntp_demo)
