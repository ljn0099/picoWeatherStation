cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
project(pico_station C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory("demos")
# add_subdirectory("src")

# target_include_directories(PRIVATE ${CMAKE_CURRENT_LIST_DIR} )
# target_link_libraries(pico_cyw43_arch_lwip_threadsafe_background pico_stdlib)
