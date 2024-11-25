add_library(
        dashboard_sender dashboard_sender.c
)

target_include_directories(dashboard_sender PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${FREERTOS_KERNEL_PATH}/include
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
        ${CMAKE_CURRENT_LIST_DIR}/../..
)

target_include_directories(dashboard_sender PRIVATE
        ../ # For common lwipopts.h
)


# pull in common dependencies and additional i2c hardware support
target_link_libraries(
        dashboard_sender
        pico_stdlib pico_lwip
        pico_cyw43_arch_lwip_threadsafe_background
        pico_async_context_freertos
        FreeRTOS-Kernel-Heap4
)
