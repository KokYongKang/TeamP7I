add_executable(
        dashboard_receiver dashboard_receiver.c
)

target_include_directories(dashboard_receiver PRIVATE
        ..
        ${PICO_SDK_PATH}/include
        ${PICO_SDK_PATH}/hardware
        ${FREERTOS_KERNEL_PATH}/include
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
        ${CMAKE_SOURCE_DIR}/motor       # Add this to find motor.h
        ${CMAKE_SOURCE_DIR}/encoder     # Add this to find encoder.h
        ${CMAKE_SOURCE_DIR}/ultrasonic     # Add this to find ultrasonic.h
        #     ../../freertos
        ../../..    # for our common lwipopts or any other standard includes, if required
)

target_compile_definitions(dashboard_receiver PRIVATE
        WIFI_SSID="${WIFI_SSID}"
        WIFI_PASSWORD="${WIFI_PASSWORD}"
)


# pull in common dependencies and additional i2c hardware support
target_link_libraries(
        dashboard_receiver PRIVATE
        pico_stdlib hardware_i2c pico_lwip
        pico_cyw43_arch_lwip_threadsafe_background
        pico_async_context_freertos
        FreeRTOS-Kernel-Heap4 motor encoder ultrasonic
)

# create map/bin/hex file etc.
pico_add_extra_outputs(dashboard_receiver)

pico_enable_stdio_usb(dashboard_receiver 1)
pico_enable_stdio_uart(dashboard_receiver 1)