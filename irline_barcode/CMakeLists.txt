add_executable(irline_barcode
    barcode.c
)

target_include_directories(irline_barcode PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/include
    ${PICO_SDK_PATH}/hardware
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
)

target_link_libraries(irline_barcode
    pico_stdlib
    FreeRTOS-Kernel-Heap4
    hardware_adc
    hardware_gpio
    hardware_timer
    hardware_pwm
)