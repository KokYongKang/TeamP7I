add_library(encoder
    encoder.c
)

target_include_directories(encoder PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
)

target_link_libraries(encoder
    pico_stdlib
    FreeRTOS-Kernel-Heap4
    hardware_gpio
    motor
)