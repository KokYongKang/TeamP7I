add_library(ultrasonic
    ultrasonic.c
)

target_include_directories(ultrasonic PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
)

target_link_libraries(ultrasonic
    pico_stdlib
    FreeRTOS-Kernel-Heap4
    hardware_gpio
)