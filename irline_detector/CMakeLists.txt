add_executable(${ProjectName}
    detector.c
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/include    
    ${PICO_SDK_PATH}/hardware   
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
    ${CMAKE_SOURCE_DIR}/motor  # Add this to find motor.h
    ${CMAKE_SOURCE_DIR}/encoder  # Add this to find motor.h
    ${CMAKE_CURRENT_LIST_DIR}/..
) 

target_link_libraries(${ProjectName} 
motor
encoder
FreeRTOS-Kernel-Heap4
pico_stdlib
hardware_pwm
hardware_adc
hardware_gpio
)

pico_add_extra_outputs(${ProjectName})

# Enable USB output and disable UART output
pico_enable_stdio_usb(${ProjectName} 1)  
pico_enable_stdio_uart(${ProjectName} 0)  