add_executable(${ProjectName}
    barcode.c
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/include    
    ${PICO_SDK_PATH}/hardware   
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
) 

target_link_libraries(${ProjectName} 
    pico_stdlib 
    FreeRTOS-Kernel-Heap4 
    hardware_adc  
    hardware_gpio  
    hardware_timer 
    hardware_pwm
)

pico_add_extra_outputs(${ProjectName})

# Enable USB output and disable UART output
pico_enable_stdio_usb(${ProjectName} 1)  
pico_enable_stdio_uart(${ProjectName} 0)  