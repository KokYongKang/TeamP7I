set(WIFI_SSID $ENV{WIFI_SSID})
set(WIFI_PASSWORD $ENV{WIFI_PASSWORD})

function(setup_proj_exec program file_name)
    #region Client
    add_executable(${program} acc_server.c)

    target_include_directories(${program} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${PICO_SDK_PATH}/include
            ${PICO_SDK_PATH}/hardware
            ${FREERTOS_KERNEL_PATH}/include
            ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
            ${CMAKE_SOURCE_DIR}/motor       # Add this to find motor.h
            ${CMAKE_SOURCE_DIR}/encoder     # Add this to find encoder.h
            ${CMAKE_SOURCE_DIR}/ultrasonic     # Add this to find ultrasonic.h
        #     ../../freertos                  
            ${CMAKE_CURRENT_LIST_DIR}/..    # for our common lwipopts or any other standard includes, if required
    )

    target_compile_definitions(${program} PRIVATE
            WIFI_SSID="${WIFI_SSID}"
            WIFI_PASSWORD="${WIFI_PASSWORD}"
    )
    # pull in common dependencies and additional i2c hardware support
    target_link_libraries(
            ${program} PRIVATE
            pico_stdlib hardware_i2c pico_lwip
            pico_cyw43_arch_lwip_threadsafe_background
            pico_async_context_freertos
            FreeRTOS-Kernel-Heap4 motor encoder ultrasonic
    )

    # create map/bin/hex file etc.
    pico_add_extra_outputs(${program})

    pico_enable_stdio_usb(${program} 1)
    pico_enable_stdio_uart(${program} 0)  
endfunction()

#region Client
setup_proj_exec(acc_client
        acc_client.c
)
#endregion

#region Server
setup_proj_exec(acc_server
        acc_server.c
)
#endregion
