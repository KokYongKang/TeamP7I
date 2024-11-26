function(setup_proj_exec program file_name)
    #region Client
    add_executable(${program} ${file_name})

    target_include_directories(${program} PRIVATE
#            ../../freertos
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required

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
            FreeRTOS-Kernel-Heap4
    )

    # create map/bin/hex file etc.
    pico_add_extra_outputs(${program})

    pico_enable_stdio_usb(${program} 1)
endfunction()

#region Client
setup_proj_exec(remote_control
        acc_client.c
)
#endregion
