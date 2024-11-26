cmake_minimum_required(VERSION 3.12)

# Name project
SET(ProjectName Team-P7I)

# Set any variables required for importing libraries
SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

# Import those libraries
include(pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Define project
project(${ProjectName})

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

set(WIFI_SSID $ENV{WIFI_SSID})
set(WIFI_PASSWORD $ENV{WIFI_PASSWORD})

# Add subdirectories
# add_subdirectory(irline_barcode)
# add_subdirectory (irline_detector)
add_subdirectory(motor)
add_subdirectory(encoder)
add_subdirectory(ultrasonic)
add_subdirectory(car)
add_subdirectory(dashboard)
add_subdirectory(remote_control)

# if you have anything in "lib" folder then uncomment below - remember to add a CMakeLists.txt
# file to the "lib" directory
#add_subdirectory(lib)