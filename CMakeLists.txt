cmake_minimum_required(VERSION 3.26)
set(PIMORONI_PICO_PATH $ENV{PIMORONI_PICO_PATH})

include(pimoroni_pico_import.cmake)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(osod_motor_2040 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(osod_motor_2040
        src/main.cpp
)
add_subdirectory(${PIMORONI_PICO_PATH} ${CMAKE_BINARY_DIR}/pimoroni-pico-build)

add_subdirectory(libs)

target_link_libraries(${PROJECT_NAME}
        motor2040
        receiver
        navigator
        state_estimator
        statemanager
        stoker
        tf_luna
        config
        common
        usb_serial
        serialtransfer
        communicator
        balance_port
        bno080
        waypoint_navigation
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
        PICO_DEFAULT_UART=1
        PICO_DEFAULT_UART_TX_PIN=20
        PICO_DEFAULT_UART_RX_PIN=21
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Create UF2 file from the binary
pico_add_extra_outputs(${PROJECT_NAME})

# Set up files for the release packages
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
        ${CMAKE_CURRENT_LIST_DIR}/README.md
        DESTINATION .
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)