set(STM_BOARD STM32F3-Discovery)
set(STM32_CHIP STM32F303VC)
set(STM32Cube_DIR /home/scohen/STM32Cube/Repository/STM32Cube_FW_F3_V1.10.0)

set(CMAKE_MODULE_PATH /home/scohen/repos/stm32-cmake/cmake)
set(CMAKE_TOOLCHAIN_FILE /home/scohen/repos/stm32-cmake/cmake/gcc_stm32.cmake)
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_BUILD_TYPE Debug)
set(STM32_LINKER_SCRIPT STM32F303VCTx_FLASH.ld)

cmake_minimum_required(VERSION 3.5)
enable_language(ASM)

project(FlightController)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs --specs=nosys.specs")

find_package(CMSIS REQUIRED)
find_package(STM32HAL COMPONENTS gpio i2c spi pcd rcc REQUIRED)
find_package(USBDevice COMPONENTS CDC REQUIRED)
find_package(STM32BSP COMPONENTS accelerometer gyroscope lsm303dlhc l3gd20 REQUIRED)  
  
include_directories(
  ${CMSIS_INCLUDE_DIRS}
  ${STM32HAL_INCLUDE_DIR}
  ${USBDevice_INCLUDE_DIR}
  ${STM32BSP_INCLUDE_DIR}
  Inc
  )
 
set(PROJECT_SOURCES
  Src/main.cpp
  Src/imu.cpp
  Src/stm32f3xx_hal_msp.c
  Src/stm32f3xx_it.c
  Src/usbd_cdc_if.c
  Src/usbd_conf.c
  Src/usbd_desc.c
  Src/usb_device.c
  )

add_executable(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCES} ${CMSIS_SOURCES} ${STM32HAL_SOURCES} ${USBDevice_SOURCES} ${STM32BSP_SOURCES})

stm32_set_target_properties(${CMAKE_PROJECT_NAME})
stm32_add_hex_bin_targets(${CMAKE_PROJECT_NAME} ALL)
stm32_print_size_of_targets(${CMAKE_PROJECT_NAME}) 

message("FLASH origin ${STM32_FLASH_ORIGIN}")

add_custom_target(flash
  COMMAND st-flash --reset write ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_PROJECT_NAME}.bin 0x08000000
  DEPENDS ${CMAKE_PROJECT_NAME}
  )
