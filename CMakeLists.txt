cmake_minimum_required(VERSION 3.21)
project(SensorsMotherboard C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(LD_PREPROCESS_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/config")
set(CMAKE_C_STANDARD 11)
set(WCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/wch_low_level_drivers")
set(MCU_TYPE CH32V303CB)
include("${WCH_DIR}/mcu_config.cmake")
set(WCH_COMPILE_OPTIONS ${WCH_ARCH_FLAGS} -Os -g3 -ffunction-sections -fdata-sections -fno-common)
include_directories(config src vendor/wch_usbfs)
add_subdirectory(lib/wch_low_level_drivers)
add_executable(firmware
  ${WCH_ASM_STARTUP}
  src/main.c src/board.c src/transport.c src/i2c_bus.c src/sensors.cpp src/config_store.c
  src/protocol.c src/app.cpp
  vendor/wch_usbfs/ch32v30x_usbfs_device.c vendor/wch_usbfs/usb_desc.c)
target_compile_options(firmware PRIVATE ${WCH_COMPILE_OPTIONS} -Wall -Wextra
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics>)
target_link_libraries(firmware PRIVATE -Wl,--start-group Core Peripheral RCC Delay CRC Flash -Wl,--end-group)
target_link_options(firmware PRIVATE ${WCH_ARCH_FLAGS} -nostartfiles --specs=nano.specs --specs=nosys.specs
  "-T${LINKER_SCRIPT}" "-L${CMAKE_BINARY_DIR}" -Wl,--gc-sections -Wl,--print-memory-usage
  "-Wl,-Map=${CMAKE_BINARY_DIR}/firmware.map"
  "-Wl,--defsym=__FLASH_LIMIT=${FLASH_LIMIT}"
  "-Wl,--defsym=__SLOWFLASH_ORIGIN=${SLOWFLASH_ORIGIN}"
  "-Wl,--defsym=__SLOWFLASH_LENGTH=${SLOWFLASH_SIZE}")
set_target_properties(firmware PROPERTIES SUFFIX .elf)
add_custom_command(TARGET firmware POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:firmware> firmware.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:firmware> firmware.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:firmware>)

add_custom_target(flash
  COMMAND "${WK_SDK_PATH}/WCH/OpenOCD/bin/openocd.exe"
    -f "${WK_SDK_PATH}/WCH/OpenOCD/bin/wch-riscv.cfg"
    -c "program firmware.elf verify reset exit"
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  DEPENDS firmware
  COMMENT "Flashing SensorsMotherboard via WCH-Link (verify and reset)"
  USES_TERMINAL VERBATIM)
