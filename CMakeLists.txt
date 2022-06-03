cmake_minimum_required(VERSION 3.17)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_SYSTEM_VERSION 1)

# Enable logging messages
set(CMAKE_VERBOSE_MAKEFILE ON)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
#set(CMAKE_AR arm-none-eabi-ar)
#set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
#set(CMAKE_OBJDUMP arm-none-eabi-objdump)
#set(SIZE arm-none-eabi-size)
#set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(LEV_REF C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if (NOT DEFINED ${LEV_FIRMWARE_DIR})
    set(LEV_FIRMWARE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
endif()

#SET(CMAKE_C_COMPILER    ${TOOLCHAIN_PATH}arm-none-eabi-gcc)
#SET(CMAKE_CXX_COMPILER  ${TOOLCHAIN_PATH}arm-none-eabi-g++)
#set(CMAKE_ASM_COMPILER  ${TOOLCHAIN_PATH}arm-none-eabi-gcc)

add_compile_options(
        -mcpu=cortex-m3
        -mthumb
        -mthumb-interwork
        -fdata-sections
        -ffunction-sections
        -Wall
        $<$<CONFIG:Debug>:-Og>
        )

set(SOURCE
        ${LEV_FIRMWARE_DIR}/TMCL.c
        ${LEV_FIRMWARE_DIR}/BLDC.c
        ${LEV_FIRMWARE_DIR}/Sensor.c
        ${LEV_FIRMWARE_DIR}/Button.c
        ${LEV_FIRMWARE_DIR}/Led.c
        ${LEV_FIRMWARE_DIR}/LedRear.c
        ${LEV_FIRMWARE_DIR}/main.c
        ${LEV_FIRMWARE_DIR}/hal/comm/CAN.c
        ${LEV_FIRMWARE_DIR}/hal/comm/SPI.c
        ${LEV_FIRMWARE_DIR}/hal/comm/UART.c
        ${LEV_FIRMWARE_DIR}/hal/comm/Eeprom.c
        ${LEV_FIRMWARE_DIR}/hal/ic/LIS2DH12.c
        ${LEV_FIRMWARE_DIR}/hal/system/Cpu.c
        ${LEV_FIRMWARE_DIR}/hal/system/IO.c
        ${LEV_FIRMWARE_DIR}/hal/system/SystemInfo.c
        ${LEV_FIRMWARE_DIR}/hal/system/Systick.c
        ${LEV_FIRMWARE_DIR}/hal/Flags.c
        ${LEV_FIRMWARE_DIR}/hal/Debug.c
        ${LEV_FIRMWARE_DIR}/TMC-API/tmc/helpers/Functions.c
        ${LEV_FIRMWARE_DIR}/TMC-API/tmc/ramp/LinearRamp.c
        ${LEV_FIRMWARE_DIR}/TMC-API/tmc/ic/TMC4671/TMC4671.c
        ${LEV_FIRMWARE_DIR}/TMC-API/tmc/ic/TMC6200/TMC6200.c
        ${LEV_FIRMWARE_DIR}/hal/modules/TMC4671-LEV-REF_v1.0.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_bkp.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_can.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_flash.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_exti.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_adc.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_dma.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_gpio.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_nvic.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_pwr.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_rcc.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_tim.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_usart.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_spi.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_i2c.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_systick.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_it.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32f10x_vector.c
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103/cortexm3_macro.s
        )

add_definitions(-DSTM32F10X_MD
                -DVECT_TAB_ROM
                -DROM_RUN
        `       -DSTM32F103xB
                -DBOOTLOADER)



set(EXECUTABLE ${PROJECT_NAME}.elf)

add_executable(${EXECUTABLE} ${SOURCE})

include_directories(
        ${LEV_FIRMWARE_DIR}
        ${LEV_FIRMWARE_DIR}/cpu/STM32F103
        ${LEV_FIRMWARE_DIR}/hal/comm
        ${LEV_FIRMWARE_DIR}/hal/ic
        ${LEV_FIRMWARE_DIR}/hal/modules
        ${LEV_FIRMWARE_DIR}/hal/system
        ${LEV_FIRMWARE_DIR}/hal/tmcl
        ${LEV_FIRMWARE_DIR}/hal/FT4222H
        ${LEV_FIRMWARE_DIR}/TMC-API/tmc/ic/TMC4671
        ${LEV_FIRMWARE_DIR}/TMC-API/tmc/ic/TMC6200
        ${LEV_FIRMWARE_DIR}/TMC-API/tmc/ramp
        ${LEV_FIRMWARE_DIR}/TMC-API/tmc/helpers
        ${LEV_FIRMWARE_DIR}/TMC-API
        )



set(LINKER_SCRIPT ${LEV_FIRMWARE_DIR}/cpu/STM32F103/stm32-tmcm.ld)
add_link_options(-T ${LINKER_SCRIPT})
add_link_options(
        -Wl,--gc-sections,-Map=${PROJECT_NAME}.map,-cref
        -u,Reset_Handler
        --specs=nosys.specs --specs=nano.specs)
add_link_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)

target_link_libraries(${EXECUTABLE}  PRIVATE c m gcc)




# Print executable size
#add_custom_command(TARGET ${EXECUTABLE}
#        POST_BUILD
#        COMMAND arm-none-eabi-size ${EXECUTABLE})
#
## Create hex file
#add_custom_command(TARGET ${EXECUTABLE}
#        POST_BUILD
#        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
#        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)

