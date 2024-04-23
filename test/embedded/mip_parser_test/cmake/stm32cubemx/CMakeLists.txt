cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_FULL_LL_DRIVER 
	HSE_VALUE=8000000 
	HSE_STARTUP_TIMEOUT=100 
	LSE_STARTUP_TIMEOUT=5000 
	LSE_VALUE=32768 
	EXTERNAL_CLOCK_VALUE=12288000 
	HSI_VALUE=16000000 
	LSI_VALUE=32000 
	VDD_VALUE=3300 
	PREFETCH_ENABLE=0 
	ART_ACCELERATOR_ENABLE=0 
	STM32F767xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ~/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.1/Drivers/STM32F7xx_HAL_Driver/Inc
    ~/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.1/Drivers/CMSIS/Device/ST/STM32F7xx/Include
    ~/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.1/Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/stm32f7xx_it.c
    ~/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.1/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_gpio.c
    ~/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.1/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_rcc.c
    ~/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.1/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_utils.c
    ~/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.1/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_exti.c
    ../../Core/Src/system_stm32f7xx.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32f767xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


