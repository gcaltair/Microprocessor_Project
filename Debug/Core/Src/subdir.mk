################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MPU6500.c \
../Core/Src/astar.c \
../Core/Src/control_logic.c \
../Core/Src/dma.c \
../Core/Src/encoder.c \
../Core/Src/freertos.c \
../Core/Src/frontier.c \
../Core/Src/gpio.c \
../Core/Src/hc04.c \
../Core/Src/i2c.c \
../Core/Src/lidar.c \
../Core/Src/localization_task.c \
../Core/Src/main.c \
../Core/Src/mapping_task.c \
../Core/Src/motor.c \
../Core/Src/navigation_task.c \
../Core/Src/occupancy_grid.c \
../Core/Src/pid.c \
../Core/Src/scan_preprocess.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/MPU6500.o \
./Core/Src/astar.o \
./Core/Src/control_logic.o \
./Core/Src/dma.o \
./Core/Src/encoder.o \
./Core/Src/freertos.o \
./Core/Src/frontier.o \
./Core/Src/gpio.o \
./Core/Src/hc04.o \
./Core/Src/i2c.o \
./Core/Src/lidar.o \
./Core/Src/localization_task.o \
./Core/Src/main.o \
./Core/Src/mapping_task.o \
./Core/Src/motor.o \
./Core/Src/navigation_task.o \
./Core/Src/occupancy_grid.o \
./Core/Src/pid.o \
./Core/Src/scan_preprocess.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/MPU6500.d \
./Core/Src/astar.d \
./Core/Src/control_logic.d \
./Core/Src/dma.d \
./Core/Src/encoder.d \
./Core/Src/freertos.d \
./Core/Src/frontier.d \
./Core/Src/gpio.d \
./Core/Src/hc04.d \
./Core/Src/i2c.d \
./Core/Src/lidar.d \
./Core/Src/localization_task.d \
./Core/Src/main.d \
./Core/Src/mapping_task.d \
./Core/Src/motor.d \
./Core/Src/navigation_task.d \
./Core/Src/occupancy_grid.d \
./Core/Src/pid.d \
./Core/Src/scan_preprocess.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MPU6500.cyclo ./Core/Src/MPU6500.d ./Core/Src/MPU6500.o ./Core/Src/MPU6500.su ./Core/Src/astar.cyclo ./Core/Src/astar.d ./Core/Src/astar.o ./Core/Src/astar.su ./Core/Src/control_logic.cyclo ./Core/Src/control_logic.d ./Core/Src/control_logic.o ./Core/Src/control_logic.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/frontier.cyclo ./Core/Src/frontier.d ./Core/Src/frontier.o ./Core/Src/frontier.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/hc04.cyclo ./Core/Src/hc04.d ./Core/Src/hc04.o ./Core/Src/hc04.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/lidar.cyclo ./Core/Src/lidar.d ./Core/Src/lidar.o ./Core/Src/lidar.su ./Core/Src/localization_task.cyclo ./Core/Src/localization_task.d ./Core/Src/localization_task.o ./Core/Src/localization_task.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mapping_task.cyclo ./Core/Src/mapping_task.d ./Core/Src/mapping_task.o ./Core/Src/mapping_task.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/navigation_task.cyclo ./Core/Src/navigation_task.d ./Core/Src/navigation_task.o ./Core/Src/navigation_task.su ./Core/Src/occupancy_grid.cyclo ./Core/Src/occupancy_grid.d ./Core/Src/occupancy_grid.o ./Core/Src/occupancy_grid.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/scan_preprocess.cyclo ./Core/Src/scan_preprocess.d ./Core/Src/scan_preprocess.o ./Core/Src/scan_preprocess.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

