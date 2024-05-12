################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Display/display.c \
../Core/Display/fonts.c \
../Core/Display/ili9341.c \
../Core/Display/st7789.c 

OBJS += \
./Core/Display/display.o \
./Core/Display/fonts.o \
./Core/Display/ili9341.o \
./Core/Display/st7789.o 

C_DEPS += \
./Core/Display/display.d \
./Core/Display/fonts.d \
./Core/Display/ili9341.d \
./Core/Display/st7789.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Display/%.o Core/Display/%.su: ../Core/Display/%.c Core/Display/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/STM32/PROJECTS/PROJECTS/LCD_ST7789_SPI_DMA_LL/Core/Display" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Display

clean-Core-2f-Display:
	-$(RM) ./Core/Display/display.d ./Core/Display/display.o ./Core/Display/display.su ./Core/Display/fonts.d ./Core/Display/fonts.o ./Core/Display/fonts.su ./Core/Display/ili9341.d ./Core/Display/ili9341.o ./Core/Display/ili9341.su ./Core/Display/st7789.d ./Core/Display/st7789.o ./Core/Display/st7789.su

.PHONY: clean-Core-2f-Display

