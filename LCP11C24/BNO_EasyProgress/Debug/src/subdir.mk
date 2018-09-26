################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/BNO055.c \
../src/BNO_CAN.c \
../src/SSD1306.c \
../src/cr_startup_lpc11xx.c \
../src/crp.c \
../src/fonts.c \
../src/sysinit.c 

OBJS += \
./src/BNO055.o \
./src/BNO_CAN.o \
./src/SSD1306.o \
./src/cr_startup_lpc11xx.o \
./src/crp.o \
./src/fonts.o \
./src/sysinit.o 

C_DEPS += \
./src/BNO055.d \
./src/BNO_CAN.d \
./src/SSD1306.d \
./src/cr_startup_lpc11xx.d \
./src/crp.d \
./src/fonts.d \
./src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M0 -D__USE_LPCOPEN -DNO_BOARD_LIB -D__LPC11XX__ -D__REDLIB__ -I"/home/greg/Documents/CLIMER/Firmware/lpc_chip_11cxx_lib/inc" -I"/home/greg/Documents/CLIMER/Automotive/LCP11C24/BNO_EasyProgress/inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


