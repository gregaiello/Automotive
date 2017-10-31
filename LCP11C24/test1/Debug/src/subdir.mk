################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/cr_startup_lpc11xx.c \
../src/crp.c \
../src/sysinit.c \
../src/test1.c 

OBJS += \
./src/cr_startup_lpc11xx.o \
./src/crp.o \
./src/sysinit.o \
./src/test1.o 

C_DEPS += \
./src/cr_startup_lpc11xx.d \
./src/crp.d \
./src/sysinit.d \
./src/test1.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -DNO_BOARD_LIB -D__CODE_RED -DCORE_M0 -D__USE_LPCOPEN -D__LPC11XX__ -D__REDLIB__ -I"/home/gregorio/Documents/MCUXpresso/lpc_chip_11cxx_lib/inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


