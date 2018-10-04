PROJECT_NAME=Firmware_Simedis_BLDC_BAREMETAL

BUILDDIR = build

CORE = CMSIS
DISCOVERY = inc
HAL = Drivers/STM32F4xx_HAL_Driver
USB = Middlewares/ST/STM32_USB_Device_Library
OLED = SSD1306_OLED

SOURCES += startup_stm32f4xx.s
SOURCES += system_stm32f4xx.c 

SOURCES += \
		src/main.c \
		$(HAL)/Src/stm32f4xx_hal.c \
		$(HAL)/Src/stm32f4xx_hal_cortex.c \
		$(HAL)/Src/stm32f4xx_hal_dma.c \
		$(HAL)/Src/stm32f4xx_hal_dma_ex.c \
		$(HAL)/Src/stm32f4xx_hal_flash.c \
		$(HAL)/Src/stm32f4xx_hal_flash_ex.c \
		$(HAL)/Src/stm32f4xx_hal_flash_ramfunc.c \
		$(HAL)/Src/stm32f4xx_hal_gpio.c \
		$(HAL)/Src/stm32f4xx_hal_i2c.c \
		$(HAL)/Src/stm32f4xx_hal_i2c_ex.c \
		$(HAL)/Src/stm32f4xx_hal_pcd.c \
		$(HAL)/Src/stm32f4xx_hal_pcd_ex.c \
		$(HAL)/Src/stm32f4xx_hal_pwr.c \
		$(HAL)/Src/stm32f4xx_hal_pwr_ex.c \
		$(HAL)/Src/stm32f4xx_hal_rcc.c \
		$(HAL)/Src/stm32f4xx_hal_rcc_ex.c \
		$(HAL)/Src/stm32f4xx_hal_tim.c \
		$(HAL)/Src/stm32f4xx_hal_tim_ex.c \
		$(HAL)/Src/stm32f4xx_hal_uart.c \
		$(HAL)/Src/stm32f4xx_ll_usb.c \
		src/stm32f4xx_hal_msp.c \
		src/syscalls.c \
		src/biss.c \
		src/mhl200_driver.c \
		src/peripherals_STM32F405.c \
		src/stm32f4xx_it.c \
		$(USB)/Class/CDC/Src/usbd_cdc.c \
		$(USB)/Core/Src/usbd_core.c \
		$(USB)/Core/Src/usbd_ctlreq.c \
		$(USB)/Core/Src/usbd_ioreq.c \
		src/usb_device.c \
		src/usbd_cdc_if.c \
		src/usbd_conf.c \
		src/usbd_desc.c \
		src/jump_to_bootloader.c \
		src/calibration_hall_sensors.c \
		src/flash.c \
		src/comm.c \
		src/ssd1306_HAL.c \
		$(OLED)/fonts.c \
		$(OLED)/ssd1306_oled.c \

OBJECTS = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(SOURCES))))

INCLUDES += -I$(CORE)/Include \
			-I$(CORE)/Device/ST/STM32F4xx/Include \
			-I$(DISCOVERY) \
			-I$(HAL)/Inc \
			-I$(USB)/Class/CDC/Inc \
			-I$(USB)/Core/Inc \
			-I$(OLED) \
			-I.

ELF = $(BUILDDIR)/$(PROJECT_NAME).elf
HEX = $(BUILDDIR)/$(PROJECT_NAME).hex
BIN = $(BUILDDIR)/$(PROJECT_NAME).bin

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
AR = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
GDB = arm-none-eabi-gdb

CFLAGS  = -g -O0 -Wall -I.\
   -mcpu=cortex-m4 $(COMMON_FLAG) -mthumb \
   $(INCLUDES) -DSTM32F405xx


COMMON_FLAG= -mfpu=fpv4-sp-d16 -mfloat-abi=hard

LDSCRIPT = STM32F405RGTx_FLASH.ld
LDFLAGS += -T$(LDSCRIPT) $(COMMON_FLAG) -mthumb -lm -lc -mcpu=cortex-m4 -nostdlib --specs=nano.specs

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(ELF): $(OBJECTS)
	$(LD) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)

$(BUILDDIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.s
	mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

flash: $(BIN)
	st-flash write $(BIN) 0x8000000

debug: $(ELF)
	$(GDB) -tui $(ELF)

all: $(HEX) $(BIN)

.PHONY: clean
	
clean:
	rm -rf build