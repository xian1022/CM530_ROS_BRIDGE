#******************************************************************************
# Makefile for CM-530 Embedded C project
# Project : 12 CM530_BRIDGE_MODE
# Toolchain : WinARM / arm-eabi-gcc
#******************************************************************************

TARGET  = CM530

# Toolchain
CC      = arm-eabi-gcc
AR      = arm-eabi-ar
OBJCOPY = arm-eabi-objcopy
OBJDUMP = arm-eabi-objdump

# Compile flags
CFLAGS  = -mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common $(EXTRA_CFLAGS)
AFLAGS  = -mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common
INCLUDES = -I. -Istm32f10x_lib/inc -IHW/inc -IAPP/inc

# Link flags
LDSCRIPT = stm32.ld
LDFLAGS  = -Wl,--gc-sections,-Map=$(TARGET).elf.map,-cref,-u,Reset_Handler -Lstm32f10x_lib -T $(LDSCRIPT)

# Application object files
APP_OBJS = \
	APP/src/main.o \
	APP/src/stm32f10x_it.o \
	APP/src/dxl_hal.o \
	APP/src/dynamixel.o \
	APP/src/syscalls.o

# STM32 library object files
LIB_OBJS = \
	stm32f10x_lib/src/stm32f10x_adc.o \
	stm32f10x_lib/src/stm32f10x_bkp.o \
	stm32f10x_lib/src/stm32f10x_exti.o \
	stm32f10x_lib/src/stm32f10x_flash.o \
	stm32f10x_lib/src/stm32f10x_gpio.o \
	stm32f10x_lib/src/stm32f10x_lib.o \
	stm32f10x_lib/src/stm32f10x_nvic.o \
	stm32f10x_lib/src/stm32f10x_pwr.o \
	stm32f10x_lib/src/stm32f10x_rcc.o \
	stm32f10x_lib/src/stm32f10x_systick.o \
	stm32f10x_lib/src/stm32f10x_tim.o \
	stm32f10x_lib/src/stm32f10x_tim1.o \
	stm32f10x_lib/src/stm32f10x_usart.o \
	stm32f10x_lib/src/cortexm3_macro.o \
	stm32f10x_lib/src/stm32f10x_can.o \
	stm32f10x_lib/src/stm32f10x_dma.o \
	stm32f10x_lib/src/stm32f10x_i2c.o \
	stm32f10x_lib/src/stm32f10x_iwdg.o \
	stm32f10x_lib/src/stm32f10x_rtc.o \
	stm32f10x_lib/src/stm32f10x_spi.o \
	stm32f10x_lib/src/stm32f10x_wwdg.o \
	stm32f10x_lib/src/stm32f10x_vector.o

LIBSTM32 = stm32f10x_lib/libstm32.a

.PHONY: all clean

all: $(TARGET).elf $(TARGET).hex $(TARGET).bin $(TARGET).lss

#------------------------------------------------------------------------------
# Link final ELF
#------------------------------------------------------------------------------
$(TARGET).elf: $(APP_OBJS) $(LIBSTM32)
	$(CC) $(LDFLAGS) $(INCLUDES) \
	-Wl,--start-group \
	$(APP_OBJS) \
	$(LIBSTM32) \
	-lc -lm -lgcc \
	-Wl,--end-group \
	--output $(TARGET).elf

#------------------------------------------------------------------------------
# Generate HEX / BIN / LSS
#------------------------------------------------------------------------------
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $(TARGET).elf $(TARGET).hex

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin

$(TARGET).lss: $(TARGET).elf
	$(OBJDUMP) -h -S -C -D $(TARGET).elf > $(TARGET).lss

#------------------------------------------------------------------------------
# Build static library
#------------------------------------------------------------------------------
$(LIBSTM32): $(LIB_OBJS)
	$(AR) cr $(LIBSTM32) $(LIB_OBJS)

#------------------------------------------------------------------------------
# Compile application sources
#------------------------------------------------------------------------------
APP/src/main.o: APP/src/main.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

APP/src/stm32f10x_it.o: APP/src/stm32f10x_it.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

APP/src/dxl_hal.o: APP/src/dxl_hal.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

APP/src/dynamixel.o: APP/src/dynamixel.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

APP/src/syscalls.o: APP/src/syscalls.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

#------------------------------------------------------------------------------
# Compile STM32 library sources
#------------------------------------------------------------------------------
stm32f10x_lib/src/stm32f10x_adc.o: stm32f10x_lib/src/stm32f10x_adc.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_bkp.o: stm32f10x_lib/src/stm32f10x_bkp.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_exti.o: stm32f10x_lib/src/stm32f10x_exti.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_flash.o: stm32f10x_lib/src/stm32f10x_flash.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_gpio.o: stm32f10x_lib/src/stm32f10x_gpio.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_lib.o: stm32f10x_lib/src/stm32f10x_lib.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_nvic.o: stm32f10x_lib/src/stm32f10x_nvic.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_pwr.o: stm32f10x_lib/src/stm32f10x_pwr.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_rcc.o: stm32f10x_lib/src/stm32f10x_rcc.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_systick.o: stm32f10x_lib/src/stm32f10x_systick.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_tim.o: stm32f10x_lib/src/stm32f10x_tim.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_tim1.o: stm32f10x_lib/src/stm32f10x_tim1.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_usart.o: stm32f10x_lib/src/stm32f10x_usart.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/cortexm3_macro.o: stm32f10x_lib/src/cortexm3_macro.s
	$(CC) $(AFLAGS) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_can.o: stm32f10x_lib/src/stm32f10x_can.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_dma.o: stm32f10x_lib/src/stm32f10x_dma.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_i2c.o: stm32f10x_lib/src/stm32f10x_i2c.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_iwdg.o: stm32f10x_lib/src/stm32f10x_iwdg.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_rtc.o: stm32f10x_lib/src/stm32f10x_rtc.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_spi.o: stm32f10x_lib/src/stm32f10x_spi.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_wwdg.o: stm32f10x_lib/src/stm32f10x_wwdg.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

stm32f10x_lib/src/stm32f10x_vector.o: stm32f10x_lib/src/stm32f10x_vector.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

#------------------------------------------------------------------------------
# Clean
#------------------------------------------------------------------------------
clean:
	del /Q APP\src\*.o 2>nul
	del /Q stm32f10x_lib\src\*.o 2>nul
	del /Q stm32f10x_lib\libstm32.a 2>nul
	del /Q $(TARGET).elf 2>nul
	del /Q $(TARGET).hex 2>nul
	del /Q $(TARGET).bin 2>nul
	del /Q $(TARGET).lss 2>nul
	del /Q $(TARGET).elf.map 2>nul
