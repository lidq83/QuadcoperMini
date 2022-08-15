ROOTPATH	=	../../
#------------------------------------------------------------------------#
# OS Arch
OS_ARCH		=	arm/armv7-m

#------------------------------------------------------------------------#
# Output files
ELF_FILE	=	dolphin-os.elf
BIN_FILE	=	dolphin-os.bin
HEX_FILE	=	dolphin-os.hex
INFO_FILE	=	dolphin-os.info
CODE_FILE	=	dolphin-os.code

#------------------------------------------------------------------------#
# Cross Compiler
CC_PRE		=	arm-none-eabi

CC			=	$(CC_PRE)-gcc
OBJCOPY		=	$(CC_PRE)-objcopy
OBJDUMP		=	$(CC_PRE)-objdump
READELF		=	$(CC_PRE)-readelf

#------------------------------------------------------------------------#
# Flags
#CFLAGS		+=	-DDEBUG
CFLAGS		+=	-Os -ffunction-sections -fdata-sections
CFLAGS		+=	-std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB
CFLAGS		+=	-mcpu=cortex-m3 --specs=nosys.specs 
CFLAGS		+=	-Wl,--gc-sections -static --specs=nano.specs 
CFLAGS		+=	-mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
CFLAGS		+=	-T"dolphin-os.ld" 

# Libraries
STM32_LIBS	=	libs/STM32F1xx_HAL_Driver
CFLAGS		+=	-I$(STM32_LIBS)/Inc
CFLAGS		+=	-Ilibs/CMSIS/Include
CFLAGS		+=	-Ilibs/CMSIS/Device/ST/STM32F1xx/Include

#------------------------------------------------------------------------#
# Kernel Path
KERNEL_PATH	=	$(ROOTPATH)/kernel
CFLAGS		+=	-I$(KERNEL_PATH)/kernel
CFLAGS		+=	-I$(KERNEL_PATH)/lib
CFLAGS		+=	-I$(KERNEL_PATH)/fs

#------------------------------------------------------------------------#
# Src Path
SRCS		=	./src
CFLAGS		+=	-I$(SRCS)
CFLAGS		+=	-I./inc

#------------------------------------------------------------------------#
# Drivers Path
DRVS		=	$(ROOTPATH)/drivers
CFLAGS		+=	-I$(DRVS)
CFLAGS		+=	-I$(DRVS)/led
CFLAGS		+=	-I$(DRVS)/mpu6050
CFLAGS		+=	-I$(DRVS)/nrf2401

#------------------------------------------------------------------------#
# Modules Path
MODULES		=	$(ROOTPATH)/modules
CFLAGS		+=	-I$(MODULES)
CFLAGS		+=	-I$(MODULES)/led
CFLAGS		+=	-I$(MODULES)/controller
CFLAGS		+=	-I$(MODULES)/nrf2401

#------------------------------------------------------------------------#
# Libs Path
LIBS		=	$(ROOTPATH)/libs
CFLAGS		+=	-I$(LIBS)

#------------------------------------------------------------------------#
# Main Board
SRC			+=	$(SRCS)/main.c

#------------------------------------------------------------------------#
# Drivers
SRC			+=	$(DRVS)/led/led.c \
				$(DRVS)/mpu6050/i2cdev.c \
				$(DRVS)/mpu6050/mpu6050.c \
				$(DRVS)/nrf2401/nrf2401.c

#------------------------------------------------------------------------#
# Modules
SRC			+=	$(MODULES)/led/led_task.c \
				$(MODULES)/controller/controller_task.c \
				$(MODULES)/nrf2401/nrf2401_task.c

#------------------------------------------------------------------------#
# Libs
SRC			+=	$(LIBS)/buff_s.c \
				$(LIBS)/crc.c \
				$(LIBS)/protocol.c \
				$(LIBS)/rand.c

#------------------------------------------------------------------------#
# System
SRC			+=	./src/system_stm32f1xx.c \
				./src/sysmem.c \
				./src/syscalls.c \
				./src/stm32f1xx_it.c \
				./src/stm32f1xx_hal_msp.c

STARTUP		=	./src/startup_stm32f103c8tx.s

#------------------------------------------------------------------------#
# Hal Libraries
SRC			+=	$(STM32_LIBS)/Src/stm32f1xx_hal.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_cortex.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_adc.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_adc_ex.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_dma.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_exti.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_flash.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_flash_ex.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_gpio.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_gpio_ex.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_i2c.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_pwr.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_rcc.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_rcc_ex.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_spi.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_tim.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_tim_ex.c \
				$(STM32_LIBS)/Src/stm32f1xx_hal_uart.c

#------------------------------------------------------------------------#
# OS Arch
SRC			+=	$(KERNEL_PATH)/arch/$(OS_ARCH)/stack.c \
				$(KERNEL_PATH)/arch/$(OS_ARCH)/switch.S

#------------------------------------------------------------------------#
# OS Kernel
SRC			+=	$(KERNEL_PATH)/kernel/core.c \
				$(KERNEL_PATH)/kernel/pcb.c \
				$(KERNEL_PATH)/kernel/sche.c \
				$(KERNEL_PATH)/kernel/sem.c \
				$(KERNEL_PATH)/kernel/top.c \
				$(KERNEL_PATH)/fs/vfs.c \
				$(KERNEL_PATH)/fs/fs.c \
				$(KERNEL_PATH)/fs/fcntl.c \
				$(KERNEL_PATH)/lib/list.c

STARTUP_OBJ	=	startup_stm32f103c8tx.o

all:$(BIN_FILE) $(HEX_FILE) $(INFO_FILE) $(CODE_FILE)

$(BIN_FILE):$(ELF_FILE)
	$(OBJCOPY) -O binary $^ $@

$(HEX_FILE):$(ELF_FILE)
	$(OBJCOPY) -O ihex $^ $@

$(INFO_FILE):$(ELF_FILE)
	$(READELF) -a $^ > $@

$(CODE_FILE):$(ELF_FILE)
	$(OBJDUMP) -S $^ > $@

$(STARTUP_OBJ):$(STARTUP)
	$(CC) $(CFLAGS) $^ -c $(STARTUP)

$(ELF_FILE):$(SRC) $(STARTUP_OBJ)
	$(CC) $(CFLAGS)	$^ -o $@

install:
	#@st-flash write $(BIN_FILE) 0x8000000
	jlink -NoGui -commanderscript loadbin_cmd.jlink

clean:
	@rm -rvf *.o *.bin *.code *.elf *.hex *.info
