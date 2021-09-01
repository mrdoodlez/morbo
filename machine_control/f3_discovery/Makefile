gccpref = arm-none-eabi-
blddir = build
target = $(blddir)/mc.elf
srcdir = Src

cubedir ?= /home/stan/pkgs/STM32CubeF3

incflags  = -I./Inc
incflags += -I$(cubedir)/Drivers/STM32F3xx_HAL_Driver/Inc
incflags += -I$(cubedir)/Drivers/CMSIS/Device/ST/STM32F3xx/Include
incflags += -I$(cubedir)/Drivers/CMSIS/Include
incflags += -I$(cubedir)/Drivers/BSP/STM32F3-Discovery
incflags += -I../grove_motor_driver
incflags += -I../remote_control/device
incflags += -I../mc_control_grove

VPATH   = ./Src
VPATH  += $(cubedir)/Drivers/STM32F3xx_HAL_Driver/Src
VPATH  += $(cubedir)/Drivers/BSP/STM32F3-Discovery
VPATH  += ../grove_motor_driver
VPATH  += ../remote_control/device
VPATH  += ../mc_control_grove

cflags = -mcpu=cortex-m4 \
	-g3 -DCORE_CM4 \
	'-D__weak=__attribute__((weak))' \
	'-D__packed=__attribute__((__packed__))' \
	-DUSE_HAL_DRIVER \
	-DSTM32F303xC \
	-O3 \
	-ffunction-sections \
	-Wall \
	-MMD -MP \
	--specs=nano.specs \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mthumb

ldflags = -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb \
	-g3 -T"STM32F303VCTx_FLASH.ld" \
	-Wl,-Map="$(blddir)/out.map" \
	-Wl,--gc-sections -static --specs=nano.specs
	
objs = $(blddir)/main.o \
	$(blddir)/startup_stm32f303xc.o \
	$(blddir)/stm32f3xx_hal_msp.o \
	$(blddir)/stm32f3xx_it.o \
	$(blddir)/system_stm32f3xx.o \
	$(blddir)/stm32f3xx_hal.o \
	$(blddir)/stm32f3xx_hal_uart.o \
	$(blddir)/stm32f3xx_hal_uart_ex.o \
	$(blddir)/stm32f3xx_hal_gpio.o \
	$(blddir)/stm32f3xx_hal_rcc.o \
	$(blddir)/stm32f3xx_hal_rcc_ex.o \
	$(blddir)/stm32f3xx_hal_i2c.o \
	$(blddir)/stm32f3xx_hal_dma.o \
	$(blddir)/stm32f3xx_hal_cortex.o \
	$(blddir)/stm32f3_discovery.o \
	$(blddir)/grove_driver.o \
	$(blddir)/rc_device.o \
	$(blddir)/mc_control.o

all: $(blddir) $(target)

$(blddir):
	mkdir $(blddir)

clean:
	rm -rf $(blddir)

$(target): $(objs)
	$(gccpref)gcc $(ldflags) $(objs) -o $@

$(blddir)/%.o: %.s
	@mkdir -p $(dir $@)
	$(gccpref)gcc $(cflags) $(flags_$(notdir $@)) -c -o $@ $<

$(blddir)/%.o: %.c
	@mkdir -p $(dir $@)
	$(gccpref)gcc $(cflags) $(incflags) $(flags_$(notdir $@)) -c -o $@ $<

