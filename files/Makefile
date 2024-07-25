# SPDX-License-Identifier: GPL-2.0

CONFIG_X86:=y
CONFIG_ACPI:=y
CONFIG_SPI:=y
CONFIG_REGMAP:=y
CONFIG_OF_GPIO:=y
CONFIG_GPIOLIB:=y
CONFIG_GPIOLIB_IRQCHIP:=y
CONFIG_GPIO_ACPI:=y
CONFIG_REGULATOR:=y
CONFIG_MODULE_SIG:=n

obj-m += upboard-cpld.o
obj-m += upboard-ec.o
obj-m += leds-upboard.o
obj-m += pinctrl-upboard.o
obj-m += pinctrl-upelement.o
obj-m += pwm-upboard.o
obj-m += pwm-lpss-pci.o
obj-m += pwm-lpss.o
spi-pxa2xx-platform-objs := spi-pxa2xx.o spi-pxa2xx-dma.o
obj-m += spi-pxa2xx-platform.o
obj-y += protos.o

KVERSION := $(shell uname -r)
KERNEL_SRC := /lib/modules/$(KVERSION)/build

all: check_protos
	make -C $(KERNEL_SRC) M=$(PWD) modules
	
clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean
	rm -f protos.h
	
# build root.
install: all
	make -C $(KERNEL_SRC) M=$(PWD) modules_install
	
# build root.
uninstall:
	make -C $(KERNEL_SRC) M=$(PWD) modules_uninstall

# Yocto out-of-tree kernel modules
modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install
	
# check kernel prototypes definition
check_protos:
	@echo "checking pwm dev struct..." 
	rm -f ./protos.h
	rm -f ./protos.o
	sed -i 's/#define CHECK_SPI_SSP/#define CHECK_PWM_PDEV/g' protos.c
	make -i -C $(KERNEL_SRC) M=$(PWD) protos.o; ([ -f protos.o ] && echo "#define TYPES_PWM_PDEV 1" > protos.h || echo "#define TYPES_PWM_PDEV 0" > protos.h )
	@echo "checking pwm prototypes..." 
	rm -f ./protos.o
	sed -i 's/#define CHECK_PWM_PDEV/#define CHECK_PWM/g' protos.c
	make -i -C $(KERNEL_SRC) M=$(PWD) protos.o; ([ -f protos.o ] && echo "#define TYPES_NO_ERROR_CODE 1" >> protos.h || echo "#define TYPES_NO_ERROR_CODE 0" >> protos.h )
	@echo "checking gpio prototypes..." 
	rm -f ./protos.o
	sed -i 's/#define CHECK_PWM/#define CHECK_GPIO/g' protos.c
	make -i -C $(KERNEL_SRC) M=$(PWD) protos.o; ([ -f protos.o ] && echo "#define TYPES_NO_OFFSET 1" >> protos.h || echo "#define TYPES_NO_OFFSET 0" >> protos.h )
	@echo "checking spi naming..." 
	rm -f ./protos.o
	sed -i 's/#define CHECK_GPIO/#define CHECK_SPI_NAMING/g' protos.c
	make -i -C $(KERNEL_SRC) M=$(PWD) protos.o; ([ -f protos.o ] && echo "#define TYPES_IS_SLAVE 1" >> protos.h || echo "#define TYPES_IS_SLAVE 0" >> protos.h )
	@echo "checking spi ssp..." 
	rm -f ./protos.o
	sed -i 's/#define CHECK_SPI_NAMING/#define CHECK_SPI_SSP/g' protos.c
	make -i -C $(KERNEL_SRC) M=$(PWD) protos.o; ([ -f protos.o ] && echo "#define TYPES_SSP_PDEV 1" >> protos.h || echo "#define TYPES_SSP_PDEV 0" >> protos.h )

setup:
	sh setup.sh
