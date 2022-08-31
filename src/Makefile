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

obj-m += upboard-fpga.o
obj-m += upboard-ec.o
obj-m += leds-upboard.o
obj-m += pinctrl-upboard.o
obj-m += pinctrl-upelement.o

ifeq ($(KERNELDIR),)
KERNELDIR=/lib/modules/$(shell uname -r)/build
endif
	
all:
	make -C $(KERNELDIR) M=$(PWD) modules

clean:
	make -C $(KERNELDIR) M=$(PWD) clean
	
# build root.
install: all
	make -C $(KERNELDIR) M=$(PWD) modules_install
	
# build root.
uninstall:
	make -C $(KERNELDIR) M=$(PWD) modules_uninstall

setup:
	sh setup.sh
