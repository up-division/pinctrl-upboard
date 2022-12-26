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

ifeq ($(KERNEL_SRC),)
KERNEL_SRC=/lib/modules/$(shell uname -r)/build
endif
	
all:
	make -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean
	
# build root.
install: all
	make -C $(KERNEL_SRC) M=$(PWD) modules_install
	
# build root.
uninstall:
	make -C $(KERNEL_SRC) M=$(PWD) modules_uninstall

# Yocto out-of-tree kernel modules
modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

setup:
	sh setup.sh
