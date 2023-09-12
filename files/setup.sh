#!/bin/sh
BASEHOOKSDIR="/lib/firmware/acpi-upgrades"
SRC_DIR=`pwd`
echo $SRC_DIR
#check board name
echo "UP_BOARD=$(cat /sys/class/dmi/id/board_name)"

#force enable ADC & SPI declare in ACPI
mkdir -p /lib/firmware/acpi-upgrades
cp $SRC_DIR/acpi/acpi-upgrades /etc/initramfs-tools/hooks/
cp $SRC_DIR/acpi/adc/*.aml $BASEHOOKSDIR
cp $SRC_DIR/acpi/spi/*.aml $BASEHOOKSDIR

if grep -q 'UPX-TGL01' /sys/class/dmi/id/board_name
then
    #check blacklist
    if grep -q 'blacklist gpio_aaeon' /etc/modprobe.d/blacklist.conf
    then
        echo "blacklist gpio_aaeon exist"
    else
        echo "blacklist gpio_aaeon" >> /etc/modprobe.d/blacklist.conf
    fi
fi
if grep -q 'UPN-EHL01' /sys/class/dmi/id/board_name
then
    #check blacklist
    if grep -q 'blacklist gpio_aaeon' /etc/modprobe.d/blacklist.conf
    then
        echo "blacklist gpio_aaeon exist"
    else
        echo "blacklist gpio_aaeon" >> /etc/modprobe.d/blacklist.conf
    fi
    mkdir -p /lib/firmware/acpi-upgrades
    cp $SRC_DIR/acpi/acpi-upgrades /etc/initramfs-tools/hooks/
    cp $SRC_DIR/acpi/gpio/*.aml $BASEHOOKSDIR
    echo 'acpi files copied!'
fi
if grep -q 'UPS-EHL01' /sys/class/dmi/id/board_name
then
    #check blacklist
    if grep -q 'blacklist gpio_aaeon' /etc/modprobe.d/blacklist.conf
    then
        echo "blacklist gpio_aaeon exist"
    else
        echo "blacklist gpio_aaeon" >> /etc/modprobe.d/blacklist.conf
    fi
    mkdir -p /lib/firmware/acpi-upgrades
    cp $SRC_DIR/acpi/acpi-upgrades /etc/initramfs-tools/hooks/
    cp $SRC_DIR/acpi/gpio/*.aml $BASEHOOKSDIR
    echo 'acpi files copied!'
fi
if grep -q 'UP-APL03' /sys/class/dmi/id/board_name
then
    #check blacklist
    if grep -q 'blacklist gpio_aaeon' /etc/modprobe.d/blacklist.conf
    then
        echo "blacklist gpio_aaeon exist"
    else
        echo "blacklist gpio_aaeon" >> /etc/modprobe.d/blacklist.conf
    fi
fi

if grep -q 'UP-ADLN01' /sys/class/dmi/id/board_name
then
    #remove pc00.i2c0 ASL, before update-initramfs
    rm /lib/firmware/acpi-upgrades/pc00.i2c0.adc0.aml
fi




