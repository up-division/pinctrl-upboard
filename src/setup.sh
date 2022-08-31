#!/bin/sh
BASEHOOKSDIR="/lib/firmware/acpi-upgrades"
SRC_DIR=`pwd`
echo $SRC_DIR
#check board name
echo $(cat /sys/class/dmi/id/board_name)
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
    cp $SRC_DIR/acpi/up6000/acpi-upgrades /etc/initramfs-tools/hooks/
    cp $SRC_DIR/acpi/up6000/*.aml $BASEHOOKSDIR
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
    cp $SRC_DIR/acpi/up6000/acpi-upgrades /etc/initramfs-tools/hooks/
    cp $SRC_DIR/acpi/up6000/*.aml $BASEHOOKSDIR
    echo 'acpi files copied!'
fi
