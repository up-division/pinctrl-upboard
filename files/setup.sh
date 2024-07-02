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

#only for ASL user space SPI bus
rm /lib/firmware/acpi-upgrades/pc00.spi0.spidev1.0.aml

if grep -q 'UPN-APL01' /sys/class/dmi/id/board_name
then
    cp $SRC_DIR/acpi/spi/pc00.spi0.spidev1.0.aml $BASEHOOKSDIR	
fi
if grep -q 'UPN-EDGE-ASLH01' /sys/class/dmi/id/board_name
then
    rm $BASEHOOKSDIR/pc00.spi0.spidev1*
    rm $BASEHOOKSDIR/pc00.spi1.spidev1*
    rm $BASEHOOKSDIR/pc00.i2c0.adc0.aml
    rm $BASEHOOKSDIR/pc00.i2c2.adc0.aml
fi    
if grep -q 'UPS-ASL01' /sys/class/dmi/id/board_name
then
    rm $BASEHOOKSDIR/pc00.spi0.spidev1*
    rm $BASEHOOKSDIR/pc00.spi1.spidev1*
    rm $BASEHOOKSDIR/pc00.i2c0.adc0.aml
    rm $BASEHOOKSDIR/pc00.i2c2.adc0.aml
fi    
if grep -q 'UP-APL01' /sys/class/dmi/id/board_name
then
    rm /lib/firmware/acpi-upgrades/pci0.i2c3.adc0.aml
fi
if grep -q 'UPX-TGL01' /sys/class/dmi/id/board_name
then
    #check blacklist
    if grep -q 'blacklist gpio_aaeon' /etc/modprobe.d/blacklist.conf
    then
        echo "blacklist gpio_aaeon exist"
    else
        echo "blacklist gpio_aaeon" >> /etc/modprobe.d/blacklist.conf
    fi
    rm /lib/firmware/acpi-upgrades/pc00.i2c0.adc0.aml
    rm /lib/firmware/acpi-upgrades/pc00.i2c2.adc0.aml
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

if grep -q 'UPS-ADLP01' /sys/class/dmi/id/board_name
then
    #remove pc00.i2c0 ASL, before update-initramfs
    rm /lib/firmware/acpi-upgrades/pc00.i2c0.adc0.aml
    rm /lib/firmware/acpi-upgrades/pc00.i2c2.adc0.aml  
fi

if grep -q 'UPX-ADLP01' /sys/class/dmi/id/board_name
then
    #remove pc00.i2c0 ASL, before update-initramfs
    rm /lib/firmware/acpi-upgrades/pc00.i2c0.adc0.aml
    rm /lib/firmware/acpi-upgrades/pc00.i2c2.adc0.aml  
fi

if grep -q 'UPX-MTL01' /sys/class/dmi/id/board_name
then
    #remove pc00.i2c0 ASL, before update-initramfs
    rm /lib/firmware/acpi-upgrades/pc00.i2c0.adc0.aml
fi




