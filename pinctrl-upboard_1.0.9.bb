SUMMARY = "UP boards pin controller modules"
DESCRIPTION = "${SUMMARY}"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

SRC_URI = "file://Makefile \
	   file://core.h \
	   file://pinctrl-intel.h \
           file://upboard-cpld.c \
	   file://upboard-cpld.h \
           file://upboard-ec.c \
           file://upboard-ec.h \
           file://leds-upboard.c \
           file://pinctrl-upboard.c \
           file://pinctrl-upelement.c \
           file://pwm-upboard.c \
           file://pwm-lpss.h \
           file://pwm-lpss.c \
           file://pwm-lpss-pci.c \
           file://protos.c \
           file://COPYING \
          "

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

RPROVIDES:${PN} += "kernel-module-upboard-cpld"
RPROVIDES:${PN} += "kernel-module-upboard-ec"
RPROVIDES:${PN} += "kernel-module-leds-upboard"
RPROVIDES:${PN} += "kernel-module-pinctrl-upboard"
RPROVIDES:${PN} += "kernel-module-pinctrl-upelement"
RPROVIDES:${PN} += "kernel-module-pwm-upboard"
