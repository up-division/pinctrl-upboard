SUMMARY = "UP boards pin controller modules"
DESCRIPTION = "${SUMMARY}"
LICENSE = "GPL-2.0-only"

inherit module

SRC_URI = "file://Makefile \
           file://upboard-fpga.c \
           file://upboard-ec.c \
           file://leds-upboard.c \
           file://pinctrl-upboard.c \
           file://pinctrl-upelement.c \
           file://COPYING \
          "

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

RPROVIDES:${PN} += "kernel-module-pinctrl-upboard"
