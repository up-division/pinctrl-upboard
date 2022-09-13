SUMMARY = "UP boards pin controller modules"
DESCRIPTION = "${SUMMARY}"
LICENSE = "CLOSED"

inherit module

SRC_URI = "file://Makefile \
	   file://core.h \
           file://upboard-fpga.c \
	   file://upboard-fpga.h \
           file://upboard-ec.c \
           file://upboard-ec.h \
           file://leds-upboard.c \
           file://pinctrl-upboard.c \
           file://pinctrl-upelement.c \
           file://COPYING \
          "

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

RPROVIDES:${PN} += "kernel-module-upboard-fpga"
RPROVIDES:${PN} += "kernel-module-upboard-ec"
RPROVIDES:${PN} += "kernel-module-leds-upboard"
RPROVIDES:${PN} += "kernel-module-pinctrl-upboard"
RPROVIDES:${PN} += "kernel-module-pinctrl-upelement"
