# pinctrl-upboard
UP boards pin controller driver for UP HAT pins.

Supported products
=============================================
* UP Squared/UP Squared Pro
* UP Core
* UP Core Plus
* UP Xtreme
* UP Xtreme i11
* UP Xtreme i12
* UP Squared 6000
* UP Squared 4000
* UP Element

Building
=============================================
build deb package
----------------------
To release debian package purpose, please make sure you have installed devscripts and debhelper package,
in pinctrl-upboard folder type following command
```
debuild -i -us -uc
```

build kernel modules
----------------------
Developing & debug purpose, please make sure you have installed linux herader and build-essential,
in pinctrl-upboard folder type following command
```
cd src && make
```

HAT Pins information
=============================================
please refer our UP WiKi for more detail
* https://wiki.up-community.org/
