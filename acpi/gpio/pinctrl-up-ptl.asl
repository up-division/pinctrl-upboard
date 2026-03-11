DefinitionBlock ("", "SSDT", 1, "ALASKA", "PINCTRL", 0x00000003)
{
    External (PCEN, UnknownObj)

    Scope (\_SB)
    {
        Device (PCTL)
        {
            Name (_HID, "AANT0F04")  // _HID: Hardware ID
            Name (_DDN, "UPX-PTL01 CPLD-based Pin Controller")  // _DDN: DOS Device Name
            Name (_UID, One)  // _UID: Unique ID
            Name (_HRV, One)  // _HRV: Hardware Revision
            Method (_CRS, 0, Serialized)  // _CRS: Current Resource Settings
            {
                Name (NBUF, ResourceTemplate ()
                {
                	//3
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI3", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0024
                        }
                        //5
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI3", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0025
                        }
                        //7
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI3", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x001E
                        }
                        //11
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI3", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0019
                        }
                        //13
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI5", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0006
                        }
                        //15
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI5", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0008
                        }
                        //19
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI1", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0026
                        }
                        //21
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI1", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0027
                        }
                        //23
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI1", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0025
                        }
                        //27
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI3", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0022
                        }
                        //29
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI5", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0003
                        }
                        //31
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI5", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0002
                        }
                        //33
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI5", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0005
                        }
                        //35
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI4", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0003
                        }
                        //37
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI5", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0013
                        }
                        //8
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI3", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0018
                        }
                        //10
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI3", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0017
                        }
                        //12
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI4", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0002
                        }
                        //16
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI3", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x001D
                        }
                        //18
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI5", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0012
                        }
                        //22
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI5", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0031
                        }
                        //24
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI1", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x002B
                        }
                        //26
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI5", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0007
                        }
                        //28
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI3", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0023
                        }
                        //32
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI5", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0004
                        }
                        //36
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI3", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x001A
                        }
                        //38
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI4", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0001
                        }
                        //40
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI4", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0000
                        }
                        //data-in
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI1", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x000E
                        }
                        //data-out
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI1", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x002A
                        }
                        //clear
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI1", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x000A
                        }
                        //enable
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI1", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0010
                        }
                        //reset
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI1", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0012
                        }
                        //strobe
                    GpioIo (Exclusive, PullDefault, 0x0000, 0x0000, IoRestrictionNone,
                        "\\_SB.GPI1", 0x00, ResourceConsumer, ,
                        )
                        {   // Pin list
                            0x0011
                        }
                })
                Return (NBUF) /* \_SB_.PCTL._CRS.NBUF */
            }

            Name (_DSD, Package (0x02)  // _DSD: Device-Specific Data
            {
                ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301") /* Device Properties for _DSD */, 
                Package (0x07)
                {
                    Package (0x02)
                    {
                        "external-gpios", 
                        Package (0x70)
                        {
                            ^PCTL, 
                            Zero, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            One, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x02, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x03, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x04, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x05, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x06, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x07, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x08, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x09, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x0A, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x0B, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x0C, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x0D, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x0E, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x0F, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x10, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x11, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x12, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x13, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x14, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x15, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x16, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x17, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x18, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x19, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x1A, 
                            Zero, 
                            Zero, 
                            ^PCTL, 
                            0x1B, 
                            Zero, 
                            Zero
                        }
                    }, 

                    Package (0x02)
                    {
                        "datain-gpio", 
                        Package (0x04)
                        {
                            ^PCTL, 
                            0x1C, 
                            Zero, 
                            Zero
                        }
                    }, 

                    Package (0x02)
                    {
                        "dataout-gpio", 
                        Package (0x04)
                        {
                            ^PCTL, 
                            0x1D, 
                            Zero, 
                            Zero
                        }
                    }, 

                    Package (0x02)
                    {
                        "clear-gpio", 
                        Package (0x04)
                        {
                            ^PCTL, 
                            0x1E, 
                            Zero, 
                            Zero
                        }
                    }, 

                    Package (0x02)
                    {
                        "enable-gpio", 
                        Package (0x04)
                        {
                            ^PCTL, 
                            0x1F, 
                            Zero, 
                            Zero
                        }
                    }, 

                    Package (0x02)
                    {
                        "reset-gpio", 
                        Package (0x04)
                        {
                            ^PCTL, 
                            0x20, 
                            Zero, 
                            Zero
                        }
                    }, 

                    Package (0x02)
                    {
                        "strobe-gpio", 
                        Package (0x04)
                        {
                            ^PCTL, 
                            0x21, 
                            Zero, 
                            Zero
                        }
                    }
                }
            })
            Method (_STA, 0, NotSerialized)  // _STA: Status
            {
                If ((PCEN == One))
                {
                    Return (0x0F)
                }
                Else
                {
                    Return (Zero)
                }
            }
        }
    }
}

