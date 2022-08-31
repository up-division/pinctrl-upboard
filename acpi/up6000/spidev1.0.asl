/*
 * This ASL can be used to declare a spidev device on SPI1 CS0
 */
DefinitionBlock ("", "SSDT", 5, "INTEL", "SPIDEV0", 0x00000001)
{
    External (_SB.PC00.SPI1, DeviceObj)

    Scope (\_SB.PC00.SPI1)
    {
        Device (TP0)
        {
            Name (_HID, "SPT0001")  // _HID: Hardware ID
            Name (_DDN, "SPI test device connected to CS0")  // _DDN: DOS Device Name
            Name (_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                SpiSerialBusV2 (0x0000, PolarityLow, FourWireMode, 0x08,
                    ControllerInitiated, 0x000F4240, ClockPolarityLow,
                    ClockPhaseFirst, "\\_SB.PC00.SPI1",
                    0x00, ResourceConsumer, , Exclusive,
                    )
            })
        }
    }
}

