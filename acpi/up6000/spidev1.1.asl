/*
 * This ASL can be used to declare a spidev device on SPI1 CS1
 */
DefinitionBlock ("", "SSDT", 5, "INTEL", "SPIDEV1", 0x00000001)
{
    External (_SB.PC00.SPI1, DeviceObj)

    Scope (\_SB.PC00.SPI1)
    {
        Device (TP1)
        {
            Name (_HID, "SPT0001")  // _HID: Hardware ID
            Name (_DDN, "SPI test device connected to CS1")  // _DDN: DOS Device Name
            Name (_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                SpiSerialBusV2 (0x0001, PolarityLow, FourWireMode, 0x08,
                    ControllerInitiated, 0x000F4240, ClockPolarityLow,
                    ClockPhaseFirst, "\\_SB.PC00.SPI1",
                    0x00, ResourceConsumer, , Exclusive,
                    )
            })
        }
    }
}

