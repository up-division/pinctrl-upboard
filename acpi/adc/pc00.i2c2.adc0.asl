/*
 * to declare a ADC device TGL/ADL
 */
DefinitionBlock ("", "SSDT", 1, "ALASKA", "ADC081C", 0x00000004)
{
    External (_SB_.PC00.I2C2, DeviceObj)

    Scope (\_SB.PC00.I2C2)
    {
        Device (ADC0)
        {
            Name (_HID, "ADC081C")  // _HID: Hardware ID
            Method (_CRS, 0, Serialized)  // _CRS: Current Resource Settings
            {
                Name (UBUF, ResourceTemplate ()
                {
                    I2cSerialBusV2 (0x0054, ControllerInitiated, 0x00061A80,
                        AddressingMode7Bit, "\\_SB.PC00.I2C2",
                        0x00, ResourceConsumer, , Exclusive,
                        )
                })
                Return (UBUF) /* \_SB_.PC00.I2C2.ADC0._CRS.UBUF */
            }

            Method (_STA, 0, NotSerialized)  // _STA: Status
            {
                Return (0x0F)
            }
        }
    }
}
