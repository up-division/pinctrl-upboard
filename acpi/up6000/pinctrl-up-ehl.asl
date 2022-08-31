DefinitionBlock ("", "SSDT", 5, "INTL", "PINCTRL", 1)
{
    Scope (\_SB)
    {
	Device (PCTL)
	{
		Name(_HID, "AANT0000")
		Name(_DDN, "UP 6000 Pin Controller")
		Name(_UID, One)
		Name(_HRV, 0x00)
		Method (_CRS, 0, Serialized)
        	{
                	Name (SBUF, ResourceTemplate()
                	{
                        	//0 I2C1_SDA 866 PIN3
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI1", 0, ResourceConsumer) { 22 }
                        	//1 I2C1_SCL 867 PIN5
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI1", 0, ResourceConsumer) { 23 }
                        	//2 ADC0 748 PIN7
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 31 }
                        	//3 GPIO17 731 PIN11
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 14 }
                        	//4 GPIO27 879 PIN13
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI1", 0, ResourceConsumer) { 35 }
                        	//5 GPIO22 971 PIN15
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 14 }
                        	//6 SPI_MOSI 979 PIN19
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 22 }
                        	//7 SPI_MISO 978 PIN21
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 21 }
                        	//8 SPI_CLK  977 PIN23
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 20 }
                        	//9 I2C0_SDA 966 PIN27
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 9 }
                        	//10 GPIO5 759 PIN29
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 42 }
                        	//11 GPIO6 760 PIN31
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 43 }
                        	//12 PWM1  888 PIN33
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI1", 0, ResourceConsumer) { 44 }
                        	//13 I2S_FRM 1011 PIN35
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 54 }
                        	//14 GPIO26 751 PIN37
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 34 }
                        	//15 UART1_TX 730 PIN8
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 13 }
                        	//16 UART1_RX 729 PIN10
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 12 }
                        	//17 I2S_CLK 1010 PIN12
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 53 }
                        	//18 GPIO23 795 PIN16
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 78 }
                        	//19 GPIO24 794 PIN18
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 77 }
                        	//20 GPIO25 968 PIN22
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 11 }
                        	//21 SPI_CS0 976 PIN24
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 19 }
                        	//22 SPI_CS1 980 PIN26
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 23 }
                        	//23 I2C0_SCL 967 PIN28
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 10 }
                        	//24 PWM0 722 PIN32
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 5 }
                        	//25 GPIO16 732 PIN36
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 15 }
                        	//26 I2S_DIN 1013 PIN38
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 56 }
                        	//27 I2S_DOUT 1012 PIN40
                        	GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 55 }
                                //blue-gpio 868
				GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI1", 0, ResourceConsumer) { 24 }
                                //yellow-gpio 726
				GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI4", 0, ResourceConsumer) { 9 }
                                //green-gpio 961
				GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI0", 0, ResourceConsumer) { 4 }
                                //red-gpio 869
				GpioIo (Exclusive, PullDefault, 0, 0, IoRestrictionNone,
                                	"\\_SB.GPI1", 0, ResourceConsumer) { 25 }

                	})
                	Return (SBUF)
        	}

        	// ACPI 5.1 _DSD used for naming the GPIOs
        	Name (_DSD, Package ()
        	{
			Buffer(0x10)
			{
				0x14, 0xD8, 0xFF, 0xDA, 0xBA, 0x6E, 0x8C, 0x4D, 
				0x8A, 0x91, 0xBC, 0x9B, 0xBF, 0x4A, 0xA3, 0x01
			}, 
                	Package ()
                	{
                        	Package () { 
                        		"external-gpios", 
                        		Package () { 
                        		^PCTL, 0, 0, 0, ^PCTL, 1, 0, 0, ^PCTL, 2, 0, 0, ^PCTL, 3, 0, 0, ^PCTL, 4, 0, 0,
                        		^PCTL, 5, 0, 0, ^PCTL, 6, 0, 0, ^PCTL, 7, 0, 0, ^PCTL, 8, 0, 0, ^PCTL, 9, 0, 0,
                        		^PCTL,10, 0, 0, ^PCTL,11, 0, 0, ^PCTL,12, 0, 0, ^PCTL,13, 0, 0, ^PCTL,14, 0, 0,
                        		^PCTL,15, 0, 0, ^PCTL,16, 0, 0, ^PCTL,17, 0, 0, ^PCTL,18, 0, 0, ^PCTL,19, 0, 0,
                        		^PCTL,20, 0, 0, ^PCTL,21, 0, 0, ^PCTL,22, 0, 0, ^PCTL,23, 0, 0, ^PCTL,24, 0, 0,
                        		^PCTL,25, 0, 0, ^PCTL,26, 0, 0, ^PCTL,27, 0, 0
                          		} 
                          	},
 				Package ()
				{
					"blue-gpio", 
					Package() {^PCTL, 28, 0, 0}
				},             	
				Package ()
				{
					"yellow-gpio", 
					Package() {^PCTL, 29, 0, 0}
				}, 
				Package ()
				{
					"green-gpio", 
					Package() {^PCTL, 30, 0, 0}
				}, 
				Package ()
				{
					"red-gpio", 
					Package() {^PCTL, 31, 0, 0}
				}
               		}
        	})
        	Method(_STA, 0, NotSerialized)
		{
			Return(0x0F)
		}
	}
    }
}
