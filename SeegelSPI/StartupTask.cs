/*
    Copyright ® 2019 May devMobile Software, All Rights Reserved
 
    MIT License

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE

	 https://www.seegel-systeme.de/produkt/raspyrfm-ii/

	 The RaspyRFM is plugged into the Raspberry PI of pin 17-26.

	 From the docs for the dual RFM69 mini at
	 17 -> 3,3 V
	 18 (GPIO 24) -> DIO1 bei Einzelmodul, DIO0 Slave bei Doppelmodul
    19 (MOSI) -> MOSI
    20 -> GND
    21 (MISO) ->MISO
    22 (GPIO 25) -> DIO0
    23 (SCK) -> SCK
    24 (CE0) -> NSS Master
    25 -> GND
    26 (CE1) -> DIO2 bei Einzelmodul, NSS Slave bei Doppelmodul
 */
namespace devMobile.IoT.Rfm69hcw.SeegelSpi
{
	using System;
	using System.Diagnostics;
	using System.Threading;
	using Windows.ApplicationModel.Background;
	using Windows.Devices.Spi;

	public sealed class StartupTask : IBackgroundTask
	{
		private const byte RegVersion = 0x10;

		public void Run(IBackgroundTaskInstance taskInstance)
		{
			SpiController spiController = SpiController.GetDefaultAsync().AsTask().GetAwaiter().GetResult();
			var settings = new SpiConnectionSettings(0)
			{
				ClockFrequency = 500000,
				Mode = SpiMode.Mode0,
			};

			SpiDevice Device = spiController.GetDevice(settings);

			while (true)
			{
				byte[] writeBuffer = new byte[] { RegVersion }; // RegVersion
				byte[] readBuffer = new byte[1];

				Device.TransferSequential(writeBuffer, readBuffer);

				byte registerValue = readBuffer[0];
				Debug.WriteLine("Register 0x{0:x2} - Value 0X{1:x2} - Bits {2}", RegVersion, registerValue, Convert.ToString(registerValue, 2).PadLeft(8, '0'));

				Thread.Sleep(10000);
			}
		}
	}
}

