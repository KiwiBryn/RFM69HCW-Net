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
*/
namespace devMobile.IoT.Rfm69Hcw.RegisterRead
{
	using System;
	using System.Diagnostics;
	using System.Threading.Tasks;
	using Windows.ApplicationModel.Background;
	using Windows.Devices.Gpio;
	using Windows.Devices.Spi;

	public sealed class StartupTask : IBackgroundTask
	{
		private const byte RegVersion = 0x10;

		public void Run(IBackgroundTaskInstance taskInstance)
		{
			GpioController gpioController = GpioController.GetDefault();

			SpiController spiController = SpiController.GetDefaultAsync().AsTask().GetAwaiter().GetResult();
			var settings = new SpiConnectionSettings(0) // Seegel Design
																	  //var settings = new SpiConnectionSettings(1)  // Adafruit
			{
				ClockFrequency = 500000,
				Mode = SpiMode.Mode0,
			};

			SpiDevice Device = spiController.GetDevice(settings);

			Task.Delay(500).Wait();

			while (true)
			{
				byte[] writeBuffer = new byte[] { RegVersion };
				byte[] readBuffer = new byte[1];

				Device.TransferSequential(writeBuffer, readBuffer);

				byte registerValue = readBuffer[0];
				Debug.WriteLine("Register 0x{0:x2} - Value 0X{1:x2} - Bits {2}", RegVersion, registerValue, Convert.ToString(registerValue, 2).PadLeft(8, '0'));

				Task.Delay(10000).Wait();
			}
		}
	}
}
