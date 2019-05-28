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

	 RegFiFo 0x00 thru RegPacketConfig2 0x3D

*/
namespace devMobile.IoT.Rfm69Hcw.RegisterScan
{
	using System;
	using System.Diagnostics;
	using System.Threading.Tasks;
	using Windows.ApplicationModel.Background;
	using Windows.Devices.Spi;

	public sealed class Rfm69HcwDevice
	{
		private SpiDevice rfm69Hcw;

		public Rfm69HcwDevice(int chipSelectPin)
		{
			SpiController spiController = SpiController.GetDefaultAsync().AsTask().GetAwaiter().GetResult();
			var settings = new SpiConnectionSettings(chipSelectPin)
			{
				ClockFrequency = 500000,
				Mode = SpiMode.Mode0,
			};

			rfm69Hcw = spiController.GetDevice(settings);
		}

		public Byte RegisterReadByte(byte registerAddress)
		{
			byte[] writeBuffer = new byte[] { registerAddress };
			byte[] readBuffer = new byte[1];
			Debug.Assert(rfm69Hcw != null);

			rfm69Hcw.TransferSequential(writeBuffer, readBuffer);

			return readBuffer[0];
		}
	}

	public sealed class StartupTask : IBackgroundTask
	{
		private const int ChipSelectLine = 0;
		private Rfm69HcwDevice rfm69HcwDevice = new Rfm69HcwDevice(ChipSelectLine);

		public void Run(IBackgroundTaskInstance taskInstance)
		{

			while (true)
			{
				for (byte registerIndex = 0; registerIndex <= 0x4F; registerIndex++)
				{
					byte registerValue = rfm69HcwDevice.RegisterReadByte(registerIndex);

					Debug.WriteLine("Register 0x{0:x2} - Value 0X{1:x2} - Bits {2}", registerIndex, registerValue, Convert.ToString(registerValue, 2).PadLeft(8, '0'));
				}

				Task.Delay(10000).Wait();
			}
		}
	}
}
