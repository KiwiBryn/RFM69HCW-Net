/*
    Copyright ® 2019 June devMobile Software, All Rights Reserved

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
namespace devMobile.IoT.Rfm69Hcw.RegisterReadAndWrite
{
	using System;
	using System.Diagnostics;
	using System.Runtime.InteropServices.WindowsRuntime;
	using System.Threading.Tasks;
	using Windows.ApplicationModel.Background;
	using Windows.Devices.Spi;
	using Windows.Devices.Gpio;

	public sealed class Rfm69HcwDevice
	{
		private SpiDevice Rfm69Hcw;
		private const byte RegisterAddressReadMask = 0X7f;
		private const byte RegisterAddressWriteMask = 0x80;

		public Rfm69HcwDevice(int chipSelectPin, int resetPin)
		{
			SpiController spiController = SpiController.GetDefaultAsync().AsTask().GetAwaiter().GetResult();
			var settings = new SpiConnectionSettings(chipSelectPin)
			{
				ClockFrequency = 500000,
				Mode = SpiMode.Mode0,
			};

			// Factory reset pin configuration
			GpioController gpioController = GpioController.GetDefault();
			GpioPin resetGpioPin = gpioController.OpenPin(resetPin);
			resetGpioPin.SetDriveMode(GpioPinDriveMode.Output);
			resetGpioPin.Write(GpioPinValue.High);
			Task.Delay(1);
			resetGpioPin.Write(GpioPinValue.Low);
			Task.Delay(10);

			Rfm69Hcw = spiController.GetDevice(settings);
		}

		public Byte RegisterReadByte(byte address)
		{
			byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
			byte[] readBuffer = new byte[1];
			Debug.Assert(Rfm69Hcw != null);

			Rfm69Hcw.TransferSequential(writeBuffer, readBuffer);

			return readBuffer[0];
		}

		public ushort RegisterReadWord(byte address)
		{
			byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
			byte[] readBuffer = new byte[2];
			Debug.Assert(Rfm69Hcw != null);

			Rfm69Hcw.TransferSequential(writeBuffer, readBuffer);

			return (ushort)(readBuffer[1] + (readBuffer[0] << 8));
		}

		public byte[] RegisterRead(byte address, int length)
		{
			byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
			byte[] readBuffer = new byte[length];
			Debug.Assert(Rfm69Hcw != null);

			Rfm69Hcw.TransferSequential(writeBuffer, readBuffer);

			return readBuffer;
		}

		public void RegisterWriteByte(byte address, byte value)
		{
			byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, value };
			Debug.Assert(Rfm69Hcw != null);

			Rfm69Hcw.Write(writeBuffer);
		}

		public void RegisterWriteWord(byte address, ushort value)
		{
			byte[] valueBytes = BitConverter.GetBytes(value);
			byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, valueBytes[0], valueBytes[1] };
			Debug.Assert(Rfm69Hcw != null);

			Rfm69Hcw.Write(writeBuffer);
		}

		public void RegisterWrite(byte address, [ReadOnlyArray()] byte[] bytes)
		{
			byte[] writeBuffer = new byte[1 + bytes.Length];
			Debug.Assert(Rfm69Hcw != null);

			Array.Copy(bytes, 0, writeBuffer, 1, bytes.Length);
			writeBuffer[0] = address |= RegisterAddressWriteMask;

			Rfm69Hcw.Write(writeBuffer);
		}

		public void RegisterDump()
		{
			Debug.WriteLine("Register dump");
			for (byte registerIndex = 0; registerIndex <= 0x3D; registerIndex++)
			{
				byte registerValue = this.RegisterReadByte(registerIndex);

				Debug.WriteLine("Register 0x{0:x2} - Value 0X{1:x2} - Bits {2}", registerIndex, registerValue, Convert.ToString(registerValue, 2).PadLeft(8, '0'));
			}
		}
	}


	public sealed class StartupTask : IBackgroundTask
	{
		private const int ChipSelectLine = 1;
		private const int ResetLine = 25;
		private Rfm69HcwDevice rfm69Device = new Rfm69HcwDevice(ChipSelectLine, ResetLine);

		const double RH_RF6M9HCW_FXOSC = 32000000.0;
		const double RH_RFM69HCW_FSTEP = RH_RF6M9HCW_FXOSC / 524288.0;

		public void Run(IBackgroundTaskInstance taskInstance)
		{
			while (true)
			{
				rfm69Device.RegisterDump();

				Debug.WriteLine("Read RegOpMode (read byte)");
				Byte regOpMode = rfm69Device.RegisterReadByte(0x1);
				Debug.WriteLine("Reg OpMode 0x{0:x2}", regOpMode);

				byte[] bytes = BitConverter.GetBytes((long)(868000000.0/ RH_RFM69HCW_FSTEP));

				Debug.WriteLine("Byte Hex 0x{0:x2} 0x{1:x2} 0x{2:x2} 0x{3:x2}", bytes[0], bytes[1], bytes[2], bytes[3]);

				rfm69Device.RegisterWriteByte(0x07, bytes[2]);
				rfm69Device.RegisterWriteByte(0x08, bytes[1]);
				rfm69Device.RegisterWriteByte(0x09, bytes[0]);

				rfm69Device.RegisterDump();

				Task.Delay(30000).Wait();
			}
		}
	}
}

