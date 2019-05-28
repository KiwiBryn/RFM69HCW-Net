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
namespace devMobile.IoT.Rfm69Hcw.RegisterReadAndWrite
{
	using System;
	using System.Diagnostics;
	using System.Runtime.InteropServices.WindowsRuntime;
	using System.Threading.Tasks;
	using Windows.ApplicationModel.Background;
	using Windows.Devices.Spi;
	using Windows.Devices.Gpio;

	public sealed class Rfm9XDevice
	{
		private SpiDevice Rfm9XLoraModem;
		private GpioPin ChipSelectGpioPin;
		private const byte RegisterAddressReadMask = 0X7f;
		private const byte RegisterAddressWriteMask = 0x80;

		public Rfm9XDevice(int chipSelectPin, int resetPin)
		{
			SpiController spiController = SpiController.GetDefaultAsync().AsTask().GetAwaiter().GetResult();
			var settings = new SpiConnectionSettings(0)
			{
				ClockFrequency = 500000,
				Mode = SpiMode.Mode0,
			};

			// Chip select pin configuration
			GpioController gpioController = GpioController.GetDefault();
			ChipSelectGpioPin = gpioController.OpenPin(chipSelectPin);
			ChipSelectGpioPin.SetDriveMode(GpioPinDriveMode.Output);
			ChipSelectGpioPin.Write(GpioPinValue.High);

			// Factory reset pin configuration
			GpioPin resetGpioPin = gpioController.OpenPin(resetPin);
			resetGpioPin.SetDriveMode(GpioPinDriveMode.Output);
			resetGpioPin.Write(GpioPinValue.Low);
			Task.Delay(10);
			resetGpioPin.Write(GpioPinValue.High);
			Task.Delay(10);

			Rfm9XLoraModem = spiController.GetDevice(settings);
		}

		public Byte RegisterReadByte(byte address)
		{
			byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
			byte[] readBuffer = new byte[1];
			Debug.Assert(Rfm9XLoraModem != null);

			ChipSelectGpioPin.Write(GpioPinValue.Low);
			Rfm9XLoraModem.Write(writeBuffer);
			Rfm9XLoraModem.Read(readBuffer);
			ChipSelectGpioPin.Write(GpioPinValue.High);

			return readBuffer[0];
		}

		public ushort RegisterReadWord(byte address)
		{
			byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
			byte[] readBuffer = new byte[2];
			Debug.Assert(Rfm9XLoraModem != null);

			ChipSelectGpioPin.Write(GpioPinValue.Low);
			Rfm9XLoraModem.Write(writeBuffer);
			Rfm9XLoraModem.Read(readBuffer);
			ChipSelectGpioPin.Write(GpioPinValue.High);

			return (ushort)(readBuffer[1] + (readBuffer[0] << 8));
		}

		public byte[] RegisterRead(byte address, int length)
		{
			byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
			byte[] readBuffer = new byte[length];
			Debug.Assert(Rfm9XLoraModem != null);

			ChipSelectGpioPin.Write(GpioPinValue.Low);
			Rfm9XLoraModem.Write(writeBuffer);
			Rfm9XLoraModem.Read(readBuffer);
			ChipSelectGpioPin.Write(GpioPinValue.High);

			return readBuffer;
		}

		public void RegisterWriteByte(byte address, byte value)
		{
			byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, value };
			Debug.Assert(Rfm9XLoraModem != null);

			ChipSelectGpioPin.Write(GpioPinValue.Low);
			Rfm9XLoraModem.Write(writeBuffer);
			ChipSelectGpioPin.Write(GpioPinValue.High);
		}

		public void RegisterWriteWord(byte address, ushort value)
		{
			byte[] valueBytes = BitConverter.GetBytes(value);
			byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, valueBytes[0], valueBytes[1] };
			Debug.Assert(Rfm9XLoraModem != null);

			ChipSelectGpioPin.Write(GpioPinValue.Low);
			Rfm9XLoraModem.Write(writeBuffer);
			ChipSelectGpioPin.Write(GpioPinValue.High);
		}

		public void RegisterWrite(byte address, [ReadOnlyArray()] byte[] bytes)
		{
			byte[] writeBuffer = new byte[1 + bytes.Length];
			Debug.Assert(Rfm9XLoraModem != null);

			Array.Copy(bytes, 0, writeBuffer, 1, bytes.Length);
			writeBuffer[0] = address |= RegisterAddressWriteMask;

			ChipSelectGpioPin.Write(GpioPinValue.Low);
			Rfm9XLoraModem.Write(writeBuffer);
			ChipSelectGpioPin.Write(GpioPinValue.High);
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
		private const int ChipSelectLine = 25;
		private const int ResetLine = 17;
		private Rfm9XDevice rfm9XDevice = new Rfm9XDevice(ChipSelectLine, ResetLine);

		public void Run(IBackgroundTaskInstance taskInstance)
		{
			while (true)
			{
				rfm9XDevice.RegisterDump();

				Debug.Print("Read RegOpMode (read byte)");
				Byte regOpMode = rfm9XDevice.RegisterReadByte(0x1);
				Debug.WriteLine("Preamble 0x{0:x2}", regOpMode);

				Debug.Print("Set LoRa mode and sleep mode (write byte)");
				rfm9XDevice.RegisterWriteByte(0x01, 0b10000000); // 

				Debug.Print("Read the preamble (read word)");
				ushort preamble = rfm9XDevice.RegisterReadWord(0x20);
				Debug.WriteLine("Preamble 0x{0:x2} - Bits {1}", preamble, Convert.ToString(preamble, 2).PadLeft(16, '0'));

				Debug.WriteLine("Set the preamble to 0x80 (write word)");
				rfm9XDevice.RegisterWriteWord(0x20, 0x80);

				Debug.WriteLine("Read the centre frequency (read byte array)");
				byte[] frequencyReadBytes = rfm9XDevice.RegisterRead(0x07, 3);
				Debug.WriteLine("Frequency Msb 0x{0:x2} Mid 0x{1:x2} Lsb 0x{2:x2}", frequencyReadBytes[0], frequencyReadBytes[1], frequencyReadBytes[2]);

				Debug.WriteLine("Set the centre frequency to 916MHz ( write byte array)");
				byte[] frequencyWriteBytes = { 0xE4, 0xC0, 0x00 };
				rfm9XDevice.RegisterWrite(0x07, frequencyWriteBytes);

				rfm9XDevice.RegisterDump();

				Task.Delay(30000).Wait();
			}
		}
	}
}

