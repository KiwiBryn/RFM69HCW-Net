/*
    Copyright ® 2019 July devMobile Software, All Rights Reserved

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
namespace devMobile.IoT.Rfm69Hcw.EnumAndMasks
{
	using System;
	using System.Diagnostics;
	using System.Text;
	using System.Threading.Tasks;
	using Windows.ApplicationModel.Background;
	using Windows.Devices.Gpio;

	sealed class Rfm69HcwDevice
	{
		// Registers from SemTech/HopeRFM69 Datasheet
		enum Registers : byte
		{
			MinValue = RegOpMode,

			RegFifo = 0x00,
			RegOpMode = 0x01,
			RegDataModul = 0x02,
			RegBitrateMsb = 0x03,
			RegBitrateLsb = 0x04,
			RegFdevMsb = 0x05,
			RegFdevLsb = 0x06,
			RegFrMsb = 0x07,
			RegFrMid = 0x08,
			RegFrLsb = 0x09,
			RegOsc1 = 0x0A,
			RegAfcCtrl = 0x0B,
			RegListen1 = 0x0D,
			RegListen2 = 0x0E,
			RegListen3 = 0x0F,
			RegVersion = 0x10,
			RegPaLevel = 0x11,
			RegPaRamp = 0x12,
			RegOcp = 0x13,
			RegLna = 0x18,
			RegRxBw = 0x19,
			RegAfcBw = 0x1A,
			RegOokPeak = 0x1B,
			RegOokAvg = 0x1C,
			RegOokFix = 0x1D,
			RegAfcFei = 0x1E,
			RegAfcMsb = 0x1F,
			RegAfcLsb = 0x20,
			RegFeiMsb = 0x21,
			RegFeiLsb = 0x22,
			RegRssiConfig = 0x23,
			RegRssiValue = 0x24,
			RegDioMapping1 = 0x25,
			RegDioMapping2 = 0x26,
			RegIrqFlags1 = 0x27,
			RegIrqFlags2 = 0x28,
			RegRssiThresh = 0x29,
			RegRxTimeout1 = 0x2A,
			RegRxTimeout2 = 0x2B,
			RegPreambleMsb = 0x2C,
			RegPreambleLsb = 0x2D,
			RegSyncConfig = 0x2E,
			RegSyncValue1 = 0x2F,
			RegSyncValue2 = 0x30,
			RegSyncValue3 = 0x31,
			RegSyncValue4 = 0x32,
			RegSyncValue5 = 0x33,
			RegSyncValue6 = 0x34,
			RegSyncValue7 = 0x35,
			RegSyncValue8 = 0x36,
			RegPacketConfig1 = 0x37,
			RegPayloadLength = 0x38,
			RegNodeAdrs = 0x39,
			RegBroadcastAdrs = 0x3A,
			RegAutoModes = 0x3B,
			RegFifoThresh = 0x3C,
			RegPacketConfig2 = 0x3D,
			RegAesKey1 = 0x3E,
			RegAesKey2 = 0x3F,
			RegAesKey3 = 0x40,
			RegAesKey4 = 0x41,
			RegAesKey5 = 0x42,
			RegAesKey6 = 0x43,
			RegAesKey7 = 0x44,
			RegAesKey8 = 0x45,
			RegAesKey9 = 0x46,
			RegAesKey10 = 0x47,
			RegAesKey11 = 0x48,
			RegAesKey12 = 0x49,
			RegAesKey13 = 0x4A,
			RegAesKey14 = 0x4B,
			RegAesKey15 = 0x4C,
			RegAesKey16 = 0x4D,
			RegTemp1 = 0x4E,
			RegTemp2 = 0x4F,
			RegTestLna = 0x58,
			RegTestPa1 = 0x5A,
			RegTestPa2 = 0x5C,
			RegTestDagc = 0x6F,
			RegTestAfc = 0x71,

			MaxValue = RegAesKey16,
		}

		// RegOp Mode flags

		// RegVersion default expected valued
		const byte RegVersionValueExpected = 0x24;

		private GpioPin InterruptGpioPin = null;
		public RegisterManager RegisterManager = null; // Future refactor this will be made private

		public Rfm69HcwDevice(ChipSelectPin chipSelectPin, int resetPin, int interruptPin)
		{
			RegisterManager = new RegisterManager(chipSelectPin);

			// Check that SX1231 chip is present
			Byte regVersionValue = RegisterManager.ReadByte((byte)Registers.RegVersion);
			if (regVersionValue != RegVersionValueExpected)
			{
				throw new ApplicationException("Semtech SX1231 not found");
			}

			GpioController gpioController = GpioController.GetDefault();

			// Factory reset pin configuration
			GpioPin resetGpioPin = gpioController.OpenPin(resetPin);
			resetGpioPin.SetDriveMode(GpioPinDriveMode.Output);
			resetGpioPin.Write(GpioPinValue.High);
			Task.Delay(100);
			resetGpioPin.Write(GpioPinValue.Low);
			Task.Delay(10);

			// Interrupt pin for RX message & TX done notification 
			InterruptGpioPin = gpioController.OpenPin(interruptPin);
			resetGpioPin.SetDriveMode(GpioPinDriveMode.Input);

			InterruptGpioPin.ValueChanged += InterruptGpioPin_ValueChanged;
		}

		private void InterruptGpioPin_ValueChanged(GpioPin sender, GpioPinValueChangedEventArgs args)
		{
			if (args.Edge != GpioPinEdge.RisingEdge)
			{
				return;
			}

			byte irqFlags = RegisterManager.ReadByte(0x28); // RegIrqFlags2
			Debug.WriteLine("{0:HH:mm:ss.fff} RegIrqFlags {1}", DateTime.Now, Convert.ToString((byte)irqFlags, 2).PadLeft(8, '0'));
			if ((irqFlags & 0b00000100) == 0b00000100)  // PayLoadReady set
			{
				// Read the length of the buffer
				byte numberOfBytes = RegisterManager.ReadByte(0x0);

				// Allocate buffer for message
				byte[] messageBytes = new byte[numberOfBytes];

				for (int i = 0; i < numberOfBytes; i++)
				{
					messageBytes[i] = RegisterManager.ReadByte(0x00); // RegFifo
				}

				string messageText = UTF8Encoding.UTF8.GetString(messageBytes);
				Debug.WriteLine("{0:HH:mm:ss} Received {1} byte message {2}", DateTime.Now, messageBytes.Length, messageText);
			}

			if ((irqFlags & 0b00001000) == 0b00001000)  // PacketSent set
			{
				RegisterManager.WriteByte(0x01, 0b00010000); // RegOpMode set ReceiveMode
				Debug.WriteLine("{0:HH:mm:ss.fff} Transmit-Done", DateTime.Now);
			}
		}

		public void RegisterDump()
		{
			RegisterManager.Dump((byte)Registers.MinValue, (byte)Registers.MaxValue);
		}
	}


	public sealed class StartupTask : IBackgroundTask
	{
		private const int ResetPin = 25;
		private const int InterruptPin = 22;
		private Rfm69HcwDevice rfm69Device = new Rfm69HcwDevice(ChipSelectPin.CS1, ResetPin, InterruptPin);

		const double RH_RF6M9HCW_FXOSC = 32000000.0;
		const double RH_RFM69HCW_FSTEP = RH_RF6M9HCW_FXOSC / 524288.0;

		public void Run(IBackgroundTaskInstance taskInstance)
		{
			rfm69Device.RegisterDump();

			// regOpMode standby
			rfm69Device.RegisterManager.WriteByte(0x01, 0b00000100);

			// BitRate MSB/LSB
			rfm69Device.RegisterManager.WriteByte(0x03, 0x34);
			rfm69Device.RegisterManager.WriteByte(0x04, 0x00);

			// Frequency deviation
			rfm69Device.RegisterManager.WriteByte(0x05, 0x02);
			rfm69Device.RegisterManager.WriteByte(0x06, 0x3d);

			// Calculate the frequency accoring to the datasheett
			byte[] bytes = BitConverter.GetBytes((uint)(915000000.0 / RH_RFM69HCW_FSTEP));
			Debug.WriteLine("Byte Hex 0x{0:x2} 0x{1:x2} 0x{2:x2} 0x{3:x2}", bytes[0], bytes[1], bytes[2], bytes[3]);
			rfm69Device.RegisterManager.WriteByte(0x07, bytes[2]);
			rfm69Device.RegisterManager.WriteByte(0x08, bytes[1]);
			rfm69Device.RegisterManager.WriteByte(0x09, bytes[0]);

			// RegRxBW
			rfm69Device.RegisterManager.WriteByte(0x19, 0x2a);

			// RegDioMapping1
			rfm69Device.RegisterManager.WriteByte(0x26, 0x01);

			// Setup preamble length to 16 (default is 3) RegPreambleMsb RegPreambleLsb
			rfm69Device.RegisterManager.WriteByte(0x2C, 0x0);
			rfm69Device.RegisterManager.WriteByte(0x2D, 0x10);

			// RegSyncConfig Set the Sync length and byte values SyncOn + 3 custom sync bytes
			rfm69Device.RegisterManager.WriteByte(0x2e, 0x90);

			// RegSyncValues1 thru RegSyncValues3
			rfm69Device.RegisterManager.WriteByte(0x2f, 0xAA);
			rfm69Device.RegisterManager.WriteByte(0x30, 0x2D);
			rfm69Device.RegisterManager.WriteByte(0x31, 0xD4);

			// RegPacketConfig1 Variable length with CRC on
			rfm69Device.RegisterManager.WriteByte(0x37, 0x90);

			rfm69Device.RegisterDump();

			while (true)
			{
				// Standby mode while loading message into FIFO
				rfm69Device.RegisterManager.WriteByte(0x01, 0b00000100);

				byte[] messageBuffer = UTF8Encoding.UTF8.GetBytes("hello world " + DateTime.Now.ToLongTimeString());
				rfm69Device.RegisterManager.WriteByte(0x0, (byte)messageBuffer.Length);
				rfm69Device.RegisterManager.Write(0x0, messageBuffer);

				// Transmit mode once FIFO loaded
				rfm69Device.RegisterManager.WriteByte(0x01, 0b00001100);

				Debug.WriteLine("{0:HH:mm:ss.fff} Send-Done", DateTime.Now);

				Task.Delay(5000).Wait();
			}
		}
	}
}
