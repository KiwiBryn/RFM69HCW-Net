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
			RegFrfMsb = 0x07,
			RegFrfMid = 0x08,
			RegFrfLsb = 0x09,
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

		// RegOpMode mode flags
		private const byte RegOpModeSequencerOff = 0b10000000;
		private const byte RegOpModeListenOn = 0b01000000;
		private const byte RegOpModeListenOff = 0b00000000;
		private const byte RegOpModeListenAbort = 0b01000000;

		[Flags]
		public enum RegOpModeMode : byte
		{
			Sleep = 0b00000000,
			StandBy = 0b00000100,
			FrequencySynthesisTx = 0b00001000,
			Transmit = 0b00001100,
			Receive = 0b00010000,
		};

		// BitRate configuration from table9 in datasheet RegBitrateMsb, RegBitrateLsb
		public enum BitRate : ushort
		{
			bps1K2 = 0x682B,
			bps2K4 = 0x3415,
			bps4K8 = 0x1A0B,
			bps9K6 = 0x0D05,
			bps19K2 = 0x0683,
			bps38K4 = 0x0341,
			bps76K8 = 0x01A1,
			bps153K6 = 0x00D0,
			bps57K6 = 0x022Cc,
			bps115K2 = 0x0116,
			bps12K5 = 0x0A00,
			bps25K = 0x0500,
			bps100k = 0x0140,
			bps150K = 0x00D5,
			bps200K = 0x00A0,
			bps250K = 0x0080,
			bps300k = 0x006B
		};
		const BitRate BitRateDefault = BitRate.bps4K8;

		// Frequency deviation configuration RegFdevMsb, RegFdevLsb 
		const ushort frequencyDeviationDefault = 0x0052;

		// Frequency configuration magic numbers from Semtech SX1231 specs RegFrMsb, RegFrMid, RegFrLsb
		private const double RH_RF6M9HCW_FXOSC = 32000000.0;
		private const double RH_RFM69HCW_FSTEP = RH_RF6M9HCW_FXOSC / 524288.0;
		public const double FrequencyDefault = 915000000.0;

		// RegListen1 settings
		public enum ListenModeIdleResolution : byte
		{
			Reserved = 0b00000000,
			IdleTime64us = 0b01000000,
			IdleTime4_1ms = 0b10000000,
			IdleTime262ms = 0b11000000,
		}
		const ListenModeIdleResolution ListenModeIdleResolutionDefault = ListenModeIdleResolution.IdleTime4_1ms;

		public enum ListenModeRXTime : byte
		{
			Reserved = 0b00000000,
			IdleTime64us = 0b00010000,
			IdleTime4_1ms = 0b00100000,
			IdleTime262ms = 0b00110000,
		}
		const ListenModeRXTime ListenModeRXTimeDefault = ListenModeRXTime.IdleTime64us;

		public enum ListenModeCrieria : byte
		{
			RssiThreshold = 0b00000000,
			RssiThresholdAndSyncAddressMatched = 0b00001000
		}
		const ListenModeCrieria ListenModeCrieriaDefault = ListenModeCrieria.RssiThreshold;

		public enum ListenModeEnd : byte
		{
			StayInRXMode = 0b00000000,
			StayInRxModeUntilPayloadReady = 0b00000010,
			StayInRxModeUntilPayloadReadyOrTimeoutInterrupt = 0b00000100,
			Reserved = 0b00000110,
		}
		const ListenModeEnd ListenModeEndDefault = ListenModeEnd.StayInRxModeUntilPayloadReadyOrTimeoutInterrupt;

		// RegListen2 
		const byte ListenCoefficientIdleDefault = 0xf5;

		// RegListen3 
		const byte ListenCoefficientReceiveDefault = 0x20;

		// RegVersion default expected value
		const byte RegVersionValueExpected = 0x24;

		// RegPaLevel 
		const bool pa0OnDefault = true;
		const bool pa1OnDefaut = false;
		const bool pa2OnDefault = false;
		const byte OutputpowerDefault = 0b00011111;

		// RegPaRamp 
		public enum PaRamp : byte
		{
			Period3_4ms = 0b00000000,
			Period2ms =   0b00000001,
			Period1ms =   0b00000010,
			Period500us = 0b00000011,
			Period250us = 0b00000100,
			Period125us = 0b00000101,
			Period100us = 0b00000110,
			Period62us =  0b00000111,
			Period50us =  0b00001000,
			Period40us =  0b00001001,
			Period31us =  0b00001010,
			Period25us =  0b00001011,
			Period20us =  0b00001100,
			Period15us =  0b00001101,
			Period12us =  0b00001110,
			Period10us =  0b00001111,
		}
		const PaRamp PaRampDefault = PaRamp.Period40us;

		// RegOcp values
		const bool OcpOnDefault = true;
		const byte OcpTrimDefault = 0b0001010;

		// RegLna
		public enum LnaZin : byte
		{
			Impedance50Ohms = 0b00000000,
			Impedance200Ohms = 0b10000000
		}
		const LnaZin LnaZinDefault = LnaZin.Impedance50Ohms;

		public enum LnaCurrentGain : byte
		{
			Manual = 0b00100000,
			Agc = 0b00000000,
		}
		const LnaCurrentGain LnaCurrentGainDefault = LnaCurrentGain.Manual;

		public enum LnaGainSelect : byte
		{
			AGC = 0b00000000,
			G1 = 0b00000001,
			G2 = 0b00000010,
			G3 = 0b00000011,
			G4 = 0b00000100,
			G5 = 0b00000101,
			G6 = 0b00000110,
			Reserved = 0b00000111,
		} 
		const LnaGainSelect LnaGainSelectDefault = LnaGainSelect.AGC;

		// RegRxBw
		const byte DccFrequencyDefault = 0b01000000;

		public enum RxBwMant
		{
			RxBwMant16 = 0b00000000,
			RxBwMant20 = 0b00001000,
			RxBwMant24 = 0b00010000,
			Reserved = 0b00011000
		}
		const RxBwMant RxBwMantDefault = RxBwMant.RxBwMant24;
		const byte RxBwExpDefault = 0b00000101;

		// RegAfcBW
		const byte DccFreqAfcDefault = 0b10000000;
		const byte RxBwMantAfcDefault = 0b00001000;
		const byte RxBwExpAfcDefault = 0b00000011;

		// Ignored these registers
		// RegOokPeak, RegOokAvg, RegOokFix, RegAfcFei, 
		// RegAfcMsb, RegAfcLsb
		// RegFeiMsb, RegFeiLsb

		// RegDioMapping1 & RegDioMapping1 CpntinuousMode Table 21/ Packet Mode Table 22 pg48

		// RegIrqFlags1
		[Flags]
		public enum RegIrqFlags1 : byte
		{
			ModeReady = 0b10000000,
			RxReady =   0b01000000,
			TxReady =	0b00100000,
			PllLock =	0b00010000,
			Rssi =		0b00001000,
			Timeout =	0b00000100,
			AutoMode =  0b00000010,
			SynAddressMatch = 0b00000001
		}

		// RegIrqFlags2
		[Flags]
		public enum RegIrqFlags2 : byte
		{
			FifoFull = 0b10000000,
			FifoNotEmpty = 0b01000000,
			FifoLevel = 0b00100000,
			FifoOverrun = 0b00010000,
			PacketSent = 0b00001000,
			PayloadReady = 0b00000100,
			CrcOk = 0b00000010,
			// Unused = 0b00000001
		}

		// RegPreambleMsb
		const ushort PreambleSizeDefault = 0x03;

		// Hardware configuration support
		private RegOpModeMode RegOpModeModeCurrent = RegOpModeMode.Sleep;
		private GpioPin InterruptGpioPin = null;
		private GpioPin ResetGpioPin = null;
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
			ResetGpioPin = gpioController.OpenPin(resetPin);
			ResetGpioPin.SetDriveMode(GpioPinDriveMode.Output);
	
			// Interrupt pin for RX message & TX done notification 
			InterruptGpioPin = gpioController.OpenPin(interruptPin);
			ResetGpioPin.SetDriveMode(GpioPinDriveMode.Input);

			InterruptGpioPin.ValueChanged += InterruptGpioPin_ValueChanged;
		}

		public void RegisterDump()
		{
			RegisterManager.Dump((byte)Registers.MinValue, (byte)Registers.MaxValue);
		}

		
		public void SetMode(RegOpModeMode mode)
		{
			byte regOpModeValue = (byte)mode;

			RegisterManager.WriteByte((byte)Registers.RegOpMode, regOpModeValue);
		}


		public void Initialise(RegOpModeMode modeAfterInitialise,
			BitRate bitRate = BitRateDefault,
			ushort frequencyDeviation = frequencyDeviationDefault,
			double frequency = FrequencyDefault,
			ListenModeIdleResolution listenModeIdleResolution = ListenModeIdleResolutionDefault, ListenModeRXTime listenModeRXTime = ListenModeRXTimeDefault, ListenModeCrieria listenModeCrieria = ListenModeCrieriaDefault, ListenModeEnd listenModeEnd = ListenModeEndDefault,
			byte listenCoefficientIdle = ListenCoefficientIdleDefault,
			byte listenCoefficientReceive = ListenCoefficientReceiveDefault,
			bool pa0On = pa0OnDefault, bool pa1On = pa1OnDefaut, bool pa2On = pa2OnDefault, byte outputpower = OutputpowerDefault,
			PaRamp paRamp = PaRampDefault,
			bool ocpOn = OcpOnDefault, byte ocpTrim = OcpTrimDefault,
			LnaZin lnaZin = LnaZinDefault, LnaCurrentGain lnaCurrentGain = LnaCurrentGainDefault, LnaGainSelect lnaGainSelect = LnaGainSelectDefault,
			byte dccFrequency = DccFrequencyDefault, RxBwMant rxBwMant = RxBwMantDefault, byte RxBwExp = RxBwExpDefault,
			byte dccFreqAfc = DccFreqAfcDefault, byte rxBwMantAfc = RxBwMantAfcDefault, byte bxBwExpAfc = RxBwExpAfcDefault,
			ushort preambleSize = PreambleSizeDefault
			)
		{
			RegOpModeModeCurrent = modeAfterInitialise;

			// Strobe Reset pin briefly to factory reset SX1231 chip
			ResetGpioPin.Write(GpioPinValue.High);
			Task.Delay(100);
			ResetGpioPin.Write(GpioPinValue.Low);
			Task.Delay(10);

			// Put the device into sleep mode so registers can be changed
			SetMode(RegOpModeMode.Sleep);
			
			// RegDataModul ignored

			// RegBitrateMsb, RegBitrateLsb
			if( bitRate != BitRateDefault)
			{
				byte[] bytes = BitConverter.GetBytes((ushort)bitRate);
				RegisterManager.WriteByte((byte)Registers.RegBitrateMsb, bytes[1]);
				RegisterManager.WriteByte((byte)Registers.RegBitrateLsb, bytes[0]);
			}

			// RegFdevMsb, RegFdevLsb
			if( frequencyDeviation != frequencyDeviationDefault)
			{
				byte[] bytes = BitConverter.GetBytes((ushort)frequencyDeviation);
				RegisterManager.WriteByte((byte)Registers.RegFdevMsb, bytes[1]);
				RegisterManager.WriteByte((byte)Registers.RegFdevLsb, bytes[0]);
			}

			// Configure RF Carrier frequency RegFrMsb, RegFrMid, RegFrLsb
			if (frequency != FrequencyDefault)
			{
				byte[] bytes = BitConverter.GetBytes((long)(frequency / RH_RFM69HCW_FSTEP));
				RegisterManager.WriteByte((byte)Registers.RegFrfMsb, bytes[2]);
				RegisterManager.WriteByte((byte)Registers.RegFrfMid, bytes[1]);
				RegisterManager.WriteByte((byte)Registers.RegFrfLsb, bytes[0]);
			}

			if ((listenModeIdleResolution != ListenModeIdleResolutionDefault) ||
				 (listenModeRXTime != ListenModeRXTimeDefault) ||
				 (listenModeCrieria != ListenModeCrieriaDefault ) ||
				 (listenModeEnd != ListenModeEndDefault))
			{
				byte regListen1Value = (byte)listenModeIdleResolution;

				regListen1Value |= (byte)listenModeRXTime;
				regListen1Value |= (byte)listenModeCrieria;
				regListen1Value |= (byte)listenModeEnd;

				RegisterManager.WriteByte((byte)Registers.RegListen1, regListen1Value);
			}

			if ( listenCoefficientIdle != ListenCoefficientIdleDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegListen2, listenCoefficientIdle);
			}

			if ( listenCoefficientReceive != ListenCoefficientReceiveDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegListen3, listenCoefficientReceive);
			}

			if ((pa0On != pa0OnDefault) || 
				 (pa1On != pa1OnDefaut) ||  
				 (pa2On != pa2OnDefault) || 
				 (outputpower != OutputpowerDefault))
			{
				byte regPaLevelValue = outputpower;

				if (pa0On)
				{
					regPaLevelValue |= 0b10000000;
				}
				if (pa1On)
				{
					regPaLevelValue |= 0b01000000;
				}
				if (pa2On)
				{
					regPaLevelValue |= 0b00100000;
				}
				RegisterManager.WriteByte((byte)Registers.RegPaLevel, regPaLevelValue);
			}

			// Set RegOcp if any of the settings not defaults
			if ((ocpOn != OcpOnDefault) || 
				 (ocpTrim != OcpTrimDefault))
			{
				byte regOcpValue = ocpTrim;

				if (ocpOn)
				{
					regOcpValue |= 0b00010000;
				}
				RegisterManager.WriteByte((byte)Registers.RegOcp, regOcpValue);
			}

			// regLnaValue
			if ((lnaZin != LnaZinDefault) ||
				 (lnaCurrentGain != LnaCurrentGainDefault) ||
				 (lnaGainSelect != LnaGainSelectDefault))
			{
				byte regLnaValue = (byte)lnaZin;

				regLnaValue |= (byte)lnaCurrentGain;
				regLnaValue |= (byte)lnaGainSelect;

				RegisterManager.WriteByte((byte)Registers.RegLna, regLnaValue);
			}

			// RegRxBw
			if ((dccFrequency != DccFrequencyDefault) ||
				 (rxBwMant != RxBwMantDefault) ||
				 (RxBwExp != RxBwExpDefault))
			{
				byte RegRxBwValue = (byte)DccFrequencyDefault;

				RegRxBwValue |= (byte)RxBwMantDefault;
				RegRxBwValue |= RxBwExpDefault;

				RegisterManager.WriteByte((byte)Registers.RegRxBw, RegRxBwValue);
			}

			// RegAfcBw
			if (( dccFreqAfc != DccFreqAfcDefault ) ||
				 ( rxBwMantAfc != RxBwMantAfcDefault ) || 
				 ( bxBwExpAfc != RxBwExpAfcDefault))
			{
				byte regAfcBwValue = dccFreqAfc;

				regAfcBwValue |= rxBwMantAfc;
				regAfcBwValue |= bxBwExpAfc;

				RegisterManager.WriteByte((byte)Registers.RegAfcBw, regAfcBwValue);
			}

			// RegPreambleMsb RegPreambleLsb
			if (preambleSize != PreambleSizeDefault)
			{
				byte[] bytes = BitConverter.GetBytes((ushort)preambleSize);
				RegisterManager.WriteByte((byte)Registers.RegPreambleMsb, bytes[1]);
				RegisterManager.WriteByte((byte)Registers.RegPreambleLsb, bytes[0]);
			}

			// Configure RegOpMode before returning
			SetMode(modeAfterInitialise);
		}

		private void InterruptGpioPin_ValueChanged(GpioPin sender, GpioPinValueChangedEventArgs args)
		{
			if (args.Edge != GpioPinEdge.RisingEdge)
			{
				return;
			}

			RegIrqFlags2 irqFlags = (RegIrqFlags2)RegisterManager.ReadByte((byte)Registers.RegIrqFlags2); // RegIrqFlags2
			Debug.WriteLine("{0:HH:mm:ss.fff} RegIrqFlags {1}", DateTime.Now, Convert.ToString((byte)irqFlags, 2).PadLeft(8, '0'));
			if ((irqFlags & RegIrqFlags2.PayloadReady) == RegIrqFlags2.PayloadReady)  // PayLoadReady set
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

			if ((irqFlags & RegIrqFlags2.PacketSent) == RegIrqFlags2.PacketSent)  // PacketSent set
			{
				RegisterManager.WriteByte(0x01, 0b00010000); // RegOpMode set ReceiveMode
				Debug.WriteLine("{0:HH:mm:ss.fff} Transmit-Done", DateTime.Now);
			}
		}
	}


	public sealed class StartupTask : IBackgroundTask
	{
		private const int ResetPin = 25;
		private const int InterruptPin = 22;
		private Rfm69HcwDevice rfm69Device = new Rfm69HcwDevice(ChipSelectPin.CS1, ResetPin, InterruptPin);

		public void Run(IBackgroundTaskInstance taskInstance)
		{
			rfm69Device.RegisterDump();

			rfm69Device.Initialise( Rfm69HcwDevice.RegOpModeMode.StandBy,
											bitRate:Rfm69HcwDevice.BitRate.bps4K8,
											frequency:915000000.0, frequencyDeviation:0X023d,
											preambleSize:16
											);

			rfm69Device.RegisterDump();

			// RegRxBW
			rfm69Device.RegisterManager.WriteByte(0x19, 0x2a);

			// RegDioMapping1
			rfm69Device.RegisterManager.WriteByte(0x26, 0x01);

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
