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
namespace devMobile.IoT.Rfm69Hcw.ReceiveTransmitEvents
{
	using System;
	using System.Diagnostics;
	using System.Text;
	using System.Threading.Tasks;
	using Windows.ApplicationModel.Background;
	using Windows.Devices.Gpio;

	sealed class Rfm69HcwDevice
	{
		public delegate void OnDataReceivedHandler(byte[] data);
		public class OnDataReceivedEventArgs : EventArgs
		{
			public byte? Address { get; set; }
			public byte[] Data { get; set; }
			public int Rssi { get; set; }
			public bool CrcOk { get; set; }
		}
		public delegate void onReceivedEventHandler(Object sender, OnDataReceivedEventArgs e);
		public event onReceivedEventHandler OnReceive;

		public delegate void OnDataTransmittedHandler(byte[] data);
		public class OnDataTransmitedEventArgs : EventArgs
		{
			public byte[] Data { get; set; }
		}
		public delegate void onTransmittededEventHandler(Object sender, OnDataTransmitedEventArgs e);
		public event onTransmittededEventHandler OnTransmit;

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
		public enum RegOpModeSequencer : byte
		{
			Off = 0b00000000,
			On = 0b10000000,
		}
		public const bool RegOpModeSequencerDefault = false;

		public enum RegOpModeListen : byte
		{
			Off = 0b00000000,
			On = 0b01000000,
		}
		public const bool RegOpModeListenDefault = false;

		private const byte RegOpModeListenAbort = 0b00100000;

		public enum RegOpModeMode : byte
		{
			Sleep = 0b00000000,
			StandBy = 0b00000100,
			FrequencySynthesiser = 0b00001000,
			Transmit = 0b00001100,
			Receive = 0b00010000,
		};
		public const RegOpModeMode RegOpModeModeDefault = RegOpModeMode.StandBy;

		// RegDataModul
		public enum DataMode : byte
		{
			PacketMode = 0b00000000,
			// Reserved 01
			ContinuousWithBitSynchroniser = 0b01000000,
			ContinuousWithourBitSynchroniser = 0b01100000,
		}
		const DataMode DataModeDefault = DataMode.PacketMode;

		public enum ModulationType
		{
			Fsk = 0b00000000,
			Ook = 0b00001000
			// Reserved 10,11
		}
		public const ModulationType ModulationTypeDefault = ModulationType.Fsk;

		public enum ModulationShapingFsk
		{
			NoShaping = 0b00000000,
			GaussianBT1_0 = 0b00000001,
			GaussianBT0_5 = 0b00000010,
			GaussianBT0_3 = 0b00000011,
		}
		public const ModulationShapingFsk modulationShapingFskDefault = ModulationShapingFsk.NoShaping;

		public enum ModulationShapingOok
		{
			NoShaping = 0b00000000,
			FilteringCutoffBR = 0b00000001,
			FilteringCutoff2BR = 0b00000010,
			// Reserved 11
		}
		public const ModulationShapingOok modulationShapingOokDefault = ModulationShapingOok.NoShaping;

		// BitRate configuration from table 9 in datasheet RegBitrateMsb, RegBitrateLsb
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
			bps57K6 = 0x022C,
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

		// RegOsc1 settings
		private const byte RcCalStart = 0b10000000;

		private enum RcCal : byte
		{
			InProgress = 0b00000000,
			Done = 0b01000000,
		}

		// RegAfcCtrl settings
		public enum AfcLowBeta : byte
		{
			Standard = 0b00000000,
			Improved = 0b00100000,
		}
		public const AfcLowBeta AfcLowBetaDefault = AfcLowBeta.Standard;

		// RegListen1 settings
		public enum ListenModeIdleResolution : byte
		{
			// Reserved = 00
			Time64us = 0b01000000,
			Time4_1ms = 0b10000000,
			Time262ms = 0b11000000,
		}
		const ListenModeIdleResolution ListenModeIdleResolutionDefault = ListenModeIdleResolution.Time4_1ms;

		public enum ListenModeRXTime : byte
		{
			// Reserved
			IdleTime64us = 0b00010000,
			IdleTime4_1ms = 0b00100000,
			IdleTime262ms = 0b00110000,
		}
		const ListenModeRXTime ListenModeRXTimeDefault = ListenModeRXTime.IdleTime64us;

		public enum ListenModeCriteria : byte
		{
			RssiThreshold = 0b00000000,
			RssiThresholdAndSyncAddressMatched = 0b00001000
		}
		const ListenModeCriteria ListenModeCriteriaDefault = ListenModeCriteria.RssiThreshold;

		public enum ListenModeEnd : byte
		{
			StayInRXMode = 0b00000000,
			StayInRxModeUntilPayloadReady = 0b00000010,
			StayInRxModeUntilPayloadReadyOrTimeoutInterrupt = 0b00000100,
			// Reserved 11
		}
		const ListenModeEnd ListenModeEndDefault = ListenModeEnd.StayInRxModeUntilPayloadReadyOrTimeoutInterrupt;

		// RegListen2 
		const byte ListenCoefficientIdleDefault = 0xf5;

		// RegListen3 
		const byte ListenCoefficientReceiveDefault = 0x20;

		// RegVersion default expected value
		const byte RegVersionValueExpected = 0x24;

		// RegPaLevel 
		public enum PaOn : byte
		{
			Pa0On = 0b10000000,
			Pa1On = 0b01000000,
			Pa2On = 0b00100000,
		}
		const bool pa0OnDefault = true;
		const bool pa1OnDefaut = false;
		const bool pa2OnDefault = false;
		// TODO consider doing dB maths like OCP trim
		const byte OutputpowerDefault = 0b00011111;
		const byte OutputpowerMinimum = 0b00000000;
		const byte OutputpowerMaximum = 0b00011111;

		// RegPaRamp 
		public enum PaRamp : byte
		{
			Time3_4ms = 0b00000000,
			Time2ms = 0b00000001,
			Time1ms = 0b00000010,
			Time500us = 0b00000011,
			Time250us = 0b00000100,
			Time125us = 0b00000101,
			Time100us = 0b00000110,
			Time62us = 0b00000111,
			Time50us = 0b00001000,
			Time40us = 0b00001001,
			Time31us = 0b00001010,
			Time25us = 0b00001011,
			Time20us = 0b00001100,
			Time15us = 0b00001101,
			Time12us = 0b00001110,
			Time10us = 0b00001111,
		}
		const PaRamp PaRampDefault = PaRamp.Time40us;

		// RegOcp values
		private enum Ocp : byte
		{
			Enabled = 0b00010000,
			Disabled = 0b00000000,
		}
		const bool OcpOnDefault = true;

		const byte OcpTrimMaMinimum = 45;
		const byte OcpTrimMaMaxmum = 120;
		const byte OcpTrimDefault = 95;

		// RegLna
		public enum LnaZin : byte
		{
			Impedance50Ohms = 0b00000000,
			Impedance200Ohms = 0b10000000
		}
		const LnaZin LnaZinDefault = LnaZin.Impedance200Ohms;

		public enum LnaCurrentGain : byte
		{
			Manual = 0b00001000,
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
			// Reserved
		}
		const LnaGainSelect LnaGainSelectDefault = LnaGainSelect.AGC;

		// RegRxBw
		const byte DccFrequencyDefault = 0b01000000;

		public enum RxBwMant
		{
			RxBwMant16 = 0b00000000,
			RxBwMant20 = 0b00001000,
			RxBwMant24 = 0b00010000,
			// Reserved
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

		//RegRssiConfig
		[Flags]
		public enum RssiConfig
		{
			RssiStart = 0b00000001,
			RssiDone = 0b00000010
		}

		// RegDioMapping1 & RegDioMapping2 Packet Mode Table 22 pg48
		// DIO 0 Bits 7&6 of RegDioMapping1
		[Flags]
		public enum Dio0Mapping
		{
			// Sleep
			// Standby
			// Frequency Synthesis
			// Reserved 00-10
			FrequencySynthesisPllLock = 0b11000000,
			ReceiveCrcOk = 0b00000000,
			ReceivePayloadReady = 0b01000000,
			ReceiveSyncAddress = 0b10000000,
			ReceiveRssi = 0b11000000,
			TransmitPacketSent = 0b00000000,
			TransmitTxReady = 0b01000000,
			// Reserved 10
			PllLock = 0b11000000
		}
		const Dio0Mapping Dio0MappingDefault = 0x00;

		// DIO 1 Bits 5&4 of RegDioMapping1
		public enum Dio1Mapping
		{
			SleepFifoLevel = 0b00000000,
			SleepFifoFull = 0b00010000,
			SleepFifoNotEmpty = 0b00100000,
			// Reserved 11
			StandByFifoLevel = 0b00000000,
			StandByFifoFull = 0b00010000,
			StandByFifoNotEmpty = 0b00100000,
			FrequencySynthesisFifoLevel = 0b00000000,
			FrequencySynthesisFifoFull = 0b00010000,
			FrequencySynthesisFifoNotEmpty = 0b00100000,
			FrequencySynthesisPllLock = 0b00110000,
			ReceiveFifoLevel = 0b00000000,
			ReceiveFifoFull = 0b00010000,
			ReceiveFifoNotEmpty = 0b00100000,
			ReceiveTimeout = 0b00110000,
			TransmitFifoLevel = 0b00000000,
			TransmitFifoFull = 0b00010000,
			TransmitFifoNotEmpty = 0b00100000,
			TransmitPllLock = 0b00110000,
		}
		const Dio1Mapping Dio1MappingDefault = 0x00;

		// DIO 2 Bits 3&2 of RegDioMapping1
		public enum Dio2Mapping
		{
		}
		const Dio2Mapping Dio2MappingDefault = 0x00;

		// DIO 2 Bits 1&0 of RegDioMapping1
		public enum Dio3Mapping
		{
		}
		const Dio3Mapping Dio3MappingDefault = 0x00;

		// DIO 2 Bits 7&6 of RegDioMapping2
		public enum Dio4Mapping
		{
		}
		const Dio4Mapping Dio4MappingDefault = 0x00;

		// DIO 2 Bits 5&4 of RegDioMapping2
		public enum Dio5Mapping
		{
		}
		const Dio5Mapping Dio5MappingDefault = 0x00;

		// RegDioMapping2 Bits 2-0
		public enum ClockOutDioMapping : byte
		{
			FXOsc = 0b00000000,
			FXOscDiv2 = 0b00000001,
			FXOscDiv4 = 0b00000010,
			FXOscDiv8 = 0b00000011,
			FXOscDiv16 = 0b00000100,
			FXOscDiv32 = 0b00000101,
			RC = 0b00000110,
			Off = 0b00000111,
		}
		public const ClockOutDioMapping ClockOutDioMappingDefault = ClockOutDioMapping.Off;

		// RegIrqFlags1
		[Flags]
		public enum RegIrqFlags1 : byte
		{
			ModeReady = 0b10000000,
			RxReady = 0b01000000,
			TxReady = 0b00100000,
			PllLock = 0b00010000,
			Rssi = 0b00001000,
			Timeout = 0b00000100,
			AutoMode = 0b00000010,
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

		// RegRssiThresh 
		const ushort RegRssiThreshholdDefault = 0xE4;

		// RegsRxTimeout1
		const ushort RegRxTimeout1Default = 0x00;

		// RegsRxTimeout2
		const ushort RegRxTimeout2Default = 0x00;

		// RegPreambleMsb
		const ushort PreambleSizeDefault = 0x03;

		// RegSyncConfig 
		// This is private because default ignored and flag set based on SyncValues parameter being specified rather than default
		private enum RegSyncConfigSyncOn
		{
			Off = 0b00000000,
			On = 0b10000000
		}
		const bool RegSyncConfigSyncOnDefault = true;

		public enum RegSyncConfigFifoFileCondition
		{
			SyncAddressInterrupt = 0b00000000,
			FifoFillCondition = 0b01000000
		}

		private const RegSyncConfigFifoFileCondition SyncFifoFileConditionDefault = RegSyncConfigFifoFileCondition.SyncAddressInterrupt;
		public readonly byte[] SyncValuesDefault = { 0x01, 0x01, 0x01, 0x01 };
		public const byte SyncValuesSizeDefault = 4;
		public const byte SyncValuesSizeMinimum = 1;
		public const byte SyncValuesSizeMaximum = 8;

		private const byte SyncToleranceDefault = 0;
		public const byte SyncToleranceMinimum = 0;
		public const byte SyncToleranceMaximum = 7;


		// RegPacketConfig1
		public enum RegPacketConfig1PacketFormat : byte
		{
			FixedLength = 0b00000000,
			VariableLength = 0b10000000
		}
		const RegPacketConfig1PacketFormat RegPacketConfig1PacketFormatDefault = RegPacketConfig1PacketFormat.FixedLength;

		public enum RegPacketConfig1DcFree : byte
		{
			None = 0b00000000,
			Manchester = 0b00100000,
			Whitening = 0b01000000,
			Reserved = 0b01100000,
		}
		const RegPacketConfig1DcFree RegPacketConfig1DcFreeDefault = RegPacketConfig1DcFree.None;

		private enum RegPacketConfig1Crc : byte
		{
			Off = 0b00000000,
			On = 0b00010000,
		}
		const bool PacketCrcOnDefault = true;

		private enum RegPacketConfig1CrcAutoClear : byte
		{
			On = 0b00000000,
			Off = 0b00001000,
		}
		const bool PacketCrcAutoClearDefault = true;

		public enum RegPacketConfig1CrcAddressFiltering : byte
		{
			None = 0b00000000,
			NodeAddress = 0b00000010,
			NodeAddressOrBroadcastAddress = 0b00000100,
			// Reserved = 11
		}
		const RegPacketConfig1CrcAddressFiltering PacketAddressFilteringDefault = RegPacketConfig1CrcAddressFiltering.None;

		// RegPayloadLength
		const byte PayloadLengthDefault = 0x40;
		const byte PayloadLengthAddressedMinimum = 0;
		const byte PayloadLengthMaximum = 255;
		const byte PayloadLengthAddressedMaximum = 254;
		const byte PayloadLengthAesEnabledMaximum = 64;

		// RegNodeAdrs
		const byte NodeAddressDefault = 0x0;

		// RegBroadcastAdrs
		const byte BroadcastAddressDefault = 0x0;

		// RegFifoThresh
		[Flags]
		public enum TxStartCondition : byte
		{
			FifoLevel = 0b00000000,
			FifoNotEmpty = 0b10000000,
		};
		const TxStartCondition TxStartConditionDefault = TxStartCondition.FifoNotEmpty;

		const byte FifoThresholdDefault = 0xF;
		const byte FifoThresholdMinimum = 0x0;
		const byte FifoThresholdaximum = 0x7E;


		// RegPacketConfig2
		private const byte InterPacketRxDelayDefault = 0;
		public const byte InterPacketRxDelayMinimum = 0x0;
		public const byte InterPacketRxDelayMaximum = 0xF;

		private enum RegPacketConfig2RestartRx : byte
		{
			Off = 0b00000000,
			On = 0b00000100,
		}

		private enum RegPacketConfig2AutoRestartRx : byte
		{
			Off = 0b00000000,
			On = 0b00000010,
		}
		private const bool AutoRestartRxDefault = true;

		private enum RegPacketConfig2Aes : byte
		{
			Off = 0b00000000,
			On = 0b00000001,
		}
		public const byte AesKeyLength = 16;
		private readonly byte[] aesKeyDefault = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

		// Hardware configuration support
		private RegOpModeMode RegOpModeModeCurrent = RegOpModeMode.Sleep;
		private RegOpModeSequencer Sequencer = RegOpModeSequencer.Off;
		private RegOpModeListen Listen = RegOpModeListen.Off;
		private Dio0Mapping Dio0 = Dio0MappingDefault;
		private Dio1Mapping Dio1 = Dio1MappingDefault;
		private Dio2Mapping Dio2 = Dio2MappingDefault;
		private Dio3Mapping Dio3 = Dio3MappingDefault;
		private Dio4Mapping Dio4 = Dio4MappingDefault;
		private Dio5Mapping Dio5 = Dio5MappingDefault;
		private ClockOutDioMapping ClockOutDio = ClockOutDioMappingDefault;
		private RegPacketConfig1PacketFormat PacketFormat = RegPacketConfig1PacketFormatDefault;
		private Byte PayloadLength;
		private bool AddressingEnabled = false;
		private bool AesEnabled = false;
		private GpioPin InterruptGpioPin = null;
		private GpioPin ResetGpioPin = null;
		public RegisterManager RegisterManager = null; // Future refactor this will be made private
		private readonly Object Rfm9XRegFifoLock = new object();
		private bool InterruptProccessing = false;

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
			InterruptGpioPin.SetDriveMode(GpioPinDriveMode.Input);

			InterruptGpioPin.ValueChanged += InterruptGpioPin_ValueChanged;
		}

		public void RegisterDump()
		{
			RegisterManager.Dump((byte)Registers.MinValue, (byte)Registers.MaxValue);
		}


		public void SetMode(RegOpModeMode mode)
		{
			byte regOpModeValue = (byte)mode;

			regOpModeValue |= (byte)Sequencer;
			regOpModeValue |= (byte)Listen;

			RegisterManager.WriteByte((byte)Registers.RegOpMode, regOpModeValue);
		}


		public void Initialise(RegOpModeMode modeAfterInitialise,
			bool sequencer = RegOpModeSequencerDefault,
			bool listen = RegOpModeListenDefault,
			BitRate bitRate = BitRateDefault,
			ushort frequencyDeviation = frequencyDeviationDefault,
			double frequency = FrequencyDefault,
			AfcLowBeta afcLowBeta = AfcLowBetaDefault,
			ListenModeIdleResolution listenModeIdleResolution = ListenModeIdleResolutionDefault, ListenModeRXTime listenModeRXTime = ListenModeRXTimeDefault, ListenModeCriteria listenModeCrieria = ListenModeCriteriaDefault, ListenModeEnd listenModeEnd = ListenModeEndDefault,
			byte listenCoefficientIdle = ListenCoefficientIdleDefault,
			byte listenCoefficientReceive = ListenCoefficientReceiveDefault,
			bool pa0On = pa0OnDefault, bool pa1On = pa1OnDefaut, bool pa2On = pa2OnDefault, byte outputpower = OutputpowerDefault,
			PaRamp paRamp = PaRampDefault,
			bool ocpOn = OcpOnDefault, byte ocpTrim = OcpTrimDefault,
			LnaZin lnaZin = LnaZinDefault, LnaCurrentGain lnaCurrentGain = LnaCurrentGainDefault, LnaGainSelect lnaGainSelect = LnaGainSelectDefault,
			byte dccFrequency = DccFrequencyDefault, RxBwMant rxBwMant = RxBwMantDefault, byte RxBwExp = RxBwExpDefault,
			byte dccFreqAfc = DccFreqAfcDefault, byte rxBwMantAfc = RxBwMantAfcDefault, byte bxBwExpAfc = RxBwExpAfcDefault,
			Dio0Mapping dio0Mapping = Dio0MappingDefault,
			Dio1Mapping dio1Mapping = Dio1MappingDefault,
			Dio2Mapping dio2Mapping = Dio2MappingDefault,
			Dio3Mapping dio3Mapping = Dio3MappingDefault,
			Dio4Mapping dio4Mapping = Dio4MappingDefault,
			Dio5Mapping dio5Mapping = Dio5MappingDefault,
			ClockOutDioMapping clockOutDioMapping = ClockOutDioMappingDefault,
			ushort preambleSize = PreambleSizeDefault,
			RegSyncConfigFifoFileCondition? syncFifoFileCondition = null, byte? syncTolerance = null, byte[] syncValues = null,
			RegPacketConfig1PacketFormat packetFormat = RegPacketConfig1PacketFormat.FixedLength,
			RegPacketConfig1DcFree packetDcFree = RegPacketConfig1DcFreeDefault,
			bool packetCrc = PacketCrcOnDefault,
			bool packetCrcAutoClear = PacketCrcAutoClearDefault,
			byte payloadLength = PayloadLengthDefault,
			byte? addressNode = null, byte? addressbroadcast = null,
			TxStartCondition txStartCondition = TxStartConditionDefault, byte fifoThreshold = FifoThresholdDefault,
			byte interPacketRxDelay = InterPacketRxDelayDefault, bool autoRestartRx = AutoRestartRxDefault,
			byte[] aesKey = null
			)
		{
			RegOpModeModeCurrent = modeAfterInitialise;
			if (sequencer)
			{
				Sequencer = RegOpModeSequencer.On;
			}
			else
			{
				Sequencer = RegOpModeSequencer.Off;
			}
			if (listen)
			{
				Listen = RegOpModeListen.On;
			}
			else
			{
				Listen = RegOpModeListen.Off;
			}
			Dio0 = dio0Mapping;
			Dio1 = dio1Mapping;
			Dio2 = dio2Mapping;
			Dio3 = dio3Mapping;
			Dio4 = dio4Mapping;
			Dio5 = dio5Mapping;
			ClockOutDio = clockOutDioMapping;
			PacketFormat = packetFormat;
			PayloadLength = payloadLength;
			AddressingEnabled = (addressNode.HasValue || addressbroadcast.HasValue);
			AesEnabled = (aesKey != null);

#region RegSyncConfig + RegSyncValue1 to RegSyncValue8 guard conditions
			if (syncValues != null)
			{
				// If sync enabled (i.e. SyncValues array provided) check that SyncValues not to short/long and SyncTolerance not to small/big
				if ((syncValues.Length < SyncValuesSizeMinimum) || (syncValues.Length > SyncValuesSizeMaximum))
				{
					throw new ArgumentException($"The syncValues array length must be between {SyncValuesSizeMinimum} and {SyncValuesSizeMaximum} bytes", "syncValues");
				}
				if (syncTolerance.HasValue)
				{
					if ((syncTolerance < SyncToleranceMinimum) || (syncTolerance > SyncToleranceMaximum))
					{
						throw new ArgumentException($"The syncTolerance size must be between {SyncToleranceMinimum} and {SyncToleranceMaximum}", "syncTolerance");
					}
				}
			}
			else
			{
				// If sync not enabled (i.e. SyncValues array null) check that no syncFifoFileCondition or syncTolerance configuration specified
				if (syncFifoFileCondition.HasValue)
				{
					throw new ArgumentException($"If Sync not enabled syncFifoFileCondition is not supported", "syncFifoFileCondition");
				}

				if (syncTolerance.HasValue)
				{
					throw new ArgumentException($"If Sync not enabled SyncTolerance is not supported", "syncTolerance");
				}
			}
#endregion

#region RegPacketConfig2 + RegAesKey1 to RegAesKey16 guard conditions
			if ((interPacketRxDelay < InterPacketRxDelayMinimum) || (interPacketRxDelay > InterPacketRxDelayMaximum))
			{
				throw new ArgumentException($"The interPacketRxDelay must be between {InterPacketRxDelayMinimum} and {InterPacketRxDelayMaximum}", "interPacketRxDelay");
			}
			if (AesEnabled && (aesKey.Length != AesKeyLength))
			{
				throw new ArgumentException($"The AES key must be {AesKeyLength} bytes", "aesKey");
			}
#endregion

			// Strobe Reset pin briefly to factory reset SX1231 chip
			ResetGpioPin.Write(GpioPinValue.High);
			Task.Delay(100);
			ResetGpioPin.Write(GpioPinValue.Low);
			Task.Delay(10);

			// Put the device into sleep mode so registers can be changed
			SetMode(RegOpModeMode.Sleep);

			// RegDataModul ignored

			// RegBitrateMsb, RegBitrateLsb
			if (bitRate != BitRateDefault)
			{
				byte[] bytes = BitConverter.GetBytes((ushort)bitRate);
				RegisterManager.WriteByte((byte)Registers.RegBitrateMsb, bytes[1]);
				RegisterManager.WriteByte((byte)Registers.RegBitrateLsb, bytes[0]);
			}

			// RegFdevMsb, RegFdevLsb
			if (frequencyDeviation != frequencyDeviationDefault)
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

			// RegAfcCtrl settings
			if (afcLowBeta != AfcLowBetaDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegAfcCtrl, (byte)afcLowBeta);
			}

			// RegListen1 settings
			if ((listenModeIdleResolution != ListenModeIdleResolutionDefault) ||
				 (listenModeRXTime != ListenModeRXTimeDefault) ||
				 (listenModeCrieria != ListenModeCriteriaDefault) ||
				 (listenModeEnd != ListenModeEndDefault))
			{
				byte regListen1Value = (byte)listenModeIdleResolution;

				regListen1Value |= (byte)listenModeRXTime;
				regListen1Value |= (byte)listenModeCrieria;
				regListen1Value |= (byte)listenModeEnd;

				RegisterManager.WriteByte((byte)Registers.RegListen1, regListen1Value);
			}

			// RegListen2 settings
			if (listenCoefficientIdle != ListenCoefficientIdleDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegListen2, listenCoefficientIdle);
			}

			// RegListen3 settings
			if (listenCoefficientReceive != ListenCoefficientReceiveDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegListen3, listenCoefficientReceive);
			}

			// RegPaLevel settings
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

			// RegPaRamp settings
			if (paRamp != PaRampDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegPaRamp, (byte)paRamp);
			}

			// RegOcp settings 
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

			// regLnaValue settings
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
				byte regRxBwValue = (byte)(dccFrequency << 5);

				regRxBwValue |= (byte)rxBwMant;
				regRxBwValue |= RxBwExp;

				RegisterManager.WriteByte((byte)Registers.RegRxBw, regRxBwValue);
			}

			// RegAfcBw settings
			if ((dccFreqAfc != DccFreqAfcDefault) ||
				 (rxBwMantAfc != RxBwMantAfcDefault) ||
				 (bxBwExpAfc != RxBwExpAfcDefault))
			{
				byte regAfcBwValue = dccFreqAfc;

				regAfcBwValue |= rxBwMantAfc;
				regAfcBwValue |= bxBwExpAfc;

				RegisterManager.WriteByte((byte)Registers.RegAfcBw, regAfcBwValue);
			}

			// RegDioMapping1
			if ((dio0Mapping != Dio0MappingDefault) ||
				 (dio1Mapping != Dio1MappingDefault) ||
				 (dio2Mapping != Dio2MappingDefault) ||
				 (dio3Mapping != Dio3MappingDefault))
			{
				byte regDioMapping1Value = (byte)dio0Mapping;

				regDioMapping1Value |= (byte)dio1Mapping;
				regDioMapping1Value |= (byte)dio2Mapping;
				regDioMapping1Value |= (byte)dio3Mapping;

				RegisterManager.WriteByte((byte)Registers.RegDioMapping1, regDioMapping1Value);
			}

			// RegDioMapping2
			if ((dio4Mapping != Dio4MappingDefault) ||
				 (dio5Mapping != Dio5MappingDefault) ||
				 (clockOutDioMapping != ClockOutDioMappingDefault))
			{
				byte regDioMapping2Value = (byte)dio4Mapping;

				regDioMapping2Value |= (byte)dio5Mapping;
				regDioMapping2Value |= (byte)clockOutDioMapping;

				RegisterManager.WriteByte((byte)Registers.RegDioMapping2, regDioMapping2Value);
			}

			// RegPreambleMsb RegPreambleLsb settings
			if (preambleSize != PreambleSizeDefault)
			{
				byte[] bytes = BitConverter.GetBytes((ushort)preambleSize);
				RegisterManager.WriteByte((byte)Registers.RegPreambleMsb, bytes[1]);
				RegisterManager.WriteByte((byte)Registers.RegPreambleLsb, bytes[0]);
			}

			// RegSyncConfig - This one is a little bit different to others as sync normally on by default, now turned on if SyncValues specified
			byte regSyncConfigValue;
			if (syncValues != null)
			{
				regSyncConfigValue = (byte)RegSyncConfigSyncOn.On;
				regSyncConfigValue |= (byte)((syncValues.Length - 1) << 3);
				if (syncFifoFileCondition.HasValue)
				{
					regSyncConfigValue |= (byte)syncFifoFileCondition;
				}
				else
				{
					regSyncConfigValue |= (byte)SyncFifoFileConditionDefault;
				}

				if (syncTolerance.HasValue)
				{
					regSyncConfigValue |= syncTolerance.Value;
				}
				else
				{
					regSyncConfigValue |= SyncToleranceDefault;
				}

				RegisterManager.WriteByte((byte)Registers.RegSyncConfig, regSyncConfigValue);
			}

			// RegSyncValue1 to RegSyncValue8
			if (syncValues != null)
			{
				RegisterManager.Write((byte)Registers.RegSyncValue1, syncValues);
			}

			// RegPacketConfig1
			if ((packetFormat != RegPacketConfig1PacketFormatDefault) ||
				 (packetDcFree != RegPacketConfig1DcFreeDefault) ||
				 (packetCrc != PacketCrcOnDefault) ||
				 (packetCrcAutoClear != PacketCrcAutoClearDefault) ||
				 addressNode.HasValue ||
				 addressbroadcast.HasValue)
			{
				byte packetConfig1Value = (byte)packetFormat;

				packetConfig1Value |= (byte)packetDcFree;

				if (packetCrc)
				{
					packetConfig1Value |= (byte)RegPacketConfig1Crc.On;
				}
				else
				{
					packetConfig1Value |= (byte)RegPacketConfig1Crc.Off;
				}

				if (packetCrcAutoClear)
				{
					packetConfig1Value |= (byte)RegPacketConfig1CrcAutoClear.On;
				}
				else
				{
					packetConfig1Value |= (byte)RegPacketConfig1CrcAutoClear.Off;
				}

				packetConfig1Value |= (byte)RegPacketConfig1CrcAddressFiltering.None;
				if (addressNode.HasValue && !addressbroadcast.HasValue)
				{
					packetConfig1Value |= (byte)RegPacketConfig1CrcAddressFiltering.NodeAddress;
				}
				if (addressNode.HasValue && addressbroadcast.HasValue)
				{
					packetConfig1Value |= (byte)RegPacketConfig1CrcAddressFiltering.NodeAddressOrBroadcastAddress;
				}

				RegisterManager.WriteByte((byte)Registers.RegPacketConfig1, packetConfig1Value);
			}

			// RegPayloadLength
			if (payloadLength != PayloadLengthDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegPayloadLength, payloadLength);
			}

			// RegNodeAdrs
			if (addressNode.HasValue)
			{
				RegisterManager.WriteByte((byte)Registers.RegNodeAdrs, addressNode.Value);
			}

			// RegBroadcastAdrs
			if (addressbroadcast.HasValue)
			{
				RegisterManager.WriteByte((byte)Registers.RegBroadcastAdrs, addressbroadcast.Value);
			}

			// RegAutoMode ignored

			// RegFifoThresh
			if ((txStartCondition != TxStartConditionDefault) || (fifoThreshold != FifoThresholdDefault))
			{
				byte regFifoThreshValue = (byte)txStartCondition;

				regFifoThreshValue |= fifoThreshold;

				RegisterManager.WriteByte((byte)Registers.RegFifoThresh, regFifoThreshValue);
			}

			// RegPacketConfig2
			if ((interPacketRxDelay != InterPacketRxDelayDefault) || (autoRestartRx != AutoRestartRxDefault) || AesEnabled)
			{
				byte packetConfig2Value = (byte)interPacketRxDelay;

				if (autoRestartRx)
				{
					packetConfig2Value |= (byte)RegPacketConfig2AutoRestartRx.On;
				}
				else
				{
					packetConfig2Value |= (byte)RegPacketConfig2AutoRestartRx.Off;
				}

				if (AesEnabled)
				{
					packetConfig2Value |= (byte)RegPacketConfig2Aes.On;
				}
				else
				{
					packetConfig2Value |= (byte)RegPacketConfig2Aes.Off;
				}

				RegisterManager.WriteByte((byte)Registers.RegPacketConfig2, packetConfig2Value);
			}

			// RegAesKey1 through RegAesKey16
			if (AesEnabled)
			{
				RegisterManager.Write((byte)Registers.RegAesKey1, aesKey);
			}
			else
			{
				RegisterManager.Write((byte)Registers.RegAesKey1, aesKeyDefault);
			}

			// Configure RegOpMode before returning
			SetMode(modeAfterInitialise);
		}

		private void ProcessPayloadReady(RegIrqFlags2 irqFlags2)
		{
			byte numberOfBytes;
			bool crcValid;
			byte? address = null;
			byte[] messageBytes;

			lock (Rfm9XRegFifoLock)
			{
				SetMode(RegOpModeMode.StandBy);

				crcValid = ((irqFlags2 & RegIrqFlags2.CrcOk) == RegIrqFlags2.CrcOk);

				// Read the length of the buffer if variable length packets
				if (PacketFormat == RegPacketConfig1PacketFormat.VariableLength)
				{
					numberOfBytes = RegisterManager.ReadByte((byte)Rfm69HcwDevice.Registers.RegFifo);
				}
				else
				{
					numberOfBytes = PayloadLength;
				}

				// Remove the address from start of the payload
				if (AddressingEnabled)
				{
					address = RegisterManager.ReadByte((byte)Rfm69HcwDevice.Registers.RegFifo);

					//Debug.WriteLine("{0:HH:mm:ss.fff} Address 0X{1:X2} {2}", DateTime.Now, address, Convert.ToString((byte)address, 2).PadLeft(8, '0'));
					numberOfBytes--;
				}

				// Allocate a buffer for the payload and read characters from the Fifo
				messageBytes = new byte[numberOfBytes];

				for (int i = 0; i < numberOfBytes; i++)
				{
					messageBytes[i] = RegisterManager.ReadByte((byte)Rfm69HcwDevice.Registers.RegFifo);
				}
			}

			if (this.OnReceive != null)
			{
				OnDataReceivedEventArgs receiveArgs = new OnDataReceivedEventArgs();

				receiveArgs.Address = address;
				receiveArgs.Data = messageBytes;
				receiveArgs.CrcOk = crcValid;

				OnReceive(this, receiveArgs);
			}
		}

		private void ProcessPacketSent()
		{
			//Debug.WriteLine("{0:HH:mm:ss.fff} Transmit-Done", DateTime.Now);

			if (this.OnTransmit != null)
			{
				OnDataTransmitedEventArgs transmitArgs = new OnDataTransmitedEventArgs();

				OnTransmit(this, transmitArgs);
			}
		}

		private void InterruptGpioPin_ValueChanged(GpioPin sender, GpioPinValueChangedEventArgs args)
		{
			if (args.Edge != GpioPinEdge.RisingEdge)
			{
				return;
			}

			if (InterruptProccessing)
			{
				Debug.WriteLine("{0:HH:mm:ss} InterruptProccessing++++++++++++", DateTime.Now);
				return;
			}
			InterruptProccessing = true;

			RegIrqFlags2 irqFlags2 = (RegIrqFlags2)RegisterManager.ReadByte((byte)Registers.RegIrqFlags2);

			if ((irqFlags2 & RegIrqFlags2.PayloadReady) == RegIrqFlags2.PayloadReady)
			{
				ProcessPayloadReady(irqFlags2);
			}

			if ((irqFlags2 & RegIrqFlags2.PacketSent) == RegIrqFlags2.PacketSent)  // PacketSent set
			{
				ProcessPacketSent();
			}

			InterruptProccessing = false;
		}

		public void SetDioPinMapping(Dio0Mapping dio0Mapping = Rfm69HcwDevice.Dio0MappingDefault,
											Dio1Mapping dio1Mapping = Rfm69HcwDevice.Dio1MappingDefault,
											Dio2Mapping dio2Mapping = Rfm69HcwDevice.Dio2MappingDefault,
											Dio3Mapping dio3Mapping = Rfm69HcwDevice.Dio3MappingDefault,
											Dio4Mapping dio4Mapping = Rfm69HcwDevice.Dio4MappingDefault,
											Dio5Mapping dio5Mapping = Rfm69HcwDevice.Dio5MappingDefault)
		{
			if ((dio0Mapping != this.Dio0) ||
				 (dio1Mapping != this.Dio1) ||
				 (dio2Mapping != this.Dio2) ||
				 (dio3Mapping != this.Dio3))
			{
				this.Dio0 = dio0Mapping;
				this.Dio1 = dio1Mapping;
				this.Dio2 = dio2Mapping;
				this.Dio3 = dio3Mapping;

				byte regDioMapping1Value = (byte)dio0Mapping;

				regDioMapping1Value |= (byte)dio1Mapping;
				regDioMapping1Value |= (byte)dio2Mapping;
				regDioMapping1Value |= (byte)dio3Mapping;

				RegisterManager.WriteByte((byte)Registers.RegDioMapping1, regDioMapping1Value);
			}

			// RegDioMapping2
			if ((dio4Mapping != this.Dio4) ||
				 (dio5Mapping != this.Dio5))
			{
				this.Dio4 = dio4Mapping;
				this.Dio5 = dio5Mapping;

				byte regDioMapping2Value = (byte)dio4Mapping;
				regDioMapping2Value |= (byte)dio5Mapping;
				regDioMapping2Value |= (byte)this.ClockOutDio;

				RegisterManager.WriteByte((byte)Registers.RegDioMapping2, regDioMapping2Value);
			}
		}

		public void SetClockOutMapping(ClockOutDioMapping clockOutDioMapping)
		{
			// RegDioMapping2
			if (clockOutDioMapping != this.ClockOutDio)
			{
				this.ClockOutDio = clockOutDioMapping;

				byte regDioMapping2Value = (byte)this.Dio4;
				regDioMapping2Value |= (byte)this.Dio5;
				regDioMapping2Value |= (byte)clockOutDioMapping;

				RegisterManager.WriteByte((byte)Registers.RegDioMapping2, regDioMapping2Value);
			}
		}

		public void SendMessage(byte[] messageBytes)
		{
#region Guard conditions
			if (AddressingEnabled)
			{
				throw new ApplicationException("Addressed message mode enabled");
			}

			if (this.AesEnabled)
			{
				if (messageBytes.Length > PayloadLengthAesEnabledMaximum)
				{
					throw new ArgumentException($"Payload maximum {PayloadLengthAesEnabledMaximum} bytes when encryption enabled", "messageBytes");
				}
			}
			else
			{
				if (messageBytes.Length > PayloadLengthMaximum)
				{
					throw new ArgumentException($"Payload maximum {PayloadLengthMaximum} bytes", "messageBytes");
				}
			}
			#endregion

			SetMode(RegOpModeMode.StandBy);

			lock (Rfm9XRegFifoLock)
			{

				if (PacketFormat == RegPacketConfig1PacketFormat.VariableLength)
				{
					RegisterManager.WriteByte((byte)Registers.RegFifo, (byte)messageBytes.Length);
				}

				foreach (byte b in messageBytes)
				{
					this.RegisterManager.WriteByte((byte)Registers.RegFifo, b);
				}
			}

			SetMode(RegOpModeMode.Transmit);
		}

		public void SendMessage(byte address, byte[] messageBytes)
		{
#region Guard conditions
			if (!AddressingEnabled)
			{
				throw new ApplicationException("Addressed message mode not enabled");
			}

			if (this.AesEnabled)
			{
				if (messageBytes.Length > PayloadLengthAesEnabledMaximum)
				{
					throw new ArgumentException($"Payload maximum {PayloadLengthAesEnabledMaximum} bytes when encryption enabled", "messageBytes");
				}
			}
			else
			{
				if (messageBytes.Length > PayloadLengthAddressedMaximum)
				{
					throw new ArgumentException($"Payload maximum {PayloadLengthAesEnabledMaximum} bytes when addressing enabled", "messageBytes");
				}
			}
			#endregion

			lock (Rfm9XRegFifoLock)
			{
				SetMode(RegOpModeMode.StandBy);


				if (PacketFormat == RegPacketConfig1PacketFormat.VariableLength)
				{
					RegisterManager.WriteByte((byte)Registers.RegFifo, (byte)(messageBytes.Length + 1)); // Additional byte for address 
				}

				RegisterManager.WriteByte((byte)Registers.RegFifo, address);

				foreach (byte b in messageBytes)
				{
					this.RegisterManager.WriteByte((byte)Registers.RegFifo, b);
				}

				SetMode(RegOpModeMode.Transmit);
			}
		}

		public short Rssi(bool forceTrigger)
		{
			short rssi = 0;

			// Based on 3.4.9. RSSI in SX1231 docs see RegRssiConfig & RegRssiValue
			if (forceTrigger)
			{
				// RSSI trigger not needed if DAGC is in continuous mode
				if (((AfcLowBeta)RegisterManager.ReadByte((byte)Registers.RegTestDagc) & AfcLowBeta.Improved) == 0x00)  
				{
					RegisterManager.WriteByte((byte)Registers.RegRssiConfig, (byte)RssiConfig.RssiStart);
					while(((RssiConfig)RegisterManager.ReadByte((byte)Registers.RegRssiConfig) & RssiConfig.RssiDone) == 0x00)
					{
						Debug.Write('?');
					}
				}
			}

			// Adjust value based on "Absolute value of the RSSI in dbm, 0.5 db steps RSSI = -RssiValue/2[dBm]
			rssi = (short)RegisterManager.ReadByte((byte)Registers.RegRssiValue);
			rssi /= -2;

			return rssi;
		}
	}


	public sealed class StartupTask : IBackgroundTask
	{
		private const int ResetPin = 25;
		private const int InterruptPin = 22;
		private Rfm69HcwDevice rfm69Device = new Rfm69HcwDevice(ChipSelectPin.CS1, ResetPin, InterruptPin);
		DateTime LastEvent = DateTime.Now;

		public void Run(IBackgroundTaskInstance taskInstance)
		{
			byte[] syncValues = { 0xAA, 0x2D, 0xD4 };
			byte[] aesKeyValues = { 0x0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0X0E, 0X0F };

			try
			{
				rfm69Device.Initialise(Rfm69HcwDevice.RegOpModeMode.StandBy,
												bitRate: Rfm69HcwDevice.BitRate.bps4K8,
												frequency: 909560000.0, frequencyDeviation: 0X023d,
												dccFrequency: 0x1, rxBwMant: Rfm69HcwDevice.RxBwMant.RxBwMant20, RxBwExp: 0x2,
												dio0Mapping: Rfm69HcwDevice.Dio0Mapping.ReceiveCrcOk,
												preambleSize: 16,
												syncValues: syncValues,
												packetFormat: Rfm69HcwDevice.RegPacketConfig1PacketFormat.VariableLength,
												autoRestartRx: false,
												addressNode: 0x22,
												addressbroadcast: 0x99//,
												//aesKey: aesKeyValues
												);

				rfm69Device.OnReceive += Rfm69Device_OnReceive;
				rfm69Device.OnTransmit += Rfm69Device_OnTransmit;

				rfm69Device.RegisterDump();

				rfm69Device.SetMode(Rfm69HcwDevice.RegOpModeMode.Receive);


				while (true)
				{
					if (true)
					{
						string message = $"hello world {Environment.MachineName} {DateTime.Now:hh-mm-ss}";

						byte[] messageBuffer = UTF8Encoding.UTF8.GetBytes(message);

						Debug.WriteLine("{0:HH:mm:ss.fff} Send-{1}", DateTime.Now, message);
						rfm69Device.SendMessage( 0x11, messageBuffer);
						//rfm69Device.SendMessage(messageBuffer);

						Debug.WriteLine("{0:HH:mm:ss.fff} Send-Done", DateTime.Now);

						Task.Delay(5000).Wait();
					}
					else
					{
						Debug.Write(".");
						Task.Delay(1000).Wait();
					}
				}
			}
			catch (Exception ex)
			{
				Debug.WriteLine(ex.Message);
			}
		}

		private void Rfm69Device_OnReceive(object sender, Rfm69HcwDevice.OnDataReceivedEventArgs e)
		{
			DateTime currentEvent = DateTime.Now;

			this.rfm69Device.SetMode(Rfm69HcwDevice.RegOpModeMode.Receive);

			try
			{
				string messageText = UTF8Encoding.UTF8.GetString(e.Data);
				if (e.Address.HasValue)
				{
					Debug.WriteLine("{0:HH:mm:ss.fff} Received To {1} a {2} byte message {3} CRC Ok {4}", DateTime.Now, e.Address, e.Data.Length, messageText, e.CrcOk);
				}
				else
				{
					Debug.WriteLine("{0:HH:mm:ss.fff} Received {1} byte message {2} CRC Ok {3}", DateTime.Now, e.Data.Length, messageText, e.CrcOk);
				}
			}
			catch (Exception ex)
			{
				Debug.WriteLine(ex.Message);
			}

			if (currentEvent - LastEvent > new TimeSpan(0, 0, 4))
			{
				Debug.WriteLine("R--------------------------------------------");
			}
			LastEvent = currentEvent;
		}

		private void Rfm69Device_OnTransmit(object sender, Rfm69HcwDevice.OnDataTransmitedEventArgs e)
		{
			DateTime currentEvent = DateTime.Now;

			rfm69Device.SetMode(Rfm69HcwDevice.RegOpModeMode.Receive);

			Debug.WriteLine("{0:HH:mm:ss.fff} Transmit-Done", DateTime.Now);

			if (currentEvent - LastEvent > new TimeSpan(0, 0, 4))
			{
				Debug.WriteLine("T--------------------------------------------");
			}
			LastEvent = currentEvent;
		}
	}
}
