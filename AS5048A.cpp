
AS5048A::AS5048A(uint8_t CSPinToSet)
{
	CSPin = CSPinToSet;
	Verbose = false;
	IsError = false;
	ConnectionSettings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
	pinMode(CSPin, OUTPUT);
	digitalWrite(CSPin, HIGH);
}

AS5048A::CalculateParity(uint16_t Value)
{
	uint8_t Count = 0;
	for (uint8_t i = 0; i < 16; i++)
	{
		if (Value & 0x1)
		{
			Count++;
		}
		Value >>= 1;
	}
	return Count & 0x1;
}

AS5048A::begin()
{
	GetAndClearErrors();
}

uint16_t AS5048A::GetZeroPosition()
{
	SPI.beginTransaction(ConnectionSettings);
	SendCommand(CommandRegisters::ZeroPositionHighByte, true);
	uint16_t HighByte = SendCommand(CommandRegisters::ZeroPositionLowByte, true);
	uint16_t LowByte = SendCommand(CommandRegisters::NOP, true);
	SPI.endTransaction();
	uint16_t ZeroPosition = 0;
	ZeroPosition = ( (HighByte & 0x00FF) << 5 ) | (LowByte & 0x001F);
	return ZeroPosition;
}

uint16_t AS5048A::SendCommand(AS5048A::CommandRegisters Command, bool IsRead)
{
	uint16_t CommandToSend = static_cast<uint16_t>(Command);
	bitWrite(CommandToSend, 14, IsRead);
	CommandToSend |= static_cast<uint16_t>(CalculateParity(CommandToSend) << 0xF);
	digitalWrite(CSSelectPin,LOW);
	uint16_t Return = SPI.transfer16(CommandToSend);
	digitalWrite(CSSelectPin,HIGH);
	IsError = bitRead(Return,14);
	delayMicroseconds(1);
	if (Verbose)
	{
		Serial.print("[AS5048A](");
		Serial.print(CommandToSend);
		Serial.print(",");
		Serial.print(Return);
		Serial.print(",");
		Serial.print(IsError);
		Serial.println(")");
	}
	return Return;
}

void AS5048A::SetZeroPosition(uint16_t ZeroPositionToSet)
{
	uint16_t HighByte = (ZeroPositionToSet >> 5) & 0x00FF;
	uint16_t LowByte = ZeroPositionToSet & 0x001F;
	SPI.beginTransaction(ConnectionSettings);
	SendCommand(CommandRegisters::ZeroPositionHighByte, false);
	SPI.transfer16( HighByte );
	SendCommand(CommandRegisters::NOP, true);
	SendCommand(CommandRegisters::ZeroPositionLowByte, false);
	SPI.transfer16( LowByte );
	SendCommand(CommandRegisters::NOP, true);
	SPI.endTransaction();
}

void AS5048A::SetZeroPosition()
{
	SetZeroPosition(GetAngle());
}

uint16_t AS5048A::GetMagnitude()
{
	SPI.beginTransaction(ConnectionSettings);
	SendCommand(CommandRegisters::Magnitude, true);
	uint16_t Magnitude = SendCommand(CommandRegisters::NOP, true);
	SPI.endTransaction();
	return Magnitude & 0x3FFF;
}

uint16_t AS5048A::GetAngle()
{
	SPI.beginTransaction(ConnectionSettings);
	SendCommand(CommandRegisters::Angle, true);
	uint16_t Angle = SendCommand(CommandRegisters::NOP, true);
	SPI.endTransaction();
	return Angle & 0x3FFF;
}

AS5048A::DiagnosticData AS5048A::GetDiagnostics()
{
	SPI.beginTransaction(ConnectionSettings);
	SendCommand(CommandRegisters::DiagnosticAGC, true);
	uint16_t Diagnostic = SendCommand(CommandRegisters::NOP, true);
	SPI.endTransaction();
	DiagnosticData Return;
	Return.OvercompensationFinished = bitRead(Diagnostic,static_cast<uint8_t>(DiagnosticFlags::OffsetCompensationFinished));
	Return.CORDICOverflow = bitRead(Diagnostic,static_cast<uint8_t>(DiagnosticFlags::CORDICOverflow));
	Return.COMPLow = bitRead(Diagnostic,static_cast<uint8_t>(DiagnosticFlags::COMPLow));
	Return.COMPHigh = bitRead(Diagnostic,static_cast<uint8_t>(DiagnosticFlags::COMPHigh));
	Return.AutomaticGainControl = (uint8_t)(Diagnostic & 0xFF);
	return Return;
}

AS5048A::ErrorFlags AS5048A::GetAndClearErrors()
{
	SPI.beginTransaction(ConnectionSettings);
	SendCommand(CommandRegisters::ClearError, true);
	uint16_t Error = SendCommand(CommandRegisters::NOP, true);
	SPI.endTransaction();
	ErrorFlags Return;
	Return.ParityError = bitRead(Error,static_cast<uint8_t>(ErrorFlags::ParityError));
	Return.CommandInvalid = bitRead(Error,static_cast<uint8_t>(ErrorFlags::CommandInvalid));
	Return.FramingError = bitRead(Error,static_cast<uint8_t>(ErrorFlags::FramingError));
	return Return;
}

bool AS5048A::GetVerbose()
{
	return Verbose;
}

void AS5048A::SetVerbose(bool VerboseToSet)
{
	Verbose = VerboseToSet;
}

SPISettings* AS5048A::GetSPISettings()
{
	return &ConnectionSettings;
}