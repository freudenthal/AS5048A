#include <SPI.h>
#include <AS5048A.h>

//Intended for RP2040
//Changes will need to be made to the SPI calls for different processors

const uint32_t SerialUSBSpeed = 115200;
const uint8_t AS5048A_CSPin = 1;
const uint8_t AS5048A_TXPin = 3;
const uint8_t AS5048A_RXPin = 4;
const uint8_t AS5048A_CLKPin = 2;
const uint32_t AS5048A_ReadDelay = 200000; //80 effective minimum by datasheet
const bool AS5048A_Verbose = false;

uint32_t AS5048A_LastRead = 0;
AS5048A Encoder(AS5048A_CSPin);

uint16_t Magnitude = 0;
uint16_t Angle = 0;
AS5048A::DiagnosticData DiagnosticResults;
AS5048A::ErrorData ErrorResults;

void setup()
{
	SPI.setRX(AS5048A_RXPin);
	SPI.setSCK(AS5048A_CLKPin);
	SPI.setTX(AS5048A_TXPin);
	SPI.begin(false);
	Serial.begin(SerialUSBSpeed);
	Encoder.Begin();
	if (AS5048A_Verbose)
	{
		Encoder.SetVerbose(AS5048A_Verbose);
	}
}

void loop()
{
	if ( (micros() - AS5048A_LastRead) > AS5048A_ReadDelay)
	{
		AS5048A_LastRead = micros();
		Magnitude = Encoder.GetMagnitude();
		Angle = Encoder.GetAngle();
		DiagnosticResults = Encoder.GetDiagnostics();
		ErrorResults = Encoder.GetAndClearErrors();
		Serial.println("**********");
		Serial.print("Magnitude : ");
		Serial.println(Magnitude);
		Serial.print("Angle : ");
		Serial.println(Angle);
		Serial.print("Gain control : ");
		Serial.println(DiagnosticResults.AutomaticGainControl);
		Serial.print("Overcompensation Finished : ");
		Serial.println(DiagnosticResults.OvercompensationFinished);
		Serial.print("CORDIC Overflow : ");
		Serial.println(DiagnosticResults.CORDICOverflow);
		Serial.print("COMPLow : ");
		Serial.println(DiagnosticResults.COMPLow);
		Serial.print("COMPHigh : ");
		Serial.println(DiagnosticResults.COMPHigh);
		Serial.print("Parity Error : ");
		Serial.println(ErrorResults.ParityError);
		Serial.print("Command Invalid : ");
		Serial.println(ErrorResults.CommandInvalid);
		Serial.print("Framing Error : ");
		Serial.println(ErrorResults.FramingError);
	}
}
