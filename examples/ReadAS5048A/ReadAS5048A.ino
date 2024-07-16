#include <SPI.h>
#include <AS5048A.h>

//Intended for RP2040
//Changes will need to be made to the SPI calls for different processors

const uint32_t SerialUSBSpeed = 115200;
const uint8_t AS5048A_CSPin = 1;
const uint8_t AS5048A_TXPin = 3;
const uint8_t AS5048A_RXPin = 4;
const uint8_t AS5048A_CLKPin = 2;
const uint8_t AS5048A_ReadDelay = 80;
const bool AS5048A_Verbose = false;

uint32_t AS5048A_LastRead = 0;
AS5048A Encodder(AS5048A_CSPin);

uint16_t Magnitude = 0;
uint16_t Angle = 0;
AS5048A::DiagnosticData DiagnosticResults;
AS5048A::ErrorFlags ErrorResults;

void setup()
{
	SPI.setRX(AS5048A_RXPin);
	SPI.setSCK(AS5048A_CLKPin);
	SPI.setTX(AS5048A_TXPin);
	SPI.begin(false);
	Serial.begin(SerialUSBSpeed);
	Encodder.Begin();
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
		DiagnosticResults = GetDiagnostics();
		ErrorResults = GetAndClearErrors();
		Serial.println("**********");
		Serial.print("Magnitude : ");
		Serial.println(Magnitude);
		Serial.print("Angle : ");
		Serial.println(Angle);
		Serial.print("Gain control : ");
		Serial.println(DiagnosticResults.AutomaticGainControl);
		Serial.print("Gain control : ");
		Serial.println(DiagnosticResults.AutomaticGainControl);
		Serial.print("Gain control : ");
		Serial.println(DiagnosticResults.AutomaticGainControl);
		Serial.print("Gain control : ");
		Serial.println(DiagnosticResults.AutomaticGainControl);
	}
}
