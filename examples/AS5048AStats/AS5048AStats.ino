#include <SPI.h>
#include <AS5048A.h>
#include <RunningStats.h>

//Intended for RP2040
//Changes will need to be made to the SPI calls for different processors
//Need RunningStats from https://github.com/freudenthal/ArduinoRunningStats

const uint32_t SerialUSBSpeed = 115200;
const uint8_t AS5048A_CSPin = 1;
const uint8_t AS5048A_TXPin = 3;
const uint8_t AS5048A_RXPin = 4;
const uint8_t AS5048A_CLKPin = 2;
const uint32_t AS5048A_ReadDelay = 100; //80 effective minimum by datasheet
const uint32_t ReportDelay = 200000;
const bool AS5048A_Verbose = false;
const double AngleMax = 16384.0;
const double RadiansToDegrees = 180.0/PI;

uint32_t StatisticsCountMax = 256;

uint32_t AS5048A_LastRead = 0;
uint32_t LastReport = 0;
AS5048A Encoder(AS5048A_CSPin);

uint16_t MagnitudeRaw = 0;
uint16_t AngleRaw = 0;
int16_t AngleSigned = 0;
AS5048A::DiagnosticData DiagnosticResults;
AS5048A::ErrorData ErrorResults;
RunningStats MagnitudeStats;
RunningStats AngleRawStats;
RunningStats XStats;
RunningStats YStats;

double MagnitudeMean = 0.0;
double MagnitudeSTD = 0.0;
double AngleMean = 0.0;
double AngleSTD = 0.0;
double AngleRadians = 0.0;
double AngleRawMean = 0.0;
double AngleRawSTD = 0.0;
double XValue = 0.0;
double YValue = 0.0;
double XMean = 0.0;
double XSTD = 0.0;
double YMean = 0.0;
double YSTD = 0.0;

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
		MagnitudeRaw = Encoder.GetMagnitude();
		AngleRaw = Encoder.GetAngle();
		AngleRadians = 2*PI*((double)(AngleRaw)/AngleMax) - PI;
		XValue = cos(AngleRadians);
		YValue = sin(AngleRadians);
		MagnitudeStats.Push((double)MagnitudeRaw);
		AngleRawStats.Push((double)AngleRaw);
		XStats.Push(XValue);
		YStats.Push(YValue);
		if (MagnitudeStats.NumDataValues() > StatisticsCountMax)
		{
			MagnitudeMean = MagnitudeStats.Mean();
			MagnitudeSTD = MagnitudeStats.StandardDeviation();
			XMean = XStats.Mean();
			XSTD = XStats.StandardDeviation();
			YMean = YStats.Mean();
			YSTD = YStats.StandardDeviation();
			AngleRawMean = AngleRawStats.Mean();
			AngleRawSTD = AngleRawStats.StandardDeviation();
			MagnitudeStats.Clear();
			XStats.Clear();
			YStats.Clear();
			AngleRawStats.Clear();
			AngleMean = atan2(YMean,XMean);
			double YMinus = atan2(YMean - YSTD,XMean);
			double YPlus = atan2(YMean + YSTD,XMean);
			double XMinus = atan2(YMean,XMean - XSTD);
			double XPlus = atan2(YMean,XMean + XSTD);
			double DeviantMean = (YMinus + YPlus + XMinus + XPlus)/4.0;
			YMinus = YMinus - DeviantMean;
			YPlus = YPlus - DeviantMean;
			XMinus = XMinus - DeviantMean;
			XPlus = XPlus - DeviantMean;
			YMinus = YMinus * YMinus;
			YPlus = YPlus * YPlus;
			XMinus = XMinus * XMinus;
			XPlus = XPlus * XPlus;
			AngleSTD = sqrt( YMinus + YPlus + XMinus + XPlus )/4.0;
			//double YMeanSq = pow(YMean,2);
			//double XMeanSq = pow(XMean,2);
			//double Dividend = (YMeanSq*pow(XSTD,2)) + (XMeanSq*pow(YSTD,2));
			//double Divisor = pow(YMeanSq+XMeanSq,2);
			//AngleSTD = sqrt( Dividend/Divisor );
		}
	}
	if ( (micros() - LastReport) > ReportDelay)
	{
		LastReport = micros();
		Serial.println("**********");
		Serial.print("Magnitude : ");
		Serial.print(MagnitudeMean);
		Serial.print(" +/- ");
		Serial.println(MagnitudeSTD);
		Serial.print("Angle Raw: ");
		Serial.print(AngleRawMean);
		Serial.print(" +/- ");
		Serial.println(AngleRawSTD);
		Serial.print("Angle : ");
		Serial.print(AngleMean * RadiansToDegrees,4);
		Serial.print(" +/- ");
		Serial.println(AngleSTD * RadiansToDegrees,4);
		Serial.print("X : ");
		Serial.print(XMean,5);
		Serial.print(" +/- ");
		Serial.println(XSTD,5);
		Serial.print("Y : ");
		Serial.print(YMean,5);
		Serial.print(" +/- ");
		Serial.println(YSTD,5);
		DiagnosticResults = Encoder.GetDiagnostics();
		ErrorResults = Encoder.GetAndClearErrors();
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
