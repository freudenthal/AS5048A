#ifndef __ARDUINO_AS5048A
#define __ARDUINO_AS5048A

#include <Arduino.h>
#include <SPI.h>

class AS5048A
{
	public:
		enum class CommandRegisters : uint16_t
		{
			NOP = 0x000, //R No operation dummy
			ClearError = 0x0001, //R Read and clear errors
			ProgrammingControl = 0x0003, //RW Programming control register, see datasheet
			ZeroPositionHighByte = 0x0016, //RW High byte half of zero position
			ZeroPositionLowByte = 0x0017, //RW Low byte half of zero position
			DiagnosticAGC = 0x3FFD, //R Diagnostics and gain control setting
			Magnitude = 0x3FFE, //R Magnitude of CORDIC output
			Angle = 0x3FFF, //R Angle output after zero offset
		};
		enum class ErrorFlags : uint8_t
		{
			FramingError = 0,
			CommandInvalid = 1,
			ParityError = 2,
		};
		enum class DiagnosticFlags : uint8_t
		{
			OffsetCompensationFinished = 0,
			CORDICOverflow = 1,
			COMPLow = 2, //Too much magnetic field
			COMPHigh = 3, //Too little magnetic field
		}
		struct DiagnosticData
		{
			bool OvercompensationFinished;
			bool CORDICOverflow;
			bool COMPLow;
			bool COMPHigh;
			uint8_t AutomaticGainControl; //0 is high field, 255 is low field
		};
		struct ErrorFlags
		{
			bool ParityError;
			bool CommandInvalid;
			bool FramingError;
		};
		AS5048A(uint8_t CSPin);
		begin();
		uint16_t GetZeroPosition();
		void SetZeroPosition(uint16_t ZeroPositionToSet);
		void SetZeroPosition();
		uint16_t GetMagnitude();
		uint16_t GetAngle();
		DiagnosticData GetDiagnostics();
		ErrorFlags GetAndClearErrors();
		SPISettings* GetSPISettings();
		bool GetVerbose();
		void SetVerbose(bool VerboseToSet);
	private:
		SPISettings ConnectionSettings;
		uint8_t CSPin;
		bool Verbose;
		bool IsError;
		uint8_t CalculateParity(uint16_t Value);
		uint16_t SendCommand(CommandRegisters Command, bool IsRead);
}

#endif