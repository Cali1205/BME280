/*
BME280.h - Header file for the BME280 Barometric Pressure Sensor Arduino Library.
Copyright (C) 2012 Love Electronics Ltd (loveelectronics.com)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 Datasheet for BME280:
 http://www.bosch-sensortec.com/content/language1/downloads/BST-BMP180-DS000-07.pdf

*/

#ifndef BME280_h
#define BME280_h

#include <inttypes.h>

#include "../../SmingCore/Wire.h"

#define BME280_Address 0x76

#define ChipIdData 0x60

#define ControlInstruction_MeasureTemperature 0xFA

#define ControlInstruction_MeasurePressure 0xF7

#define ControlInstruction_MeasureHumidity 0xFD
#define ControlInstruction_ControlHumidity 0xF2



#define Reg_ChipId 0xD0
#define Reg_Version 0xD1
#define Reg_CAL26 0xE1 // R calibration stored in 0xE1-0xF0
#define Reg_Control 0xF4
#define Reg_Config 0xF5
#define Reg_CalibrationStart 0x88
#define Reg_CalibrationEnd 0xE7
#define Reg_AnalogConverterOutMSB 0xF6
#define Reg_SoftReset 0xE0
#define SoftResetInstruction 0xB6

#define ErrorCode_1 "Entered sample resolution was invalid. See datasheet for details."
#define ErrorCode_1_Num 1

#define BME280_Mode_UltraLowPower		0
#define BME280_Mode_Standard			1
#define BME280_Mode_HighResolution		2
#define BME280_Mode_UltraHighResolution	3

class BME280
{
	public:
	  BME280();

	  void Initialize(void);

	  int GetUncompensatedTemperature();
	  float CompensateTemperature(int uncompensatedTemperature);
	  
	  long GetUncompensatedPressure();
	//  long CompensatePressure(long uncompensatedPressure);
	  
	  float GetTemperature();
	//  long GetPressure();
	//  float GetHumidity();
	  
	  //float GetAltitude(float currentSeaLevelPressureInPa);
	  
	  void SoftReset();
	  uint8_t SetResolution(uint8_t sampleResolution, bool oversample);

	  void PrintCalibrationData();

	  uint8_t EnsureConnected();
	  uint8_t IsConnected;
	  const char* GetErrorText(int errorCode);
	protected:
	  void Write(int address, int byte);
	  uint8_t* Read(int address, int length);
	  void Read2(int address, int length, uint8_t buffer[]);
	private:
		uint8_t OversamplingSetting;
        bool Oversample;
		int16_t ConversionWaitTimeMs;
		int16_t LastTemperatureData;
		int16_t LastTemperatureTime;
		int16_t AcceptableTemperatureLatencyForPressure;


		uint16_t BME280_REGISTER_DIG_T1;
		int16_t BME280_REGISTER_DIG_T2;
		int16_t BME280_REGISTER_DIG_T3;

		uint16_t BME280_REGISTER_DIG_P1;
		int16_t BME280_REGISTER_DIG_P2;
		int16_t BME280_REGISTER_DIG_P3;
		int16_t BME280_REGISTER_DIG_P4;
		int16_t BME280_REGISTER_DIG_P5;
		int16_t BME280_REGISTER_DIG_P6;
		int16_t BME280_REGISTER_DIG_P7;
		int16_t BME280_REGISTER_DIG_P8;
		int16_t BME280_REGISTER_DIG_P9;

		uint8_t BME280_REGISTER_DIG_H1;
		int16_t BME280_REGISTER_DIG_H2;
		int8_t BME280_REGISTER_DIG_H3;
		int16_t BME280_REGISTER_DIG_H4;
		int16_t BME280_REGISTER_DIG_H5;
		int8_t BME280_REGISTER_DIG_H6;

        uint8_t buffer[64];
};
#endif
