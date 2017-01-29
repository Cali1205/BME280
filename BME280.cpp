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

#include "BME280.h"
#include "Arduino.h"
//#include <math.h>

BME280::BME280()
{
  ConversionWaitTimeMs = 5;
  OversamplingSetting = 0;
  Oversample = false;

  LastTemperatureTime = -1000;
  LastTemperatureData = 0;

  AcceptableTemperatureLatencyForPressure = 1000;

  SetResolution(BME280_Mode_Standard, false);
}

uint8_t BME280::EnsureConnected()
{
	Read2(Reg_ChipId, 1, buffer);

	if(buffer[0] == ChipIdData)
		IsConnected = 1;
	else
		IsConnected = 0;

	return buffer[0];//IsConnected;
}

void BME280::Initialize()
{
	Read2(Reg_CalibrationStart, Reg_CalibrationEnd - Reg_CalibrationStart + 2, buffer);
	// This data is in Big Endian format from the BME280.
    BME280_REGISTER_DIG_T1 = (buffer[0] << 8) | buffer[1];
    BME280_REGISTER_DIG_T2 = (buffer[2] << 8) | buffer[3];
    BME280_REGISTER_DIG_T3 = (buffer[4] << 8) | buffer[5];

    BME280_REGISTER_DIG_P1 = (buffer[6] << 8) | buffer[7];
    BME280_REGISTER_DIG_P2 = (buffer[8] << 8) | buffer[9];
    BME280_REGISTER_DIG_P3 = (buffer[10] << 8) | buffer[11];
    BME280_REGISTER_DIG_P4 = (buffer[12] << 8) | buffer[13];
    BME280_REGISTER_DIG_P5 = (buffer[14] << 8) | buffer[15];
    BME280_REGISTER_DIG_P6 = (buffer[16] << 8) | buffer[17];
    BME280_REGISTER_DIG_P7 = (buffer[18] << 8) | buffer[19];
    BME280_REGISTER_DIG_P8 = (buffer[20] << 8) | buffer[21];
	BME280_REGISTER_DIG_P9 = (buffer[22] << 8) | buffer[23];

	BME280_REGISTER_DIG_H1 = (buffer[24] << 8) | buffer[25];
    BME280_REGISTER_DIG_H2 = (buffer[26] << 8) | buffer[27];
    BME280_REGISTER_DIG_H3 = (buffer[28] << 8) | buffer[29];
    BME280_REGISTER_DIG_H4 = (buffer[30] << 8) | buffer[31];
    BME280_REGISTER_DIG_H5 = (buffer[32] << 8) | buffer[33];
	BME280_REGISTER_DIG_H6 = (buffer[34] << 8) | buffer[35];
}

void BME280::PrintCalibrationData()
{
	Serial.print("T1:\t"); Serial.println(BME280_REGISTER_DIG_T1);
	Serial.print("T2:\t"); Serial.println(BME280_REGISTER_DIG_T2);
	Serial.print("T3:\t"); Serial.println(BME280_REGISTER_DIG_T3);

	Serial.print("P1:\t"); Serial.println(BME280_REGISTER_DIG_P1);
	Serial.print("P2:\t"); Serial.println(BME280_REGISTER_DIG_P2);
	Serial.print("P3:\t"); Serial.println(BME280_REGISTER_DIG_P3);
	Serial.print("P4:\t"); Serial.println(BME280_REGISTER_DIG_P4);
	Serial.print("P5:\t"); Serial.println(BME280_REGISTER_DIG_P5);
	Serial.print("P6:\t"); Serial.println(BME280_REGISTER_DIG_P6);
	Serial.print("P7:\t"); Serial.println(BME280_REGISTER_DIG_P7);
	Serial.print("P8:\t"); Serial.println(BME280_REGISTER_DIG_P8);
	Serial.print("P9:\t"); Serial.println(BME280_REGISTER_DIG_P9);

	Serial.print("H1:\t"); Serial.println(BME280_REGISTER_DIG_H1);
	Serial.print("H2:\t"); Serial.println(BME280_REGISTER_DIG_H2);
	Serial.print("H3:\t"); Serial.println(BME280_REGISTER_DIG_H3);
	Serial.print("H4:\t"); Serial.println(BME280_REGISTER_DIG_H4);
	Serial.print("H5:\t"); Serial.println(BME280_REGISTER_DIG_H5);
	Serial.print("H6:\t"); Serial.println(BME280_REGISTER_DIG_H6);

}

int BME280::GetUncompensatedTemperature()
{
    // Instruct device to perform a conversion.
    Write(Reg_Control, ControlInstruction_MeasureTemperature);
    // Wait for the conversion to complete.
    delay(5);
    Read2(Reg_AnalogConverterOutMSB, 2, buffer);
    int value = (buffer[0] << 8) | buffer[1];
    return value;
}

long BME280::GetUncompensatedPressure()
{
    long pressure = 0;
    int loops = Oversample ? 3 : 1;

    for (int i = 0; i < loops; i++)
    {
        // Instruct device to perform a conversion, including the oversampling data.
        uint8_t CtrlByte = ControlInstruction_MeasurePressure + (OversamplingSetting << 6);
        Write(Reg_Control, CtrlByte);
        // Wait for the conversion
        delay(ConversionWaitTimeMs);
        // Read the conversion data.
        uint8_t buffer[3];
		Read2(Reg_AnalogConverterOutMSB, 3, buffer);

        // Collect the data (and push back the LSB if we are not sampling them).
        pressure = ((((long)buffer[0] <<16) | ((long)buffer[1] <<8) | ((long)buffer[2])) >> (8-OversamplingSetting));
    }
    return pressure / loops;
}


//BME280_S32_t t_fine;
//BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T)
//{
//BME280_S32_t var1, var2, T;
//var1 = ((((adc_T>>3) – ((BME280_S32_t)dig_T1<<1))) * ((BME280_S32_t)dig_T2)) >> 11;
//var2 = (((((adc_T>>4) – ((BME280_S32_t)dig_T1)) * ((adc_T>>4) – ((BME280_S32_t)dig_T1))) >> 12) *
//((BME280_S32_t)dig_T3)) >> 14;
//t_fine = var1 + var2;
//T = (t_fine * 5 + 128) >> 8;
//return T;
//}
//The data type “BME280_S32_t” should define a 32 bit signed integer variable type and can
//usually be defined as “long signed int”.
//The data type “BME280_U32_t” should define a 32 bit unsigned integer variable type and can
//usually be defined as “long unsigned int”.

float BME280::CompensateTemperature(int uncompensatedTemperature)
{

	int temperature;
	int x2;
	long x1;
	x1 = (((((long)uncompensatedTemperature>>3) - ((long)BME280_REGISTER_DIG_T1<<1))) * ((long)BME280_REGISTER_DIG_T2)) >> 11;
	x2 = ((((((long)uncompensatedTemperature>>4) - ((long)BME280_REGISTER_DIG_T1)) * (((long)uncompensatedTemperature>>4) - ((long)BME280_REGISTER_DIG_T1))) >> 12) * ((long)BME280_REGISTER_DIG_T3)) >> 14;
	long t_fine = x1 + x2;
	temperature = (t_fine * 5 + 128) >> 8;  /* temperature in 0.1 deg C*/

	float fTemperature = temperature;
	//fTemperature /= 10.0;

	// Record this data because it is required by the pressure algorithem.
	//LastTemperatureData = param_b5;
	//LastTemperatureTime = millis();

	return fTemperature;


//	int temperature;
//    int x2;
//	long x1;
//	x1 = (((long)uncompensatedTemperature - (long)Calibration_AC6) * (long)Calibration_AC5) >> 15;
//    x2 = ((long)Calibration_MC << 11) / (x1 + Calibration_MD);
//    int param_b5 = x1 + x2;
//    temperature = (int)((param_b5 + 8) >> 4);  /* temperature in 0.1 deg C*/
//    float fTemperature = temperature;
//	fTemperature /= 10.0;
//
//    // Record this data because it is required by the pressure algorithem.
//    LastTemperatureData = param_b5;
//    LastTemperatureTime = millis();
//
//    return fTemperature;
}
//
//long BME280::CompensatePressure(long uncompensatedPressure)
//{
//	int msSinceLastTempReading = millis() - LastTemperatureTime;
//    // Check to see if we have old temperature data.
//    if (msSinceLastTempReading > AcceptableTemperatureLatencyForPressure)
//        GetTemperature(); // Refresh the temperature.
//
//    // Data from the BME280 datasheet to test algorithm.
//    /*OversamplingSetting = 0;
//    uncompensatedPressure = 23843;
//    LastTemperatureData = 2399;
//    Calibration_AC1 = 408;
//    Calibration_AC2 = -72;
//    Calibration_AC3 = -14383;
//    Calibration_AC4 = 32741;
//    Calibration_AC5 = 32757;
//    Calibration_AC6 = 23153;
//    Calibration_B1 = 6190;
//    Calibration_B2 = 4;
//    Calibration_MB = -32767;
//    Calibration_MC = -8711;
//    Calibration_MD = 2868;*/
//
//    // Algorithm taken from BME280 datasheet.
//    long b6 = LastTemperatureData - 4000;
//    long x1 = (Calibration_B2 * (b6 * b6 >> 12)) >> 11;
//    long x2 = Calibration_AC2 * b6 >> 11;
//    long x3 = x1 + x2;
//    long b3 = ((Calibration_AC1 * 4 + x3) << OversamplingSetting) + 2;
//    b3 = b3 >> 2;
//    x1 = Calibration_AC3 * b6 >> 13;
//    x2 = (Calibration_B1 * (b6 * b6 >> 12)) >> 16;
//    x3 = ((x1 + x2) + 2) >> 2;
//    long b4 = Calibration_AC4 * (x3 + 32768) >> 15;
//    unsigned long b7 = (((uncompensatedPressure - b3)) * (50000 >> OversamplingSetting));
//    long p;
//    if (b7 < 0x80000000)
//	{
//		p = ((b7 * 2) / b4);
//	}
//    else
//	{
//        p = ((b7 / b4) * 2);
//	}
//
//    x1 = (p >> 8) * (p >> 8);
//    x1 = (x1 * 3038) >> 16;
//    x2 = (-7357 * p) >> 16;
//    p = p + ((x1 + x2 + 3791) >> 4);
//
//    return p;
//}

void BME280::SoftReset()
{
    Write(Reg_SoftReset, SoftResetInstruction);
    delay(100);
}

float BME280::GetTemperature()
{
    return CompensateTemperature(GetUncompensatedTemperature());
}

//long BME280::GetPressure()
//{
//    return CompensatePressure(GetUncompensatedPressure());
//}

/*float BME280::GetAltitude(float currentSeaLevelPressureInPa)
{
    // Get pressure in Pascals (Pa).
    float pressure = GetPressure();
    // Calculate altitude from sea level.
    float altitude = 44330.0 * (1.0 - powf(pressure / currentSeaLevelPressureInPa, (float)0.1902949571836346));
    return altitude;
}*/

uint8_t BME280::SetResolution(uint8_t sampleResolution, bool oversample)
{
    OversamplingSetting = sampleResolution;
    Oversample = oversample;
    switch (sampleResolution)
    {
        case 0:
            ConversionWaitTimeMs = 5;
            break;
        case 1:
            ConversionWaitTimeMs = 8;
            break;
        case 2:
            ConversionWaitTimeMs = 14;
            break;
        case 3:
            ConversionWaitTimeMs = 26;
            break;
        default:
            return ErrorCode_1_Num;
    }
}

void BME280::Write(int address, int data)
{
  Wire.beginTransmission(BME280_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

void BME280::Read2(int address, int length, uint8_t buffer[])
{
  Wire.beginTransmission(BME280_Address);
  Wire.write(address);
  Wire.endTransmission();

  Wire.beginTransmission(BME280_Address);
  Wire.requestFrom(BME280_Address, length);

  while(Wire.available())
  {
	  for(uint8_t i = 0; i < length; i++)
	  {
		  buffer[i] = Wire.read();
	  }
  }
  Wire.endTransmission();
}

const char* BME280::GetErrorText(int errorCode)
{
	if(ErrorCode_1_Num == 1)
		return (const char*)ErrorCode_1;

	return (const char*)"Error not defined.";
}
