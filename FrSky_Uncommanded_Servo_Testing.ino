/*
    Name:       FrSky_Uncommanded_Servo_Testing.ino
    Created:	01/02/2020 07:07:44
    Author:     LT9\Kevin
	MPU:		Teensy v3.2
*/

/*
  FrSky S-Port Telemetry library
  (c) Pawelsky 20180402
  Not for commercial use
*/

#include <FrSkySportTelemetry.h>
#include <FrSkySportSingleWireSerial.h>
#include <FrSkySportSensorXjt.h>
#include <FrSkySportSensorVario.h>
#include <FrSkySportSensorSp2uart.h>
#include <FrSkySportSensorRpm.h>
#include <FrSkySportSensorGps.h>
#include <FrSkySportSensorFlvss.h>
#include <FrSkySportSensorFcs.h>
#include <FrSkySportSensorAss.h>
#include <FrSkySportSensor.h>
#include <FrSkySportPolling.h>
#include <FrSkySportDecoder.h>
#include "SBUS.h"
//#define DEBUG_DATA
#define REPORT_ERRORS

// a SBUS object, which is on hardware
// serial port 1
SBUS x4r(Serial1);

FrSkySportSensorFlvss flvss1;							// Create FLVSS sensor with default ID
FrSkySportSensorRpm rpm;								// Create RPM sensor with default ID
FrSkySportSensorRpm rpm2(FrSkySportSensor::ID15);       // Create RPM sensor with given ID
FrSkySportSensorRpm rpm3(FrSkySportSensor::ID16);       // Create RPM sensor with given ID
FrSkySportSensorRpm rpm4(FrSkySportSensor::ID17);       // Create RPM sensor with given ID
FrSkySportSensorSp2uart sp2uart;						// Create SP2UART Type B sensor with default ID
FrSkySportTelemetry telemetry;							// Create telemetry object without polling
//FrSkySportSensorAss ass;                              // Create ASS sensor with default ID
//FrSkySportSensorFcs fcs;                              // Create FCS-40A sensor with default ID (use ID8 for FCS-150A)
//FrSkySportSensorGps gps;                              // Create GPS sensor with default ID
//FrSkySportSensorVario vario;                          // Create Variometer sensor with default ID


// channel, fail safe, and lost frames data
uint16_t channels[16];
int channelsCounter = 0;
int channelsDetected[16];
unsigned long channelsStartMillis = 0;
int channelsLongestMillis = 0;

bool failSafe = false;
int failSafeCounter = 0;
bool failSafeDetected = false;
unsigned long failSafeStartMillis = 0;
int failSafeLongestMillis = 0;

bool lostFrame = false;
int lostFrameCounter = 0;
bool lostFrameDetected = false;
unsigned long lostFrameStartMillis = 0;
int lostFrameLongestMillis = 0;


void setup()
{
	// begin the SBUS communication
	x4r.begin();

	telemetry.begin(FrSkySportSingleWireSerial::SERIAL_2, &flvss1, &rpm, &rpm2, &rpm3, &rpm4, &sp2uart);
}

void loop()
{
	// look for a good SBUS packet from the receiver
	if (x4r.read(&channels[0], &failSafe, &lostFrame)) {
#if defined(DEBUG_DATA)
		Serial.print("failSafe = "); Serial.println(failSafe);
		Serial.print("lostFrame = "); Serial.println(lostFrame);
		for (int i = 0; i < 16; i++) {
			Serial.print("CH"); Serial.print(i); Serial.print(" = "); Serial.println(channels[i]);
		}
#endif
#if defined(REPORT_ERRORS)
		// failSafe
		if (failSafe == true && failSafeDetected == false) {
				failSafeStartMillis = millis();
				failSafeCounter++;
				Serial.print("failSafes Detected = "); Serial.println(lostFrameCounter);
				failSafeDetected = true;
		}
		if (failSafe == false && failSafeDetected == true) {
			int temp = millis() - failSafeStartMillis;
			if (temp > failSafeLongestMillis)  failSafeLongestMillis = temp;
			Serial.print("FailSafe Recovered = "); Serial.print(temp); Serial.println("ms");
			failSafeDetected = false;
		}

		// lostFrame
		if (lostFrame == true && lostFrameDetected == false) {
			lostFrameStartMillis = millis();
			lostFrameCounter++;
			Serial.print("lostFrames Detected = "); Serial.println(lostFrameCounter);
			lostFrameDetected = true;
		}
		if (lostFrame == false && lostFrameDetected == true) {
			int temp = millis() - lostFrameStartMillis;
			if (temp > lostFrameLongestMillis)  lostFrameLongestMillis = temp;
			Serial.print("lostFrame Recovered = "); Serial.print(temp); Serial.println("ms");
			lostFrameDetected = false;
		}


		// channels
		for (int ch = 0; ch < 16; ch++) {
			if (channels[ch] != 992 && channelsDetected[ch] == false) {
				channelsCounter++;
				channelsStartMillis = millis();
				Serial.print("Channel Error Detected = "); Serial.println(channelsCounter);
				Serial.print("Channel "); Serial.print(ch); Serial.print(" incorrect at "); Serial.println(channels[ch]);
				channelsDetected[ch] = true;
			}
			if (channels[ch] == 992 && channelsDetected[ch] == true) {
				int temp = millis() - lostFrameStartMillis;
				if (temp > channelsLongestMillis)  channelsLongestMillis = temp;
				Serial.print("channels Recovered = "); Serial.print(temp); Serial.println("ms");
				channelsDetected[ch] = false;
			}
		}

#endif
	}

	rx_Telemetry();

}

void rx_Telemetry() {
	// Procedure will populate the telemetry control and then call the send data routine
	// Several controls are not used and are commented out to save memory / timing.

	//Send the telemetry data, note that the data will only be sent for sensors
	//that are being polled at given moment
	float cell1 = 3.01;
	float cell2 = 3.02;
	//int ambientTemp = 20;
	//int canopyTemp = 35;
	//int engineTemp = 140;
	int mainRPMSensorDetectedRPM = 14500;
	int clutchRPMSensorDetectedRPM = 14400;
	int clutchFullyEngagedRPM = 8995;
	//clutchFullyEngaged = true;
	int error = 99;
	//int error1 = 998;
	float reg = 28.1;
	float bec = 6.9;

	// Set LiPo voltage sensor (FLVSS) data (we use two sensors to simulate 8S battery 
	// (set Voltage source to Cells in menu to use this data for battery voltage)
	// (each cell must be above 0.5v to be counted)
	// (set any cell to 0.01 to stop transmission)
	flvss1.setData(cell1, cell2, 0.00, 0.00, 0.00, 0.00);  // Cell voltages in volts (cells 1-6)
	//flvss2.setData(4.13, 4.14);                          // Cell voltages in volts (cells 7-8) - Not Used

	// Set RPM/temperature sensor data
	// (set number of blades to 2 in telemetry menu to get correct rpm value)
	rpm.setData(mainRPMSensorDetectedRPM,		// ID 5
		failSafeCounter,			
		failSafeLongestMillis);			
	rpm2.setData(clutchRPMSensorDetectedRPM,	// ID 15
		lostFrameCounter,			
		lostFrameLongestMillis);				
	rpm3.setData(error,							// ID 16
		channelsCounter,						
		channelsLongestMillis);					
	rpm4.setData(clutchFullyEngagedRPM,			// Temporary
		999,					// Temporary
		999);					// Temporary

	// Set SP2UART sensor data
	// (values from 0.0 to 3.3 are accepted)
	sp2uart.setData(reg,	// ADC3 voltage in volts
					bec);	// ADC4 voltage in volts

// Set variometer data
// (set Variometer source to VSpd in menu to use the vertical speed data from this sensor for variometer).
//vario.setData(250.5,  // Altitude in meters (can be negative)
//              -1.5);  // Vertical speed in m/s (positive - up, negative - down)

// Set airspeed sensor (ASS) data
//ass.setData(76.5);	// Airspeed in km/h

// Set current/voltage sensor (FCS) data
// (set Voltage source to FAS in menu to use this data for battery voltage,
//  set Current source to FAS in menu to use this data for current readins)
//fcs.setData(25.3,		// Current consumption in amps
//            12.6);	// Battery voltage in volts

// Set GPS data
//gps.setData(48.858289, 2.294502,  // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
//            245.5,                // Altitude in m (can be negative)
//            100.0,                // Speed in m/s
//            90.23,                // Course over ground in degrees (0-359, 0 = north)
//            14, 9, 14,            // Date (year - 2000, month, day)
//            12, 00, 00);          // Time (hour, minute, second) - will be affected by timezone setings in your radio
	telemetry.send();
}
