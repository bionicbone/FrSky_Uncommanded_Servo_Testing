/*
    Name:       FrSky_Uncommanded_Servo_Testing.ino
    Created:	01/02/2020 07:07:44
    Author:     LT9\Kevin
	MPU:		Teensy v3.2 / v4.0
*/

/*
  FrSky S-Port Telemetry library
  (c) Pawelsky 20180402
  Not for commercial use
*/

// for the ILI9341 display

#include <ili9341_t3n_font_OpenSans.h>
#include <ili9341_t3n_font_ComicSansMS.h>
#include <ili9341_t3n_font_ArialBold.h>
#include <ili9341_t3n_font_Arial.h>
#include <ILI9341_t3n.h>
#include <ILI9341_fonts.h>
#include <PWMServo.h>
#include "SPI.h"
// for Pawelsky's FrSky S.PORT Library
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
// for the Arduino SBUS phaser
#include "SBUS.h"

// Config
const byte NUMBER_OF_CH = 16;							// 8 or 16 channels to scan
const byte NUMBER_OF_RX = 4;							// Max 2 for Teensy 3.2 / Max 6 for Teensy 4.0
#define FrSky_SERIAL SERIAL_7							// SERIAL_3 for Teensy 3.2 / SERIAL_7 for Teensy 4.0
const byte MAX_CHANNEL_INCREASE = 104;			// ~8 per frame, 48 = 6 frames, not less than 10

// Connected Rx Names for Identification, connected to Serial_1, Serial_2, ect...
const String RX_NAMES[] = { "Rx1-X4R(v1)", "Rx2-XM+(v1)" ,"Rx3-X4R(v2)", "Rx4-XM+(v2)" };

// Program options

// Normally ON
#define SERIAL_UPDATE										// Send TFT data to USB Serial
#define UPDATE_DISPLAY									// Activate the TFT display and update
//#define SEND_TELEMETRY									// Activate Telemetry and Send

// Normally OFF
//#define DEBUG_DATA										// Dump Previous & Current Channel Data to USB
//#define NO_RX_ATTACHED								// Allow execution like an Rx is attached to Rx1.
//#define REPORT_ERRORS									// Reports SBUS LostFrame and FailSafe flags
//#define REPORT_ERRORS_BADFRAMES				// Reports Bad Frames (Frame Holds) on the SBUS.


// For the Adafruit shield, these are the default.
#define TFT_DC  9
#define TFT_CS 10
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC);

// a SBUS object, which is on hardware
SBUS sbusRx1(Serial1);
SBUS sbusRx2(Serial2);
SBUS sbusRx3(Serial3);
SBUS sbusRx4(Serial4);

// frsky Telemetry setup
#if defined(SEND_TELEMETRY)
FrSkySportSensorRpm frskyMainStats(FrSkySportSensor::ID5); 		// Main Stats, frameCounter, Rx Number, LoopMillis
FrSkySportSensorRpm frskyRx1(FrSkySportSensor::ID15);			// Rx1 Data - badFramesPercentage * 100, channelsLongestMillis, channelsMaxChange
FrSkySportSensorRpm frskyRx2(FrSkySportSensor::ID16);			// Create RPM sensor with given ID
FrSkySportSensorRpm frskyRx3(FrSkySportSensor::ID17);			// Create RPM sensor with given ID
FrSkySportSensorRpm frskyRx4(FrSkySportSensor::ID18);			// Create RPM sensor with given ID
FrSkySportTelemetry telemetry;									// Create telemetry object without polling
#endif


// channel, fail safe, and lost frames data
uint16_t  channels[4][16];
uint16_t  channelsPrevious[4][16];

// for detecting and timing channel holds and for how long.
unsigned long  channelsStartHoldMillis[4] = { 0 };
unsigned long   channelsMaxHoldMillis[4] = { 0 };
bool channelHoldTriggered[4] = { false };
unsigned long  channelsStartMaxChangeMillis[4][16] = { { 0 } };
unsigned long   channelsMaxChangeMillis[4][8] = { { 0 } };
bool channelMaxChangeTriggered[4][16] = { { false } };
uint16_t channelsMaxChange[4][8] = { { 0 } };
int channelsMaxChangeCounter[4] = { 0 };
bool channelNewMaxChangeTriggered[4][16] = { { false } };


//int totalLoops = 0;										// Simple Counter based on 9ms Loops

// BB_Bits = monitoring SBUS for missing frames, independent of the SBUS lost frame flag.
int totalValidFramesCounter[4] = { 0 };
int badFramesCounter[4] = { 0 };
float badFramesPercentage[4] = { 0 };

// SBUS Speed
int sbusSpeed_uS[4] = { 0 };

// monitoring SBUS fail safe flag as set by Rx
bool failSafe[4] = { false };
int failSafeCounter[4] = { 0 };
bool failSafeDetected[4] = { false };
unsigned long failSafeStartMillis[4] = { 0 };
int failSafeLongestMillis[4] = { 0 };

// monitoring SBUS lost frame flag as set by Rx
bool lostFrame[4] = { false };
int lostFrameCounter[4] = { 0 };
bool lostFrameDetected[4] = { false };
unsigned long lostFrameStartMillis[4] = { 0 };
int lostFrameLongestMillis[4] = { 0 };

bool firstRun = true;								// used to allow everything to stabilize for the first x frames
bool updateDisplay = false;
bool updateSerial = false;

bool longLoop = false;								//if there is a processing delay it corrupts the data 4 frames later
byte longLoopCounter = 0;
bool sbusData = false;

bool tempLostFrameStop = false;
unsigned long tempLostFrameStopMillis = 0;

void setup()
{
	// Set up Serial Monitor
	Serial.begin(115200);
	delay(1); // allow serial to start
	
	tft.begin();
	tft.fillScreen(ILI9341_BLACK);
	tft.setTextColor(ILI9341_YELLOW);
	tft.setTextSize(3);
	tft.setRotation(0);
	tft.setCursor(0, 0);
	tft.print("Rx Monitor");
	tft.setCursor(30, 60);
	tft.print("Starting...");
	
	// begin the SBUS communication
	sbusRx1.begin();
	sbusRx2.begin();
	sbusRx3.begin();
	sbusRx4.begin();

#if defined(SEND_TELEMETRY)
	// begin FrSky Telemetry
	telemetry.begin(FrSkySportSingleWireSerial::FrSky_SERIAL, &frskyMainStats, &frskyRx1, &frskyRx2, &frskyRx3, &frskyRx4);
#endif

	//servo_MovementVisualiser();
}


void loop()
{
	unsigned long loopMillis = millis();

	if (totalValidFramesCounter[0] > 1000) { firstRun = false; } // Allow everything to stabilse

#if!defined(NO_RX_ATTACHED)
	// Cycle through each Rx
	for (int rx = 0; rx < NUMBER_OF_RX; rx++) {

		// Read the correct Serial port for the SBUS and pass to the main code.
		switch (rx) {
		case 0:
			if (sbusRx1.read(&channels[rx][0], &failSafe[rx], &lostFrame[rx])) { do_Stuff(rx); }
			break;
		case 1:
			if (sbusRx2.read(&channels[rx][0], &failSafe[rx], &lostFrame[rx])) { do_Stuff(rx); }
			break;
		case 2:
			if (sbusRx3.read(&channels[rx][0], &failSafe[rx], &lostFrame[rx])) { do_Stuff(rx); }
			break;
		case 3:
			if (sbusRx4.read(&channels[rx][0], &failSafe[rx], &lostFrame[rx])) { do_Stuff(rx); }
			break;
		default:
			// catch nothing
			break;
		}

		// FrSky Telemetery
#if defined(SEND_TELEMETRY)
		rx_Telemetry(rx, millis() - loopMillis);
#endif
	}
#else
	do_Stuff(0);
	// FrSky Telemetery
#if defined(SEND_TELEMETRY)
	rx_Telemetry(0, millis() - loopMillis);
#endif
#endif


	// Need to ensure the loop is no longer than 9ms, if it is start the longLoopCounter
	if (millis() > loopMillis + 9) {
		Serial.print("WARNING: Loop Speed Error @"); Serial.print(millis() - loopMillis); Serial.println("ms");
		longLoop = true;
		longLoopCounter = 0;
	}
	else {
		//Serial.print("Loop Speed "); Serial.print(millis() - loopMillis); Serial.println("ms");
	}

	// Do we need to advance the longLoopCounter?
	if (sbusData == true) {
		if (longLoop == true && longLoopCounter < 15) {
			longLoopCounter++;
		}
		else {
			longLoop = false;
			longLoopCounter = 0;
		}
	}

	// FrSky Telemetery
#if defined(SEND_TELEMETRY)
	rx_Telemetry(99, millis() - loopMillis);				 
#endif

	//totalLoops++; 
	sbusData = false;      // used to control the longLoopCounter
}

// Controls what processes should be exectutes for each Rx
void do_Stuff(int rx) {
	totalValidFramesCounter[rx]++;
	sbusData = true;
	
	check_FailSafe(rx);
	check_LostFrame(rx);
	//if (rx == 0) { check_LostFrame(rx); }

#if defined(DEBUG_DATA)
	debug_Data(rx);
	//if (rx == 0) { debug_Data(rx); }
#endif
	check_BadFrame(rx);
	// Must proccess this last as it updates the previousChannel variable
	check_ChannelSignificantChange(rx);

	//debug_RxUpdate(rx);

	// Only update the display every 1000 frame due to processing time
	if (rx == 0 && (float)totalValidFramesCounter[0] / 1000 == int((float)totalValidFramesCounter[0] / 1000)) { updateSerial = true; }
	if (rx==0 && (float)totalValidFramesCounter[0] / 10000 == int((float)totalValidFramesCounter[0] / 10000)) { updateDisplay = true; }
#if defined (UPDATE_DISPLAY)
	if (updateSerial == true) { Serial_Update(); }
	if (updateDisplay ==true) { display_Update(); }
#endif
	updateSerial = false;
	updateDisplay = false;
}

// Updates the ILI9341 display
// WARNING: Long Process !!
void display_Update() {
	tft.flush();
	tft.clearWriteError();
	tft.fillScreen(ILI9341_BLACK);
	int x = 0; int y = 0;
	for (int rx = 0; rx < NUMBER_OF_RX; rx++) {
		tft.setCursor(x, y);
		tft.setFont(Arial_9);
		tft.setTextColor(ILI9341_GREEN);
		tft.println(RX_NAMES[rx]);
		tft.setTextColor(ILI9341_YELLOW);
		//tft.setTextSize(1);
		tft.setFont(Arial_8);
		tft.print("SF "); tft.print(totalValidFramesCounter[rx]);
		//tft.print(",  SBUS Speed "); tft.print(sbusSpeed_uS[rx]); tft.println("us");
		tft.print(",  BF "); tft.print(badFramesCounter[rx]); tft.print(", BF "); tft.print(badFramesPercentage[rx]); tft.println("%");
		tft.print("FS "); tft.print(failSafeCounter[rx]); 
		tft.print(",  FSM "); tft.print(failSafeLongestMillis[rx]); tft.print("ms, ");
		tft.print("LF "); tft.print(lostFrameCounter[rx]);
		tft.print(",  LFM "); tft.print(lostFrameLongestMillis[rx]); tft.print("ms");
		tft.print(",  CHM "); tft.print(channelsMaxHoldMillis[rx]); tft.println("ms");
		tft.print("SD "); tft.print((float)channelsMaxChange[rx][0] / 2000 * 100); tft.print("% :"); 
		for (int sd = 0; sd < 8; sd++) {
			tft.print((float)channelsMaxChange[rx][sd] / 2000 * 100); tft.print("% :");
			tft.print(channelsMaxChangeMillis[rx][sd]); tft.print("ms, ");
		}
		y += 78;
	}
	tft.flush();
	tft.clearWriteError();
	Serial.println("WARNING: Display Updated expect a speed issue");
}


// Update the USB Serial Monitor
void Serial_Update() {
#if defined(SERIAL_UPDATE)
	Serial.println("");
	for (int rx = 0; rx < NUMBER_OF_RX; rx++) {
		Serial.println(RX_NAMES[rx]);
		Serial.print("SF "); Serial.print(totalValidFramesCounter[rx]);
		//Serial.print(",  SBUS Speed "); Serial.print(sbusSpeed_uS[rx]); Serial.println("us");
		Serial.print(",  BF "); Serial.print(badFramesCounter[rx]); Serial.print(", BF "); Serial.print(badFramesPercentage[rx]); Serial.println("%");
		Serial.print("FS "); Serial.print(failSafeCounter[rx]);
		Serial.print(",  FSM "); Serial.print(failSafeLongestMillis[rx]); Serial.print("ms, ");
		Serial.print("LF "); Serial.print(lostFrameCounter[rx]);
		Serial.print(",  LFM "); Serial.print(lostFrameLongestMillis[rx]); Serial.print("ms");
		Serial.print(",  CHM "); Serial.print(channelsMaxHoldMillis[rx]); Serial.println("ms");
		Serial.print("SD "); 
		
		for (int sd = 0; sd < 8; sd++) {
			Serial.print((float)channelsMaxChange[rx][sd] / 2000 * 100); Serial.print("% :");
			Serial.print(channelsMaxChangeMillis[rx][sd]); Serial.print("ms, ");
		}
		Serial.println("");
	}
#endif
}


// Checks and times Channel Holds or Significant Change in SBUS channel data
void check_ChannelSignificantChange(int rx) {
	for (int ch = 0; ch < NUMBER_OF_CH; ch++) {
		// After many tests its concluded that a "Display Update" creates a significant
		// long loop. Normally longLoopCounters >=3 or <=8 find "significant changes".
		// Given its not the same each time then its better to ignore the next 10 readings.
		// We do this by setting the previous reading to the current reading
		if (longLoopCounter >= 1 && longLoopCounter <= 10) { channelsPrevious[rx][ch] = channels[rx][ch]; }

		if (firstRun == true) {
			channelsPrevious[rx][ch] = channels[rx][ch];
			channelsMaxChange[rx][channelsMaxChangeCounter[rx]] = 0;
			channelsMaxChangeMillis[rx][channelsMaxChangeCounter[rx]] =0;
		}

		// detect channel hold
		if (channels[rx][ch] == channelsPrevious[rx][ch] && channelHoldTriggered[rx] == false && firstRun == false) {
			//Serial.print(RX_NAMES[rx]); Serial.print(" = "); Serial.println("Channel Hold");
			//Serial.print("Pre"); Serial.print(rx+1); Serial.print(" = "); Serial.println(channelsPrevious[rx][ch]);
			//Serial.print("New"); Serial.print(rx+1); Serial.print(" = "); Serial.println(channels[rx][ch]);
			channelHoldTriggered[rx] = true;
			channelsStartHoldMillis[rx] = millis();
		}
		if (channels[rx][ch] != channelsPrevious[rx][ch] && channelHoldTriggered[rx] == true) {
			if (millis() - channelsStartHoldMillis[rx] > channelsMaxHoldMillis[rx]) {
				channelsMaxHoldMillis[rx] = millis() - channelsStartHoldMillis[rx];
			}
			//Serial.print(RX_NAMES[rx]);  Serial.print(" = "); Serial.println("Channel Hold Recovered");
			channelHoldTriggered[rx] = false;
			channelsPrevious[rx][ch] = channels[rx][ch];  // After a hold dont count a significant change
		}

		// detect channel significant change
		if (abs(channels[rx][ch] - channelsPrevious[rx][ch]) > MAX_CHANNEL_INCREASE && channelMaxChangeTriggered[rx][ch] == false) {
	
			if (abs(channels[rx][ch] - channelsPrevious[rx][ch]) > channelsMaxChange[rx][channelsMaxChangeCounter[rx]]) {
				channelsStartMaxChangeMillis[rx][ch] = millis();
				Serial.print("__________________"); Serial.print(RX_NAMES[rx]); Serial.print(" = "); Serial.println("Significant Change");
				Serial.print("__________________Frame"); Serial.print(" = "); Serial.println(totalValidFramesCounter[rx]);
				Serial.print("__________________LongLoop"); Serial.print(" = "); Serial.println(longLoop);
				Serial.print("__________________LongLoopCounter"); Serial.print(" = "); Serial.println(longLoopCounter);
				Serial.print("__________________Pre CH"); Serial.print(ch); Serial.print(" = "); Serial.println(channelsPrevious[rx][ch]);
				Serial.print("__________________New CH"); Serial.print(ch); Serial.print(" = "); Serial.println(channels[rx][ch]);
				Serial.print("__________________New Max Change = "); Serial.println(abs(channels[rx][ch] - channelsPrevious[rx][ch]));
				Serial.print("__________________millis() CH"); Serial.print(ch); Serial.print(" = "); Serial.println(channelsStartMaxChangeMillis[rx][ch]);
				channelsMaxChange[rx][channelsMaxChangeCounter[rx]] = abs(channels[rx][ch] - channelsPrevious[rx][ch]);
				channelNewMaxChangeTriggered[rx][ch] = true;		
			}
			
			channelMaxChangeTriggered[rx][ch] = true;
		}
		
		// has the significant change recovered
		if (channels[rx][ch] != channelsPrevious[rx][ch] && abs(channels[rx][ch] - channelsPrevious[rx][ch]) <= MAX_CHANNEL_INCREASE) {
			
			if (channelNewMaxChangeTriggered[rx][ch] == true) {
				channelsMaxChangeMillis[rx][channelsMaxChangeCounter[rx]] = millis() - channelsStartMaxChangeMillis[rx][ch];
				channelNewMaxChangeTriggered[rx][ch] = false;
				Serial.print(RX_NAMES[rx]); Serial.print(" CH"); Serial.print(ch); Serial.print(" = "); Serial.print("Significant Change Recovered in ");
				Serial.print(channelsMaxChangeMillis[rx][channelsMaxChangeCounter[rx]]); Serial.println("ms");
				if (channelsMaxChangeCounter[rx] < 6) { channelsMaxChangeCounter[rx]++; }
			}
			
			//if (channelMaxChangeTriggered[rx][ch] == true) {
			//	Serial.print(RX_NAMES[rx]); Serial.print(" = "); Serial.println("Significant Change Recovered");
			//}
			channelMaxChangeTriggered[rx][ch] = false;
		}

		channelsPrevious[rx][ch] = channels[rx][ch];
	}
}

//TODO - Remove Temp Code
// Checks the Lost Frmae flag in the SBUS
void check_LostFrame(int rx) {
	// lostFrame

	if (lostFrame[rx] == true && lostFrameDetected[rx] == false) {
		lostFrameStartMillis[rx] = millis();
		lostFrameCounter[rx]++;
#if defined(REPORT_ERRORS)
		Serial.println("");
		Serial.print(RX_NAMES[rx]); Serial.print(" = ");
		Serial.print("lostFrames Reported = "); Serial.println(lostFrameCounter[rx]);
		Serial.println("");
#endif
		lostFrameDetected[rx] = true;
		//// TEMP CODE
		//tempLostFrameStop = true;
		//tempLostFrameStopMillis = millis();
	}
	if (lostFrame[rx] == false && lostFrameDetected[rx] == true) {
		int temp = millis() - lostFrameStartMillis[rx];
		if (temp > lostFrameLongestMillis[rx])  lostFrameLongestMillis[rx] = temp;
#if defined(REPORT_ERRORS)
		Serial.println("");
		Serial.print(RX_NAMES[rx]); Serial.print(" = ");
		Serial.print("lostFrame Recovered = "); Serial.print(temp); Serial.println("ms");
		Serial.println("");
#endif
		lostFrameDetected[rx] = false;
	}
	// TEMP CODE
	//if (tempLostFrameStop == true && millis() - tempLostFrameStopMillis > 2000) {
	//	Serial.println("lostFrame_STOP");
	//	while (true) {}
	//}


}


//TODO - Check Channel 9 if we have 16 channnels
void check_BadFrame(int rx) {
	// Channel Monitoring - Monitor_Channel
	
	//if there is a processing delay it is detected 4 frames later
	if (longLoopCounter >= 1 && longLoopCounter <= 10) { channelsPrevious[rx][0] = channels[rx][0]; }

	// dont count if data is stabilsing
	if (firstRun == true) {
		channelsPrevious[rx][0] = channels[rx][0];
		channelsMaxChange[rx][channelsMaxChangeCounter[rx]] = 0;
		channelsMaxChangeMillis[rx][channelsMaxChangeCounter[rx]] = 0;
	}
	
	uint8_t diff = abs(channels[rx][0] - channelsPrevious[rx][0]);
	if (diff > 10 * (NUMBER_OF_CH / 8)) {
		int badFrames = ((float)(abs(channels[rx][0] - channelsPrevious[rx][0])) / (NUMBER_OF_CH / 2)) - 1;
		badFramesCounter[rx] += badFrames;
		badFramesPercentage[rx] = 100 - (((float)badFramesCounter[rx] / totalValidFramesCounter[rx]) * 100);
#if defined(REPORT_ERRORS_BADFRAMES)
		Serial.print(RX_NAMES[rx]); Serial.print(" = ");
		Serial.print("CH"); Serial.print(0); Serial.print(" = ");
		Serial.print(channels[rx][0]); Serial.print(" vs "); Serial.print(channelsPrevious[rx][0]);
		Serial.print(" = "); Serial.println(channelsPrevious[rx][0] - channels[rx][0]);
		Serial.print("badFrames Calculated = "); Serial.println(badFrames);
#endif
	}
}


// Checks the fail safe flag in the SBUS
void check_FailSafe(int rx) {

	// failSafe
	if (failSafe[rx] == true && failSafeDetected[rx] == false) {
		failSafeStartMillis[rx] = millis();
		failSafeCounter[rx]++;
#if defined(REPORT_ERRORS)
		Serial.print(RX_NAMES[rx]); Serial.print(" = ");
		Serial.print("failSafes Detected = "); Serial.println(lostFrameCounter[rx]);
#endif
		failSafeDetected[rx] = true;
	}
	if (failSafe[rx] == false && failSafeDetected[rx] == true) {
		int temp = millis() - failSafeStartMillis[rx];
		if (temp > failSafeLongestMillis[rx])  failSafeLongestMillis[rx] = temp;
#if defined(REPORT_ERRORS)
		Serial.print(RX_NAMES[rx]); Serial.print(" = ");
		Serial.print("FailSafe Recovered = "); Serial.print(temp); Serial.println("ms");
#endif
		failSafeDetected[rx] = false;
	}
}


//TODO - Changes will not work with only 8 Channels
// Dumps Previous vs Current Channel information to USB serial.
void debug_Data(int rx) {
#if defined(DEBUG_DATA)
	// detect which channels updated
	if (abs(channelsPrevious[rx][0] - channels[rx][0]) != 0) {
		if (abs(channelsPrevious[rx][8] - channels[rx][8]) != 0) {
			Serial.println("++++++++++++++++++++++++++++++++++");
			Serial.println("++++++ ALL CHANNELS CHANGED ++++++");
			Serial.println("++++++++++++++++++++++++++++++++++");
		}
		else {
			Serial.println("");
			Serial.println("CH1-8 updated - CH9-16 repeated");
		}
	}
	if (abs(channelsPrevious[rx][0] - channels[rx][0]) == 0) {
		if (abs(channelsPrevious[rx][8] - channels[rx][8]) == 0) {
			Serial.println("-------------------------------");
			Serial.println("------ ALL CHANNELS HELD ------");
			Serial.println("-------------------------------");
		}
		else {
			Serial.println("CH1-8 repeated - CH9-16 updated");
		}
	}
	
	for (int ch = 0; ch < NUMBER_OF_CH; ch++) {
		Serial.print(RX_NAMES[rx]); Serial.print(" = ");
		Serial.print("CH"); Serial.print(ch+1); Serial.print(" = ");
		Serial.print(channelsPrevious[rx][ch]); Serial.print(" vs "); Serial.print(channels[rx][ch]);
		Serial.print(" = "); Serial.print(channelsPrevious[rx][ch] - channels[rx][ch]);
		Serial.print(" LongLoop "); Serial.println(longLoopCounter);
	}
#endif
}


// Dumps data after each Rx processing
void debug_RxUpdate(int rx) {

	Serial.print(RX_NAMES[rx]); Serial.print(" = ");
	Serial.print(channelsMaxHoldMillis[rx]); Serial.print("ms, ");
	Serial.print((float)channelsMaxChange[rx][channelsMaxChangeCounter[rx]] / 2000 * 100); Serial.print("% "); Serial.print(channelsMaxChangeMillis[rx][channelsMaxChangeCounter[rx]]); Serial.println("ms");

	//Serial.print(RX_NAMES[rx]); Serial.print(" = ");
	//Serial.print("Total Frame Count: "); Serial.println(totalLoops);
	//Serial.print(RX_NAMES[rx]); Serial.print(" = ");
	//Serial.print("Success Rate: "); Serial.print(badFramesPercentage[rx]); Serial.println("% based on repeated frames found");
	//Serial.print(RX_NAMES[rx]); Serial.print(" = ");
	//Serial.print("Success Rate: "); Serial.print(100 - (((float)lostFrameCounter[rx] / totalLoops) * 100)); Serial.println("% based on lost frames reported by Rx");
	//Serial.print(RX_NAMES[rx]); Serial.print(" = ");
	//Serial.print(totalLoops); Serial.print(" : "); Serial.println(badFramesCounter[rx]);
}


// Transmits all the telemetry information for basic Tx monitoring
#if defined(SEND_TELEMETRY)
void rx_Telemetry(byte rx, int loopMillis) {
	// Procedure will populate the telemetry control and then call the send data routine

	// Set RPM/temperature sensor data
	// (set number of blades to 2 in telemetry menu to get correct totalLoops value)

	// ID 5
	frskyMainStats.setData(999,							// Total Frames based on Loop Counter (Rx does not announce)
		rx,												// Rx number, 99 = wait loop
		loopMillis);									// Loop execution time (before delay)

		// ID 15 - Rx1
	frskyRx1.setData(badFramesPercentage[0] * 100,
		channelsMaxChangeMillis[0][channelsMaxChangeCounter[0]],
		channelsMaxChange[0][channelsMaxChangeCounter[0]]);

	// ID 16 - Rx2
	frskyRx2.setData(badFramesPercentage[1] * 100,
		channelsMaxChangeMillis[1][channelsMaxChangeCounter[1]],
		channelsMaxChange[1][channelsMaxChangeCounter[1]]);

	// ID 17 - Rx3
	frskyRx3.setData(badFramesPercentage[2] * 100,
		channelsMaxChangeMillis[2][channelsMaxChangeCounter[2]],
		channelsMaxChange[2][channelsMaxChangeCounter[2]]);

	// ID 18 - Rx4
	frskyRx4.setData(badFramesPercentage[3] * 100,
		channelsMaxChangeMillis[3][channelsMaxChangeCounter[3]],
		channelsMaxChange[3][channelsMaxChangeCounter[3]]);

	telemetry.send();
}
#endif

void servo_MovementVisualiser() {
	PWMServo myservo;
	myservo.attach(3, 1000, 2000);
	
	tft.flush();
	tft.clearWriteError();
	tft.fillScreen(ILI9341_BLACK);
	tft.setTextColor(ILI9341_GREEN);
	tft.setTextSize(2);
	tft.setCursor(0, 0);
	tft.println("Servo Visualiser");
	tft.println("");
	delay(1000);
	tft.println("Test Full Scale");
	tft.println("");
	for (int i = 0; i < 3; i++) {
		myservo.write(170);
		delay(500);
		myservo.write(90);
		delay(500);
	}
	delay(1000);

	for (int f = 0; f < 50; f++) {
		tft.flush();
		tft.clearWriteError();
		tft.fillScreen(ILI9341_BLACK);
		tft.setTextColor(ILI9341_GREEN);
		tft.setTextSize(2);
		tft.setCursor(0, 0);
		tft.println("Servo Visualiser");
		tft.println("");
		tft.println("Test Full Scale for x ms");
		tft.println("");
		tft.println(f * 9); tft.println("ms");
		for (int i = 0; i < 3; i++) {
			myservo.write(170);
			delay(f * 9);
			myservo.write(90);
			delay(500);
		}
		delay(1000);
	}
}