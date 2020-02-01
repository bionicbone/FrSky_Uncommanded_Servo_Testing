/*
    Name:       FrSky_Uncommanded_Servo_Testing.ino
    Created:	01/02/2020 07:07:44
    Author:     LT9\Kevin
*/

#include "SBUS.h"
//#define DEBUG_DATA
#define REPORT_ERRORS

// a SBUS object, which is on hardware
// serial port 1
SBUS x4r(Serial1);

// channel, fail safe, and lost frames data
uint16_t channels[16];
int channelsCounter = 0;
int channelsDetected[16];
unsigned long channelsStartMillis = 0;

bool failSafe = false;
int failSafeCounter = 0;
bool failSafeDetected = false;
unsigned long failSafeStartMillis = 0;

bool lostFrame = false;
int lostFrameCounter = 0;
bool lostFrameDetected = false;
unsigned long lostFrameStartMillis = 0;


void setup()
{
	// begin the SBUS communication
	x4r.begin();

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
				lostFrameCounter++;
				Serial.print("failSafes Detected = "); Serial.println(lostFrameCounter);
				failSafeDetected = true;
		}
		if (failSafe == false && failSafeDetected == true) {
			Serial.print("FailSafe Recovered = "); Serial.print(millis() - failSafeStartMillis); Serial.println("ms");
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
			Serial.print("lostFrame Recovered = "); Serial.print(millis() - lostFrameStartMillis); Serial.println("ms");
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
				Serial.print("channels Recovered = "); Serial.print(millis() - channelsStartMillis); Serial.println("ms");
				channelsDetected[ch] = false;
			}
		}

#endif
	}

}
