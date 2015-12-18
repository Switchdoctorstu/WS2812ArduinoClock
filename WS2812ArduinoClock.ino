// hi/*
tm xmit - 4S watch 13 jmp to 0x000
// Stuarts code to drive Colourduino and TM1638 from GPS and DS1307 RTC
*	This code is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.
 *	This library is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *	Lesser General Public License for more details.
 
tm should always hold the current most valid time source
 
 // RF module needs external 3.3v supply plus decoupling capacitor

// Reads GPS on serial 3
// if GPS invalid:Reads the RTC 
// Displays time on TM1638
// Handles reset of RTC
// Handles button updates to RTC  < needs fixing
// Sends display to array of 4 Colorduino displays on I2C

Gets time from RTC or GPS receiver on Serial3
Sends I2C Character & Raster to Colorduino Receiver

All code attributed to original authors, i've put this together from 
so many sources that i can't track them all.


Currently working on:
OK: make packets fixed length = 16 bytes
OK: stx,type,index,data*11,chk,cr
OK: sending char packets
XX: BST offset
OK: RF handler
OK: Temperature from thermistor on AO
OK: adding bmp180 module
OK: startup time from rtc 
XX: Marquee - working on scrolling text
OK: Added debug modes to clear serial noise
OK: Added gps handler
OK: Added I2C display module handler
OK: watchdog - timer working
OK: self reset
OK: brightness

*/


//****************************************************
// define the modes that we can work as
#define RTCMODE 0  // running off RTC
#define GPSMODE 1	// running from GPS
#define SYSMODE 3 	// running off system clock
#define NUMMODES 3
#define DEBUGBACKGROUND false 
#define DEBUGWATCHDOG false
#define DEBUGI2C false
#define DEBUGMATRIX false
#define DEBUGGPS false
#define DEBUGREPORT true
#define DEBUGINBOX false
#define DEBUGTIME false
#define DEBUGBMP false
#define DEBUGMINITIME false
#define MATRIXENABLED true
#define RFENABLED true
#define DEBUGRF false
// GPS Definitions
#define GPSBUFFERLEN 128
#define ARGCOUNT 40
#define MAXI2CDEVICES 20
#define MAGNOADDRESS 0x1e
#define MAXERRORCOUNT 5
//#define MAXMOUNTMSGLEN 128
#define GPSPASSTHRU true  // enable pass-thru of GPS data so PC sees GPS

#define FIRSTMODULE 4  // i2c address of 1st module
#define LASTMODULE 7	// i2c address of last module
// some pins
#define INBOXPIN A0
#define RFCEPIN 49      // mega pins for RF module
#define RFCSNPIN 53

// Include the right libraries
#include <Wire.h>
#include <Time.h>
#include <DS1307RTC.h>  // RTC
#include <TM1638.h>  // LED Display 
#include <String.h>
#include <SFE_BMP180.h>

// RF includes
#include <SPI.h>
#include "printf.h"
#include "nRF24L01.h"  // Radio Libraries
#include "RF24.h" // Radio
// watchdog
#include <avr/wdt.h>


// ************************************
// Define Objects and variables
// ************************************
RF24 radio(RFCEPIN,RFCSNPIN); // define the RF transceiver
// flip the send and receive definitions for remote nodes
const uint64_t send_pipes[5] = { 0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL };
const uint64_t receive_pipes[5] = { 0x3A3A3A3AD2LL, 0x3A3A3A3AC3LL, 0x3A3A3A3AB4LL, 0x3A3A3A3AA5LL, 0x3A3A3A3A96LL };
char RFpacketout[8]; // RF packet buffers
char RFpacketin[8]; // RF packet buffers
unsigned long RFinterval=60055; // 30 second RF poll
unsigned long RFtimer; // Poll timer

struct sRGB{
	char r;
	char g;
	char b;
};

typedef struct sRGB RGB;

RGB pelcolour={0,0,0}; //picture element colour
// TM1638(byte dataPin, byte clockPin, byte strobePin, boolean activateDisplay = true, byte intensity = 7);
TM1638 dm (5, 6, 7);
tmElements_t tm;   	// storage for time components
time_t timeNow;    // storage for local clock

SFE_BMP180 pressure_sensor; // define the BMP180
double BMPaltitude,BMPpressure;//BMP180 variables
double BMPbaselinepressure; // baseline pressure
double BMPtemperature;  // Temperature of device
double BMPTemperatureHistory[48];
double BMPPressureHistory[48]; // 48 hour history
unsigned long logtimer,loginterval=5050; // 5 seconds
int logcounter=0; // count of log entries

int i2cdelay=5;//wait after each packet

// Declare all the global vars
int lastminute=0;
unsigned long waitcheckTime=0; // timer for time checking
unsigned long waitcheckButtons=0; // timer for buttons
unsigned long intervalcheckTime=1012;
unsigned long intervalcheckButtons=503;
unsigned long reportTime=1007;
unsigned long reportInterval=10007;
unsigned long gapSecond=0;
unsigned long lasttime=0;
unsigned long thistime=0;	// Time handles
unsigned long millisNow=0;
unsigned long NextTimeSyncTime=0;
unsigned long NextTimeSyncInterval=60023; // 30 seconds
unsigned long displayNextUpdateTime=0;
unsigned long displayUpdateInterval=913; // 0.9 seconds
boolean dots=0;
boolean moduleOff=0;
// setup mode 
int 	currentmode=RTCMODE;
int 	displaymode=1; // major display mode 0=char 1=raster
boolean rtcokFlag=false;
boolean gpstimeokFlag=false;

int screenwidth=32;
int screenheight=8;

const char *monthName[12] = {// array of month names
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

// global GPS vars
int 	i2cdevicecount=0;  // count of attached I2C devices
int 	i2caddresses[MAXI2CDEVICES];
int	 	index=0;
String 	gpsstring="                                                                               ";
char 	gpsmsg[GPSBUFFERLEN];
unsigned int gpsmsgindex=0;
int 	trigger=0;
int 	i2cmagno=0;
int 	fontheight=7;
int 	fontwidth=5;
char 	timearray[6]; // string for current valid time
int 	inboxsensor=0;
long inboxchecktime=0;
long inboxcheckinterval=5000;

// gps handling
String gpsfixvalid="N";
String gpsfixtime="000000";
String gpsfixdate="000000";
String gpslat="0.000000";
String gpslatort = "N";
String gpslong="0.000000"; // gps longitude
String gpslongort = "E";  // gps longitude compass
String Arg[ARGCOUNT];
String GPShhmmssdss="000000.00";
String GPSdd="01";
String GPSmm="01";
String GPSyy="01";
String GPSzh="00";
String GPSzm="00";

int argcount=0;  // number of arguments
int returncode=0; // generic return code

// buffer to hold raster 4 modules *8*8
unsigned char matrix[256][3];

int minifontwidth=3;
int minifontheight=5;
const unsigned char font3x5[][3]={
	 { 0x00, 0x00, 0x00 },               /*   - 0x20 - 32 */
        { 0x00, 0x29, 0x00 },               /* ! - 0x21 - 33 */ 
        { 0x00, 0x29, 0x00 },               /* " - 0x22 - 34 */ 
        { 0x31, 0x10, 0x31 },               /* # - 0x23 - 35 */
        { 0x24, 0x2a, 0x7f },               /* $ - 0x24 - 36 */
        { 0x23, 0x13, 0x08 },               /* % - 0x25 - 37 */
        { 0x08, 0x14, 0x08 },               /* o degrees- 0x26 - 38 */
        { 0x00, 0x05, 0x03 },               /* ' - 0x27 - 39 */
        { 0x00, 0x1c, 0x22 },               /* ( - 0x28 - 40 */
        { 0x00, 0x41, 0x22 },               /* ) - 0x29 - 41 */
        { 0x14, 0x08, 0x3e },               /* * - 0x2a - 42 */
        { 0x04, 0x31, 0x04 },               /* + - 0x2b - 43 */
        { 0x00, 0x01, 0x02 },               /* , - 0x2c - 44 */
        { 0x04, 0x04, 0x04 },               /* - - 0x2d - 45 */
        { 0x00, 0x01, 0x00 },               /* . - 0x2e - 46 */
        { 0x03, 0x04, 0x24 },               /* / - 0x2f - 47 */
	{31,17,31}, // 0
	{0,31,0}, // 1
	{19,21,9}, // 2
	{17,21,10}, //3
	{28,4,31}, // 4
	{29,21,22}, //5
	{14,21,22},//6
	{16,16,31},//7 
	{31,21,31}, //8
	{28,20,31} //9
};

const unsigned char font_5x7[][5] = { // Font Table
        { 0x00, 0x00, 0x00, 0x00, 0x00 },               /*   - 0x20 - 32 */
        { 0x00, 0x00, 0x5f, 0x00, 0x00 },               /* ! - 0x21 - 33 */ 
        { 0x00, 0x07, 0x00, 0x07, 0x00 },               /* " - 0x22 - 34 */ 
        { 0x14, 0x7f, 0x14, 0x7f, 0x14 },               /* # - 0x23 - 35 */
        { 0x24, 0x2a, 0x7f, 0x2a, 0x12 },               /* $ - 0x24 - 36 */
        { 0x23, 0x13, 0x08, 0x64, 0x62 },               /* % - 0x25 - 37 */
        { 0x36, 0x49, 0x55, 0x22, 0x50 },               /* & - 0x26 - 38 */
        { 0x00, 0x05, 0x03, 0x00, 0x00 },               /* ' - 0x27 - 39 */
        { 0x00, 0x1c, 0x22, 0x41, 0x00 },               /* ( - 0x28 - 40 */
        { 0x00, 0x41, 0x22, 0x1c, 0x00 },               /* ) - 0x29 - 41 */
        { 0x14, 0x08, 0x3e, 0x08, 0x14 },               /* * - 0x2a - 42 */
        { 0x08, 0x08, 0x3e, 0x08, 0x08 },               /* + - 0x2b - 43 */
        { 0x00, 0x50, 0x30, 0x00, 0x00 },               /* , - 0x2c - 44 */
        { 0x08, 0x08, 0x08, 0x08, 0x08 },               /* - - 0x2d - 45 */
        { 0x00, 0x60, 0x60, 0x00, 0x00 },               /* . - 0x2e - 46 */
        { 0x20, 0x10, 0x08, 0x04, 0x02 },               /* / - 0x2f - 47 */
        { 0x3e, 0x51, 0x49, 0x45, 0x3e },               /* 0 - 0x30 - 48 */
        { 0x00, 0x42, 0x7f, 0x40, 0x00 },               /* 1 - 0x31 - 49 */
        { 0x42, 0x61, 0x51, 0x49, 0x46 },               /* 2 - 0x32 - 50 */
        { 0x21, 0x41, 0x45, 0x4b, 0x31 },               /* 3 - 0x33 - 51 */
        { 0x18, 0x14, 0x12, 0x7f, 0x10 },               /* 4 - 0x34 - 52 */
        { 0x27, 0x45, 0x45, 0x45, 0x39 },               /* 5 - 0x35 - 53 */
        { 0x3c, 0x4a, 0x49, 0x49, 0x30 },               /* 6 - 0x36 - 54 */
        { 0x01, 0x71, 0x09, 0x05, 0x03 },               /* 7 - 0x37 - 55 */
        { 0x36, 0x49, 0x49, 0x49, 0x36 },               /* 8 - 0x38 - 56 */
        { 0x06, 0x49, 0x49, 0x29, 0x1e },               /* 9 - 0x39 - 57 */
        { 0x00, 0x36, 0x36, 0x00, 0x00 },               /* : - 0x3a - 58 */
        { 0x00, 0x56, 0x36, 0x00, 0x00 },               /* ; - 0x3b - 59 */
        { 0x08, 0x14, 0x22, 0x41, 0x00 },               /* < - 0x3c - 60 */
        { 0x14, 0x14, 0x14, 0x14, 0x14 },               /* = - 0x3d - 61 */
        { 0x00, 0x41, 0x22, 0x14, 0x08 },               /* > - 0x3e - 62 */
        { 0x02, 0x01, 0x51, 0x09, 0x06 },               /* ? - 0x3f - 63 */
        { 0x32, 0x49, 0x79, 0x41, 0x3e },               /* @ - 0x40 - 64 */
        { 0x7e, 0x11, 0x11, 0x11, 0x7e },               /* A - 0x41 - 65 */
        { 0x7f, 0x49, 0x49, 0x49, 0x36 },               /* B - 0x42 - 66 */
        { 0x3e, 0x41, 0x41, 0x41, 0x22 },               /* C - 0x43 - 67 */
        { 0x7f, 0x41, 0x41, 0x22, 0x1c },               /* D - 0x44 - 68 */
        { 0x7f, 0x49, 0x49, 0x49, 0x41 },               /* E - 0x45 - 69 */
        { 0x7f, 0x09, 0x09, 0x09, 0x01 },               /* F - 0x46 - 70 */
        { 0x3e, 0x41, 0x49, 0x49, 0x7a },               /* G - 0x47 - 71 */
        { 0x7f, 0x08, 0x08, 0x08, 0x7f },               /* H - 0x48 - 72 */
        { 0x00, 0x41, 0x7f, 0x41, 0x00 },               /* I - 0x49 - 73 */
        { 0x20, 0x40, 0x41, 0x3f, 0x01 },               /* J - 0x4a - 74 */
        { 0x7f, 0x08, 0x14, 0x22, 0x41 },               /* K - 0x4b - 75 */
        { 0x7f, 0x40, 0x40, 0x40, 0x40 },               /* L - 0x4c - 76 */
        { 0x7f, 0x02, 0x0c, 0x02, 0x7f },               /* M - 0x4d - 77 */
        { 0x7f, 0x04, 0x08, 0x10, 0x7f },               /* N - 0x4e - 78 */
        { 0x3e, 0x41, 0x41, 0x41, 0x3e },               /* O - 0x4f - 79 */
        { 0x7f, 0x09, 0x09, 0x09, 0x06 },               /* P - 0x50 - 80 */
        { 0x3e, 0x41, 0x51, 0x21, 0x5e },               /* Q - 0x51 - 81 */
        { 0x7f, 0x09, 0x19, 0x29, 0x46 },               /* R - 0x52 - 82 */
        { 0x46, 0x49, 0x49, 0x49, 0x31 },               /* S - 0x53 - 83 */
        { 0x01, 0x01, 0x7f, 0x01, 0x01 },               /* T - 0x54 - 84 */
        { 0x3f, 0x40, 0x40, 0x40, 0x3f },               /* U - 0x55 - 85 */
        { 0x1f, 0x20, 0x40, 0x20, 0x1f },               /* V - 0x56 - 86 */
        { 0x3f, 0x40, 0x38, 0x40, 0x3f },               /* W - 0x57 - 87 */
        { 0x63, 0x14, 0x08, 0x14, 0x63 },               /* X - 0x58 - 88 */
        { 0x07, 0x08, 0x70, 0x08, 0x07 },               /* Y - 0x59 - 89 */
        { 0x61, 0x51, 0x49, 0x45, 0x43 },               /* Z - 0x5a - 90 */
        { 0x00, 0x7f, 0x41, 0x41, 0x00 },               /* [ - 0x5b - 91 */
        { 0x02, 0x04, 0x08, 0x10, 0x20 },               /* \ - 0x5c - 92 */
        { 0x00, 0x41, 0x41, 0x7f, 0x00 },               /* ] - 0x5d - 93 */
        { 0x04, 0x02, 0x01, 0x02, 0x04 },               /* ^ - 0x5e - 94 */
        { 0x40, 0x40, 0x40, 0x40, 0x40 },               /* _ - 0x5f - 95 */
        { 0x00, 0x01, 0x02, 0x04, 0x00 },               /* ` - 0x60 - 96 */
        { 0x20, 0x54, 0x54, 0x54, 0x78 },               /* a - 0x61 - 97 */
        { 0x7f, 0x48, 0x44, 0x44, 0x38 },               /* b - 0x62 - 98 */
        { 0x38, 0x44, 0x44, 0x44, 0x20 },               /* c - 0x63 - 99 */
        { 0x38, 0x44, 0x44, 0x48, 0x7f },               /* d - 0x64 - 100 */
        { 0x38, 0x54, 0x54, 0x54, 0x18 },               /* e - 0x65 - 101 */
        { 0x08, 0x7e, 0x09, 0x01, 0x02 },               /* f - 0x66 - 102 */
        { 0x38, 0x44, 0x44, 0x54, 0x34 },               /* g - 0x67 - 103 */
        { 0x7f, 0x08, 0x04, 0x04, 0x78 },               /* h - 0x68 - 104 */
        { 0x00, 0x44, 0x7d, 0x40, 0x00 },               /* i - 0x69 - 105 */
        { 0x20, 0x40, 0x44, 0x3d, 0x00 },               /* j - 0x6a - 106 */
        { 0x7f, 0x10, 0x28, 0x44, 0x00 },               /* k - 0x6b - 107 */
        { 0x00, 0x41, 0x7f, 0x40, 0x00 },               /* l - 0x6c - 108 */
        { 0x7c, 0x04, 0x18, 0x04, 0x78 },               /* m - 0x6d - 109 */
        { 0x7c, 0x08, 0x04, 0x04, 0x78 },               /* n - 0x6e - 110 */
        { 0x38, 0x44, 0x44, 0x44, 0x38 },               /* o - 0x6f - 111 */
        { 0x7c, 0x14, 0x14, 0x14, 0x08 },               /* p - 0x70 - 112 */
        { 0x08, 0x14, 0x14, 0x18, 0x7c },               /* q - 0x71 - 113 */
        { 0x7c, 0x08, 0x04, 0x04, 0x08 },               /* r - 0x72 - 114 */
        { 0x48, 0x54, 0x54, 0x54, 0x20 },               /* s - 0x73 - 115 */
        { 0x04, 0x3f, 0x44, 0x40, 0x20 },               /* t - 0x74 - 116 */
        { 0x3c, 0x40, 0x40, 0x20, 0x7c },               /* u - 0x75 - 117 */
        { 0x1c, 0x20, 0x40, 0x20, 0x1c },               /* v - 0x76 - 118 */
        { 0x3c, 0x40, 0x30, 0x40, 0x3c },               /* w - 0x77 - 119 */
        { 0x44, 0x28, 0x10, 0x28, 0x44 },               /* x - 0x78 - 120 */
        { 0x0c, 0x50, 0x50, 0x50, 0x3c },               /* y - 0x79 - 121 */
        { 0x44, 0x64, 0x54, 0x4c, 0x44 },               /* z - 0x7a - 122 */
        { 0x00, 0x08, 0x36, 0x41, 0x00 },               /* { - 0x7b - 123 */
        { 0x00, 0x00, 0x7f, 0x00, 0x00 },               /* | - 0x7c - 124 */
        { 0x00, 0x41, 0x36, 0x08, 0x00 },               /* } - 0x7d - 125 */
        { 0x1C, 0x3E, 0x0F, 0x3E, 0x1C },               /* heart - 0x7e - 126 */
};

char matrixMessage[64];
int matrixMessagePointer=0; // pointer into banner messsage for marquee display 
boolean matrixMessageValid=false;
int matrixMessageCursor=0; 
unsigned int brightness=0xff; // brightness mask for display
unsigned int inboxlow=0xff; // lowest inbox reading
unsigned int inboxhigh=0x00; // 
unsigned int maxbrightness=0xff; // maximum brightness mask for display
unsigned int minbrightness=0x02; // min brightness mask for display
unsigned int contrast=0x40; // subtracted from brightness to give background

/****************************************************************************/
/*																			*/
/* Code Starts Here															*/
/*																			*/
/****************************************************************************/

void checkinboxsensor(){  // reads the inbox sensor pin
	if(millisNow>inboxchecktime){
		inboxchecktime+=inboxcheckinterval;
		inboxsensor=analogRead(INBOXPIN);
		brightness=map(inboxsensor,0,1023,maxbrightness,minbrightness);
		constrain(brightness,0,255);
		contrast=brightness*31/32;
		if(DEBUGINBOX){
			Serial.print("In Box Value:");
			Serial.println(inboxsensor,DEC);
			Serial.print("Brightness:");
			Serial.println(brightness,DEC);
			Serial.print("Contrast:");
			Serial.println(contrast,DEC);
			
			
		}
	}
}

 boolean isbst(int day, int month, int dow){ // checks if date is in bst
        if (month < 3 || month > 10)  return false; 
        if (month > 3 && month < 10)  return true; 
        int previousSunday = day - dow;
        if (month == 3) return previousSunday >= 25;
        if (month == 10) return previousSunday < 25;
        return false; // this line never gonna happen
    }

tmElements_t bstadjust(tmElements_t checkElements){
	if(isbst(checkElements.Day,checkElements.Month,weekday())){
		
	}
}	

void gettime(){ //			Loads all of the time variables from GPS>RTC>System clock		//

	millisNow=millis(); // timer for local loop triggers
	timeNow=now(); // get the current time stamp
  // get hrs and mins in usable format
	if(gpsfixvalid=="A"||gpsfixvalid=="V"){  // check for valid time from GPS
		gpsfixtime.toCharArray(timearray,6);
		currentmode=GPSMODE;
		if(gpsfixtime!=""){
			long int t=gpsfixtime.toInt();
			if(DEBUGTIME)Serial.println("GPS Fix Time" + gpsfixtime  + "  ToInt:" + String(t));
			tm.Hour=t/10000;
			t=t % 10000; // mod 10000
			tm.Minute=t/100;
			t=t % 100;
			tm.Second=t;
		}
		if(gpsfixdate!=""){
			long int t=gpsfixdate.toInt();
			tm.Day = t/10000;
			t=t % 10000;
			tm.Month = t/100;
			t=t % 100;
			t=t+2000-1970;
			tm.Year =	t;
		}
		if(isbst(tm.Day,tm.Month,weekday(timeNow))){
			tm.Hour=tm.Hour+1;
			if(DEBUGTIME)Serial.println("Date is in BST");
		
		}else{
			if(DEBUGTIME)Serial.println("Date is not in BST");
		
		};
	}	
	else{
		// get rtc time instead
		if (RTC.read(tm)) {
			rtcokFlag=true;
			currentmode=RTCMODE;
		}	
		else	{
			rtcokFlag=false;
			if (RTC.chipPresent()) {
				if(DEBUGTIME)Serial.println("The DS1307 RTC is stopped.  Attempting to initialise");
				setupRTC();
			} else {
				if(DEBUGTIME)Serial.println("DS1307 read error!  Please check the circuitry.");
			}
			// load tm from system clock - NEED REST OF PARMS
			tm.Hour=hour(timeNow);
			tm.Minute=minute(timeNow);
			tm.Second=second(timeNow);
			tm.Day=day(timeNow);
			tm.Month=month(timeNow);
			tm.Year=year(timeNow);
		}
	}
	
}

void print2digits(int number) {
  if (number > 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

ISR(WDT_vect) {// Watchdog timer interrupt.
	//  careful not to use functions they may cause the interrupt to hang and
	// prevent a reset.
	wdt_disable();
	if(DEBUGWATCHDOG)Serial.println("Watchdog!!");

	// flash pin 13 to signal error
	pinMode(13,OUTPUT);
	for(int n=0;n<10;n++){
		digitalWrite(13,LOW);
		delay(20);
		digitalWrite(13,HIGH);
		delay(20);
	}
	asm volatile ("  jmp 0"); // Bootstrap back to zero
}

void setup() {  // Main Setup Routine
	wdt_disable(); // disable the watchdog
	Serial.begin(19200);
	Serial3.begin(9600); // GPS on Mega
	// while (!Serial) ; // wait for serial
	watchdogSetup(); // start the watchdog
	// temp RF debug
	if(DEBUGRF){
	printf_begin();
	radio.begin();
	radio.openReadingPipe(1,receive_pipes[1]);
	radio.startListening();
	radio.printDetails();
	}
	
	
	delay(2000);// let rtc settle
	
		Serial.println("Stuart's LED RTC - GPS and DS1307RTC V0.1");
		Serial.println("-----------------------------------------");
	
	// Set the processor time
	setTime(0,0,0,16,10,1964);
	// get processor time
	timeNow=now();
	
	// Start I2C
	int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
	if (rtn != 0) {
		Serial.println(F("I2C bus error. Could not clear"));
		if (rtn == 1) {
		  Serial.println(F("SCL clock line held low"));
		} else if (rtn == 2) {
		  Serial.println(F("SCL clock line held low by slave clock stretch"));
		} else if (rtn == 3) {
			Serial.println(F("SDA data line held low"));
		}
	} 
	else { // bus clear
    // re-enable Wire
    // now can start Wire Arduino master
    Wire.begin();
	}

	scani2c();
	// setup the pressure_sensor
	if (pressure_sensor.begin())
		Serial.println("BMP180 init success");
	else{
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
		Serial.println("BMP180 init fail (disconnected?)\n\n");
  }

  // Get the baseline pressure:
  
  BMPbaselinepressure = getPressure();
  if(DEBUGBMP){
	Serial.print("baseline pressure: ");
	Serial.print(BMPbaselinepressure);
	Serial.println(" mb");
  }  
  
  // reset interval timers
	unsigned long t=millis();
	waitcheckTime = t + intervalcheckTime;  
	waitcheckButtons = t + intervalcheckButtons;
	displayNextUpdateTime=t+displayUpdateInterval;
	RFtimer=t+RFinterval;
	NextTimeSyncTime=t; // trigger initial time sync
	logtimer=t;  // trigger initial log
  if(RFENABLED)setupRF(); // start the radio 
   gettime();
   matrixMessageValid=false;
   //banner=String("Started                                                                                    ");
}

void watchdogSetup(void) {
	if(DEBUGWATCHDOG)Serial.println("Setting Up Watchdog");
// cli(); // disable all interrupts
wdt_reset(); // reset the WDT timer
/*
WDTCSR configuration:
WDIE = 1: Interrupt Enable
WDE = 1 :Reset Enable
WDP3 = 0; // :For 2000ms Time-out
WDP2 = 1; // :For 2000ms Time-out
WDP1 = 1; // :For 2000ms Time-out
WDP0 = 1; // :For 2000ms Time-out
*/

// Enter Watchdog Configuration mode:
WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog settings:
WDTCSR =  (1<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0)  | (1<<WDIE);
wdt_reset(); // reset the WDT timer
//wdt_enable(); // enable watchdog
// sei(); // enable interrupts
pinMode(13,OUTPUT);
digitalWrite(13,LOW); // signal by pin 13 low
if(DEBUGWATCHDOG)Serial.println("Watchdog set");

}

void setupRF(){
	radio.begin();
	radio.setRetries(10,3);
	radio.setPayloadSize(8);
	// we've got 5 pipes defined
	// this is the hub so we need 5 x send and 5 x receive
	
	if(DEBUGRF)		Serial.println("Starting Radio");
	radio.openWritingPipe(send_pipes[0]);
	radio.openReadingPipe(1,receive_pipes[1]);
    radio.openReadingPipe(2,receive_pipes[2]);
    radio.openReadingPipe(3,receive_pipes[3]);
    radio.openReadingPipe(4,receive_pipes[4]);
   // radio.openReadingPipe(5,receive_pipes[4]);
	radio.startListening();
	if(DEBUGRF){	printf_begin();
		radio.printDetails();
	}
	
}

void loop(){
	uint8_t pipe_number;
// main code here - runs repeatedly
		wdt_reset(); // reset the watchdog
		digitalWrite(13,LOW); // signal by pin 13 low
		gettime();   // read the time from the RTC or GPS 
		checkDisplayTimer();
		checkButtons();
		if(DEBUGREPORT)checkReport(); // see if we need to report
		if(Serial3.available()>0)GetGPSData();// check for GPS data
		if(Serial.available()>0)readconsole();// check for console data
		checktimesync(); // keep RTC in sync with GPS if available
		checkinboxsensor();
		pollRF(); // poll the rf receivers
		if(radio.available(&pipe_number)){
			if(DEBUGRF)Serial.println("RF Data on pipe:"+String(pipe_number));
			readRF();
		}
		
	}

void readconsole(){  // we have serial console data
		char inchar=Serial.read();
		if(GPSPASSTHRU)Serial3.write(inchar);
}
	
void readRF(){
	if(radio.read( &RFpacketin, sizeof(RFpacketin) )){
	if(DEBUGRF){
		Serial.print("RF Packet Received");
		for(int n=0;n<8;n++){
			Serial.print(":");
			Serial.print(RFpacketin[n],HEX);
		}
		Serial.println();
	}	
	}  else{
		if(DEBUGRF)Serial.println("RF read failed");
	}// get the packet
}

void pollRF(){
	if(millisNow>RFtimer){
		RFtimer=millisNow+RFinterval; // reset the timer`
		// Build poll packet
		RFpacketout[0]=0x23;
		RFpacketout[7]=0x0d;
		radio.stopListening();
		
		for(int n=1;n<5;n++){  // cycle thu remote units 1-4
			RFpacketout[2]=n;
			radio.openWritingPipe(send_pipes[n]);
			radio.openReadingPipe(n,receive_pipes[n]);
			// poll the remote
			if(radio.write(&RFpacketout,sizeof(RFpacketout))){
				if(DEBUGRF) Serial.println("RF Poll sent to node "+String(n));
			}
			else{
				if(DEBUGRF) Serial.println("RF Poll send failed to node"+String(n));
			}
			delay(10);
		}
		radio.openReadingPipe(1,receive_pipes[1]);
    radio.openReadingPipe(2,receive_pipes[2]);
    radio.openReadingPipe(3,receive_pipes[3]);
    radio.openReadingPipe(4,receive_pipes[4]);
radio.startListening();
	}
}	
	
double getPressure() {
  char status;
  double T,P,p0,a,nullpoint;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure_sensor.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure_sensor.getTemperature(BMPtemperature);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure_sensor.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.
		T=BMPtemperature;
        status = pressure_sensor.getPressure(P,BMPtemperature);
        if (status != 0)
        {
			// we have a good reading so log it if needed
			if(millisNow>logtimer){
				// make sure we have valid data
				nullpoint=0.0;
				if((BMPtemperature!=nullpoint)&&(BMPpressure!=nullpoint)){
					// shift the logs down
					for(int l=screenwidth-1;l>-1;l--){
						BMPPressureHistory[l+1]=BMPPressureHistory[l];
						BMPTemperatureHistory[l+1]=BMPTemperatureHistory[l];
					}
					BMPPressureHistory[0]=BMPpressure;
					BMPTemperatureHistory[0]=T;
					logtimer=millisNow+loginterval;
					logcounter++;
				}
			}
			return(P);
			
        }
        else if(DEBUGBMP)Serial.println("error retrieving pressure measurement\n");
      }
      else if(DEBUGBMP)Serial.println("error starting pressure measurement\n");
    }
    else if(DEBUGBMP)Serial.println("error retrieving temperature measurement\n");
  }
  else if(DEBUGBMP)Serial.println("error starting temperature measurement\n");
  return(0);
}

void fillMatrix(struct sRGB rgb){ 
	for(int p=0;p<256;p++){
		
		matrix[p][0]=rgb.r;
		matrix[p][1]=rgb.g;
		matrix[p][2]=rgb.b;
	}
}	

void writeMatrixSmall(char displayarray[], int xoffset){ 
	char fontbyte;
	
	int yoffset=2;
	int z=0;
	
	char red=brightness;
	char green = brightness;
	char blue=brightness;
	
	/* // clear the matrix
	pelcolour.r=0;
	pelcolour.g=0;
	pelcolour.b=0;
	fillMatrix(pelcolour);
	*/
	
	if(DEBUGMINITIME){
		Serial.println();
		Serial.print("minifontwidth:");
		Serial.println(minifontwidth);
		Serial.print("minifontheight:");
		Serial.println(minifontheight);
		//Serial.println("Minitime Local:"+localtime);
		for(z=0;z<6;z++){
			Serial.println(displayarray[z],HEX);
		}
	}
	// display it in small font
	int c,w,h;
	char ch;
	for(c=0;c<6;c++){ // cycle thru digits
	ch=displayarray[c];
	if((ch>0x20)&&(ch<0x40)){ // check char is valid
		for(w=0;w<minifontwidth;w++){
			fontbyte=font3x5[ch-32][w];		
			for(h=0;h<minifontheight;h++){
				z=(c*(minifontwidth+1))+((h+yoffset)*32)+w+xoffset;
					if(z<256){
					if(fontbyte&1<<h){ // bit is set
						if(DEBUGMINITIME)Serial.print("1");
						matrix[z][0]=red ;
						matrix[z][1]=green;
						matrix[z][2]=blue ;
					} 
					else { // bit is not set
						if(DEBUGMINITIME)Serial.print("0");
						//matrix[z][0]=backred;
						//matrix[z][1]=backgreen;
						//matrix[z][2]=backblue;		
					}
					}	
			}
			if(DEBUGMINITIME)Serial.println();
		}
	}
		if(DEBUGMINITIME)Serial.println();
	}
}

void writeMatrixLarge(char displayarray[],int charcount, int xoffset, char red,char green,char blue){ // message ,count, offset, r,g,b
	char fontbyte;
	
	int yoffset=0;
	int z=0;
	
	//char red=brightness;
	//char green = brightness;
	//char blue=brightness;
	
	if(DEBUGMINITIME){
		Serial.println();
		Serial.print("fontwidth:");
		Serial.println(fontwidth);
		Serial.print("fontheight:");
		Serial.println(fontheight);
		Serial.print("Length:");
		Serial.println(charcount,DEC);
		
		for(z=0;z<6;z++){
			Serial.println(displayarray[z],HEX);
		}
	}
	// display it in large font
	int c,w,h;
	char ch;
	int numchars=screenwidth/(fontwidth+1)+1;
	if(charcount>numchars)charcount=numchars;
	for(c=0;c<charcount;c++){ // cycle thru digits
	ch=displayarray[c];
	if((ch>0x20)&&(ch<0x7f)){ // check char is valid
		for(w=0;w<fontwidth;w++){
			fontbyte=font_5x7[ch-32][w];		
			for(h=0;h<fontheight;h++){
				z=(c*(fontwidth+1))+((h+yoffset)*32)+w+xoffset;
					if(z<256){
					if(fontbyte&1<<h){ // bit is set
						if(DEBUGMINITIME)Serial.print("1");
						matrix[z][0]=red ;
						matrix[z][1]=green;
						matrix[z][2]=blue ;
					} 
					else { // bit is not set
						if(DEBUGMINITIME)Serial.print("0");
						//matrix[z][0]=backred;
						//matrix[z][1]=backgreen;
						//matrix[z][2]=backblue;		
					}
					}	
			}
			if(DEBUGMINITIME)Serial.println();
		}
	}
		if(DEBUGMINITIME)Serial.println();
	}
}

void buildMiniTimeMatrix(){
	char fontbyte;
	int xoffset=8;
	int yoffset=2;
	int z=0;
	int intensity = 128;
	
	char red=brightness;
	char green = brightness;
	char blue=brightness;
	// clear the matrix
	pelcolour.r=0;
	pelcolour.g=0;
	pelcolour.b=0;
	fillMatrix(pelcolour);
	// get the time
	String localtime=timestring();
	localtime.toCharArray(timearray,7);
	if(DEBUGMINITIME){
		Serial.println();
		Serial.print("minifontwidth:");
		Serial.println(minifontwidth);
		Serial.print("minifontheight:");
		Serial.println(minifontheight);
		Serial.println("Minitime Local:"+localtime);
		for(z=0;z<6;z++){
			Serial.println(timearray[z],HEX);
		}
	}
	// display it in small font
	int c,w,h;
	char ch;
	for(c=0;c<6;c++){ // cycle thru digits
	ch=timearray[c];
	if((ch>0x20)&&(ch<0x40)){ // check char is valid
		for(w=0;w<minifontwidth;w++){
			fontbyte=font3x5[ch-32][w];
			
			for(h=0;h<minifontheight;h++){
				z=(c*(minifontwidth+1))+((h+yoffset)*32)+w+xoffset;
					if(fontbyte&1<<h){ // bit is set
						if(DEBUGMINITIME)Serial.print("1");
						matrix[z][0]= red ;
						matrix[z][1]=green;
						matrix[z][2]=blue ;
					} 
					else { // bit is not set
						if(DEBUGMINITIME)Serial.print("0");
						//matrix[z][0]=backred;
						//matrix[z][1]=backgreen ;
						//matrix[z][2]=backblue ;		
					}	
			}
			if(DEBUGMINITIME)Serial.println();
		}
	}
		if(DEBUGMINITIME)Serial.println();
	}
	
}

void buildBackground(int bgtype){ // 0=bmp 1= temp
// get max and min
int hp,x,y,z=0;
double maxv=-999999;
double minv=99999;
double v,range,hv;

	char backred=0x00;
	char backgreen=0x00;
	char backblue=brightness-contrast ;
	char red=0x00;
	char green = brightness-contrast ;
	char blue=0x00;

	for(hp=0;(hp<32)&&(hp<logcounter-1);hp++){ // get the max and min
			if(bgtype==0)v=BMPPressureHistory[hp];
			if(bgtype==1)v=BMPTemperatureHistory[hp];
		if(v!=0){
			if(v<minv)minv=v;
			if(v>maxv)maxv=v;
		}
	}

	range=maxv-minv; // get the range of values
	if(DEBUGBACKGROUND){
		Serial.print("Maxv:");
		Serial.println(maxv);
		Serial.print("Minv:");
		Serial.println(minv);
		Serial.print("Range:");
		Serial.println(range);
	}
	if(range==0)range=1; // make sure its not zero
	for(x=0;x<screenwidth;x++){
		if(bgtype==0)hv=BMPPressureHistory[screenwidth-x];
		if(bgtype==1)hv=BMPTemperatureHistory[screenwidth-x];
		v=(hv-minv)/range;
		v=abs(v)*screenheight;
		if(v>screenheight)v=screenheight;
		if(DEBUGBACKGROUND){
			Serial.print("log:");
			Serial.print(hv);
			Serial.print(" scaled:");
			Serial.println(v);
		}
		for(y=0;y<screenheight;y++){
			z=(y*screenwidth)+x;
			if(v>y){ //fore-colour
			matrix[z][0]=red ;
			matrix[z][1]=green ;
			matrix[z][2]=blue ;
			
			}else{ 	// Back-colour
			matrix[z][0]=backred;
			matrix[z][1]=backgreen ;
			matrix[z][2]=backblue;
			}
		}
		
	}
	
}

void buildMatrix(){
	// finally, if we build a matrix display here it should be sent to the display
	// if we set displaymode to 0x01
	// matrix =ColourRGB[256]
	pelcolour.r=0;
	pelcolour.g=0;
	pelcolour.b=0;
	fillMatrix(pelcolour);  // clear the dot matrix buffer
	
	//int character = (millis()/displayUpdateInterval % 64); // rotate character
// start by just copying time chars to buffer
int intensity = 128;
char backred=0x00;
char backgreen=brightness/8;
char backblue=brightness/4;
char red=brightness;
char green = brightness;
char blue=brightness;
char fontbyte;
char mybyte;
unsigned int z,m,l,w,y;
int x;
int offset=8; // shift clock x

	// set the colours up based on temperature
		//backgreen=BMPtemperature;
		//backgreen=backgreen & 0x0e;
		//backred=64-BMPtemperature; 
		//backred=backred &0x1f;
		//backblue=BMPtemperature;
		//backblue=backblue &0x0c;
	// time should be in timearray
	String localtime=timestring();
	localtime.toCharArray(timearray,6);
	if(gpsfixvalid=="V"){
		backblue=0x0F;
	}
	if(gpsfixvalid=="A"){
		backgreen=0x0F;
	}
// build the raster
	for(m=0;m<4;m++){  // Module Loop
		for(x=7;x>-1;x--){
			if(x<fontwidth){
				fontbyte=font_5x7[timearray[m]-32][x];
			}
			else {fontbyte=0;
			}
			
			if(DEBUGMATRIX){
				Serial.print("M:");
				Serial.print(m,DEC);
			}
			for(y=0;y<8;y++){ // cycle thru bits
			z=(m*6)+(y*32)+x+offset;
				if(fontbyte&128>>y){ // bit is set
					if(DEBUGMATRIX)Serial.print("1");
					matrix[z][0]=red;
					matrix[z][1]=green;
					matrix[z][2]=blue;
				} 
				else { // bit is not set
					if(DEBUGMATRIX)Serial.print("0");
					//matrix[z][0]=backred;
					//matrix[z][1]=backgreen;
					//matrix[z][2]=backblue;		
				}	
			}
			if(DEBUGMATRIX)Serial.println();
		}
	}
	}

/*
void bannertomatrix(){
	if(bannervalid){
		int bannerlength=banner.length();
		int bytecount=bannerlength*(fontwidth+1);
		int x,y,c,b,z,p=0;
		char fontbyte;
		int offset=0;
		int intensity = 128;
		char backred=0x00;
		char backgreen=0x04;
		char backblue=0x04;
		char red=0xFF;
		char green = 0xFF;
		char blue=0xFF;
		char bannerchar;
		pelcolour.r=backred;
		pelcolour.g=backgreen;
		pelcolour.b=backblue;
		fillMatrix(pelcolour);  // clear the dot matrix buffer
		for(x=0;x<screenwidth;x++){ // move along the x
			// calculate the bit pattern for the column
			bannerchar=banner.charAt(((bannerpointer+x)%bytecount)/(fontwidth+1));  // c contains character from banner
			b=(bannerpointer+x)%(fontwidth+1); // number of bits into the font
			p=bannerchar-32;
			if(DEBUGMATRIX){
				if((x%(fontwidth+1))==0) {
					Serial.print("Banner Char=");
					Serial.print(bannerchar,HEX);
					Serial.print(" Pointer:");
					Serial.print(p,HEX);
					Serial.print(" Bits into font=");
					Serial.println(b,HEX);
					
				}
			}
			if(b==fontwidth){
				fontbyte=0x00;
			}else{
				fontbyte=font_5x7[p][b];	
			}
			for(y=0;y<fontheight+1;y++){ // cycle thru bits
				z=(y*32)+x+offset;
				if(fontbyte&128>>y){ // bit is set
					if(DEBUGMATRIX)Serial.print("1");
					matrix[z][0]=red;
					matrix[z][1]=green;
					matrix[z][2]=blue;
				} 
				 else { // bit is not set
					if(DEBUGMATRIX)Serial.print("0");
				//	matrix[z][0]=backred;
				//	matrix[z][1]=backgreen;
				//	matrix[z][2]=backblue;		
				}	
			}
			if(DEBUGMATRIX)Serial.println();	
		}
		drawseconds();
			
		// check to see if we're near the end of the message
		bannerpointer=(bannerpointer+2)%bytecount;	// move the pointer on
		if(bannerpointer+screenwidth>bytecount)bannerpointer=0;
	}
}
*/
void drawseconds(int width,int xoffset){
	int s=tm.Second;
	s=s*width/60;
	// s=s+224;
	s=s+xoffset;
	if(s<256)matrix[s][0]=0xFF; // set pixel red
}

void drawgraphs(){
	int graphwidth=6;
	int graphheight=8;
	int graphstartx=0;
	int z,y,i,t,c,d,p;
	char backred = 0x00;
	char backblue =0x00;
	char backgreen =0x00;
	char red=brightness;
	char green =0x00;
	char blue =0x00;
	t=BMPtemperature;
	i=0;
	if(t<10){
		blue=brightness;
		red=0x00;
		green=0x00;
		}
		else{
			if(t<20){
				red=0x00;
				green=brightness;
				blue=0x00;
			}
			else{
				red=brightness;
				blue=0x00;
				green=0x00;
			}
		}
	
	d=graphheight*graphwidth;
	c=d*t/60;
	if(c>d)c=d;
	for(z=0;z<graphheight;z++){
		for(y=0;y<graphwidth;y++){
			p=(z*32)+graphstartx+y;
			if(i<c){ // foreground
			matrix[p][0]=red;
			matrix[p][1]=green;
			matrix[p][2]=blue;
			}
			else{  // background
				matrix[p][0]=backred;
				matrix[p][1]=backgreen;
				matrix[p][2]=backblue;
			
			}
			i++;
		}
	}
	// now the pressure
	z=(BMPpressure-960)*screenheight/80;
	if(z<0)z=0;
	if(z>screenheight)z=screenheight;
	for(y=0;y<8;y++){
		if(y<z){
            matrix[y*32+6][1]=brightness;
            matrix[y*32+7][1]=brightness;
}
		if(y>z){
  matrix[y*32+6][2]=brightness ;
  matrix[y*32+7][2]=brightness ;
}
	}
}

void sendToMatrix(){  	// draw something out to LED grid displays
	char c;
	
	
	if(displaymode==0){
		// get hrs and mins in usable format
	String localtime=timestring();
	localtime.toCharArray(timearray,6);
		// wants 
		//  '#' 
		// 		type = 0x01
		//    	character in ASCII
		// red,blue,green
		// ... padding to make whole packet 16 bytes
		// etx = 0x0d
		// cycle thru modules
		int red=128; int blue=128; int green=128;
		for(int a=FIRSTMODULE;a<LASTMODULE+1;a++){
			Wire.beginTransmission(a); // start talking to Module
			Wire.write(0x23);
			Wire.write(0x01);
			c=timearray[a-FIRSTMODULE];
			Wire.write(c); // send the character
			Wire.write(red);
			Wire.write(green);
			Wire.write(blue);
			// pad out to 16 bytes
			
			for(int n=0;n<9;n++){
				Wire.write(0x00);
			}
			Wire.write(0x0d);
			returncode=Wire.endTransmission();
			delay(i2cdelay);
			if(DEBUGI2C){
				Serial.println("sent "+ String(c) + " To Address:" +String(a)+" Return:"+String(returncode));
			}
		}
	}
	
	if(displaymode==1){
		// we're in raster mode so send some chars
		// protocol goes:
		//  0 	stx
		//  1 	0x02   = type 2 = raster
		//  2 	0xLC = Line(0-7) Colour(0-2) 0=red 1=blue 2=green
		// 	3-10 	byte x 8    intensity data for that line
		// 	11	padding to 16 bytes
		// 	15 	etx = 0x0d
			
		for(int colour=0;colour<3;colour++){
			for(int line=0;line<8;line++){
				for(int m=FIRSTMODULE;m<LASTMODULE+1;m++){	
					// red first
					Wire.beginTransmission(m);
					Wire.write(0x23);  // stx= '#'
					Wire.write(0x02); // type = 0x02 = raster
					char t=line*16+colour;
					Wire.write(t);
					if(DEBUGI2C) Serial.print("A:"+String(m)+" ");
					int z=((m-FIRSTMODULE)*8)+(line*32);
					for(int p=z;p<(z+8);p++){
						Wire.write(matrix[p][colour]);
						if(DEBUGI2C) {
							Serial.print(matrix[p][colour],HEX);
							Serial.print(",");
						}
					}
					if(DEBUGI2C) Serial.println();
					// Pad out to 16 bytes
					Wire.write(0x00);
					Wire.write(0x00);
					Wire.write(0x00);
					Wire.write(0x00); // should be checksum.. later
					Wire.write(0x0d); // send the ETX
					returncode=Wire.endTransmission(true); // send stop message
					if(DEBUGI2C){Serial.println(returncode,HEX);}
					if(returncode!=0){ // if not successful clear the i2c bus
						if(returncode!=2) returncode=I2C_ClearBus(); // 2 = not there
					}
					delay(i2cdelay);
				}
			}
		}
		// send flip to modules
			for(int m=FIRSTMODULE;m<LASTMODULE+1;m++){	
				sendflip(m);
			}
	}
}
	
void sendflip(int m){
	Wire.beginTransmission(m);
					Wire.write(0x23);  // stx= '#'
					Wire.write(0x03); // type = 0x03 = flip buffer
					for(int n=0;n<13;n++) Wire.write(0x00); // fill buffer
					// Wire.beginTransmission(m);
					Wire.write(0x0d);  // etx= CR
					returncode=Wire.endTransmission(true);
					delay(i2cdelay);
}
	
void sendclear(int m, int r,int g, int b){ // send clear screen to matrix
// clear screen to 
			// protocol goes:
			//  0 	stx
			//  1 	0x04   = type 4 = Fill
			//  2 	red
			// 	3	green
			//	4  	blue
			// etx in 15
					Wire.beginTransmission(m);
					Wire.write(0x23);  // stx= '#'
					Wire.write(0x04); // type = 0x04 = fill screen
					Wire.write(r);
					Wire.write(g);
					Wire.write(b);
					for(int n=0;n<10;n++) Wire.write(0x00); // fill buffer
					Wire.write(0x0d);  // etx= CR
					returncode=Wire.endTransmission(true);
					delay(i2cdelay);
}	

void checktimesync(){
	if(millisNow>NextTimeSyncTime){
		NextTimeSyncTime=millisNow+NextTimeSyncInterval;
		if(DEBUGTIME)Serial.println("syncing time <*****************************************");
		// Time precedence
		// GPS if Valid
		// RTC if valid
		// system clock
		if((gpsfixvalid=="A")){ //reload tm and write it to the RTC
			long int t=gpsfixtime.toInt();
			if(DEBUGTIME)Serial.println("GPS Fix Time" + gpsfixtime  + "  ToInt:" + String(t));
			tm.Hour=t/10000;
			t=t % 10000; // mod 10000
			tm.Minute=t/100;
			t=t % 100;
			tm.Second=t;
			t=gpsfixdate.toInt();
			tm.Day = t/10000;
			t=t % 10000;
			tm.Month = t/100;
			t=t % 100;
			t=t+2000-1970;
			tm.Year =	t;
			// now write it to RTC
			if(DEBUGTIME){
				Serial.println("Writing:"+ String(tm.Hour)+":"+String(tm.Minute) +":"+String(tm.Second)+":"+String(tm.Day)+":"+String(tm.Month)+":"+String(tm.Year));
			}
			if (RTC.write(tm))			{
				Serial.println("RTC reset from GPS");
			}
			else			{
				Serial.println("Error Writing to RTC");		
			}
			//
		}
		// set system time to RTC
		// need to validate data before setting
		if(rtcokFlag){
			if(tm.Hour>-1&&tm.Hour<24&&tm.Minute>-1&&tm.Minute<60){
				setTime(tm.Hour,tm.Minute,tm.Second,tm.Day,tm.Month,tm.Year-30);
				if(DEBUGTIME)Serial.println("system clock set to tm");
			}
			else{
				if(DEBUGTIME)Serial.println("time in tm not valid for sync");
			}
		}
	}
}
	
void checkDisplayTimer(){
	
  if (millisNow > waitcheckTime) {
    dots = !dots;
    drawToModule();
    waitcheckTime = millisNow+ intervalcheckTime;
  }
  if(tm.Minute!=lastminute){
	  lastminute=tm.Minute;
	//  buildMatrix(); // build the raster to display on the matrix
	//  if(MATRIXENABLED)sendToMatrix();
	//  displayNextUpdateTime=millisNow+displayUpdateInterval; // reset LED display timer
  }
  if(millisNow>displayNextUpdateTime){
	  displayNextUpdateTime=millisNow+displayUpdateInterval; // reset LED display timer
	  //if(MARQUEEENABLED)buildmarquee();
	  
	  // change the display based on the seconds count
	  int timediv=(tm.Second +3)  /6 % 5;
	 
	 buildmatrixdisplay(timediv);
	
	  if(MATRIXENABLED)sendToMatrix();
  }
}

void buildmatrixdisplay(int type){
	// 0 = Big Time
	// 1 = Mini Time
	// 3 = pressure
	// 4 = temperature
	
	
	 switch(type){
		  case 0: {
			  buildMiniTimeMatrix();
	// Draw temperature graph
	drawgraphs(); 
	drawseconds(32,0);
		  
			  break;
		  }
		  case 1:{
			  buildMatrix();
// Draw temperature graph
	drawgraphs(); 
	drawseconds(32,0);

			  break;
		  }
		  
		  case 2: {
			 dtostrf(BMPtemperature,3,2,matrixMessage);
				  matrixMessage[5]=0x26;
					buildBackground(1);
					writeMatrixSmall(matrixMessage,2);
					drawseconds(32,0);
					matrixMessage[0]=0x7e;
					int se=tm.Second&0x01;
					se=se+25;
					writeMatrixLarge(matrixMessage,1,se,0xff,0x00,0x00);// message ,count, offset, r,g,b  
			  break;
		  }
		  
		  case 3: {
			  // 	write pressure in mini font
					//sprintf(displaythis,"%f",BMPpressure);
					dtostrf(BMPpressure,4,1,matrixMessage);
					buildBackground(0);
					writeMatrixSmall(matrixMessage,2);
					drawseconds(32,0);
			  break;
		  }
		  
		  case 4: {
			 dtostrf(BMPtemperature,3,2,matrixMessage);
				  matrixMessage[5]=0x26;
					buildBackground(1);
					writeMatrixSmall(matrixMessage,2);
					drawseconds(32,0);
					matrixMessage[0]=0x7e;
					int se=tm.Second&0x01;
					se=se+25;
					writeMatrixLarge(matrixMessage,1,se,0xff,0x00,0x00);// message ,count, offset, r,g,b  
			  
			  break;
		  }
		  
		  default :{
			  buildMiniTimeMatrix();
			  break;
		  }
	  }
}

void buildmarquee(){
	//String m;
	// String msg = String("initialising marquee                                                           ");
	//msg = timestring();
	
	BMPpressure=getPressure(); // get the current pressure
	BMPaltitude=pressure_sensor.altitude(BMPpressure,BMPbaselinepressure);
			
	//banner=String("S+P :");
	//banner+="Alt:"+String(BMPaltitude)+"m Tmp:"+String(BMPtemperature);
	//banner+= hour(timeNow);
    //banner+=":";
	//banner= String(banner+String(minute(timeNow)));
    //  banner=String(banner+String(":"));
	//  banner= String(banner+String(second(timeNow)));  
	//		if(isAM(timeNow)) banner=String(banner+"AM ");
	//		if(isPM(timeNow)) banner=String(banner+"PM ");

	//banner+=datestring();
	//banner+=" ";
	//banner+=String(BMPpressure);
	//banner+="mb";
	// banner = String(banner+" End of Message");
	//if(banner.length()<64)bannervalid=true;
	
}

void checkReport() {
		if( millisNow> reportTime){ // if we're due
			reportTime=millisNow+reportInterval; // reset interval
			Serial.println("*******************************");
			
			Serial.print("Onboard Clock:"+String(hour(timeNow))+":"+ String(minute(timeNow))) +":"+ String(second(timeNow));  // Print system time
			if(isAM(timeNow)) Serial.println("AM");
			if(isPM(timeNow)) Serial.println("PM");
			// now() time in seconds since 1/1/70
			Serial.println(String(day(timeNow))+":"+String(month(timeNow))+":"+String(year(timeNow)));             // The day now (1-31)
			Serial.println("DOTW:"+String(weekday(timeNow)));         // Day of the week, Sunday is day 1
 			//Serial.println(String(dayStr(weekday(time_now)))+" "+String(day(time_now))+String(monthStr(month(time_now))));
			//Serial.println(String( monthShortStr(month(time_now)))+" "+String(dayShortStr(weekday(time_now))));
			Serial.println("Time Status:"+String(timeStatus()));
			Serial.println("Time Now:"+String(timeNow));
			if(rtcokFlag==true){
				Serial.println("RTC Ok");
				
			}
			else
			{
				Serial.println("RTC not valid.");
			}
			Serial.print("tm. Time = ");
			print2digits(tm.Hour);
				Serial.write(':');
				print2digits(tm.Minute);
				Serial.write(':');
				print2digits(tm.Second);
				Serial.print(", Date(D/M/Y) ");
				Serial.print(tm.Day);
				Serial.write('/');
				Serial.print(tm.Month);
				Serial.write('/');
				Serial.print(tmYearToCalendar(tm.Year));
				Serial.println();
			Serial.println("GPS fix status:" + gpsfixvalid);
			if((gpsfixvalid=="A")||(gpsfixvalid="V")){
				Serial.println("GPS fix time:" + gpsfixtime + " Date:"+gpsfixdate);
				Serial.print("GPS latitude:" + gpslat + " " + gpslatort);
				Serial.println(" longitude:" + gpslong + " " + gpslongort);  // gps longitude compass
				
			}
			else{
				Serial.println("GPS fix Invalid");
			}			
			BMPpressure=getPressure(); // get the current pressure
			BMPaltitude=pressure_sensor.altitude(BMPpressure,BMPbaselinepressure);
			Serial.println("Altitude:"+String(BMPaltitude)+" Temperature:"+String(BMPtemperature));
			Serial.println("Sync:"+String(NextTimeSyncTime-millisNow));
		
			
		}
		
		
	}

void checkButtons(){
  if (millisNow > waitcheckButtons) {
    // dm.setLEDs(0);
    byte buttons=dm.getButtons();
    if(buttons>0){
      for (byte i=0;i<8;i++){
        if ((buttons & 1<<i) != 0) {
          Serial.print("Button Event:");
		  Serial.println(i);
		  buttonEvent(i);
          waitcheckButtons +=(intervalcheckButtons*5);
          // drawToModule();
        }
      }
    }
    waitcheckButtons =millisNow+intervalcheckButtons;
  }
}

void drawToModule(){
  if (!moduleOff){
    // unsigned long elapSecond = round(millis() / 1000);
    // unsigned long totalSecond = gapSecond + elapSecond;
    byte pos = 2; // totalSecond % 4;
    // if (pos>2) pos=1;
	// get hrs and mins in usable format
	dm.clearDisplay();
	
	dm.setDisplayToString(timestring(),(dots * 80),pos);
	
   }
}

String timestring()   {
return twochars(tm.Hour)+twochars(tm.Minute)+twochars(tm.Second);
}

String datestring(){
	String thisstring;
		thisstring=dayShortStr(weekday(timeNow));
			thisstring+=" ";
			thisstring+=String(day(timeNow));
			thisstring+=" ";
			thisstring+=String(monthShortStr(month(timeNow)));
			thisstring+=" ";
			thisstring+=String(year(timeNow));
	return thisstring;
}
String twochars(int number){
	if(number==0) {
		return"00";
	}
	else {
		if (number > 0 && number < 10)	{
			return "0"+String(number);
		}
		else	{
			return String(number);
		}
	}
}

void buttonEvent(byte inp){
  dm.setLED( 1,inp);
  switch (inp) {
  case 0: { // inc hours
    if (tm.Hour != 23)
    {  
		tm.Hour++ ;
	}
    else
	{
      tm.Hour=0;
    }
	// now write it to RTC
	if (RTC.write(tm))
	{
		Serial.println("hour set");
	}
	else
	{
		Serial.println("Error Writing to RTC");		
    }
    break;
  }
  case 1:  { // decrement hours
  if (tm.Hour != 0)
    {  
		tm.Hour-- ;
	}
    else
	{
      tm.Hour=23;
    }
	// now write it to RTC
	if (RTC.write(tm)) {
			Serial.println("hour set");
		}
	else
	{
	Serial.println("Error Writing to RTC");		
    }
    break;
  }
  case 2: { // increment minutes
    if (tm.Minute != 59)
    {  
		tm.Minute++ ;
	}
    else
	{
      tm.Minute=0;
    }
	// now write it to RTC
	if (RTC.write(tm)) {
			Serial.println("minute set");
	}
	else
	{
		Serial.println("Error Writing to RTC");		
    }
	break;
  }
  case 3:  { // decrement Minutes
    if (tm.Minute != 0)
    {  
		tm.Minute-- ;
	}
    else
	{
      tm.Minute=59;
    }
	// now write it to RTC
	if (RTC.write(tm)) {
			Serial.println("minute set");
	}
	else
	{
		Serial.println("Error Writing to RTC");		
    }
	break;
  }
  case 4: { // reset seconds
    tm.Second=0;
	if (RTC.write(tm)) {
			Serial.println("seconds reset");
	}
	else
	{
		Serial.println("Error Writing to RTC");		
    }
	
	break;
  }
  case 5:{  // led on/off
    moduleOff = !moduleOff;
    if (moduleOff) dm.clearDisplay();
    break;
  }
  case 6:
  Serial.println("Button 6 Pressed !");
    break;
  case 7:{
	// cycle thru the modes
	Serial.println("Button 7 Pressed !");
	Serial.print("Mode:");
	Serial.println(currentmode);
	currentmode++;
	// send clearscreen
	for(int m=4;m<8;m++){
		sendclear(m,0,0,0x20);
	}
	if(displaymode==0)
	{
		displaymode=1;
	}
	else{
		displaymode=0;
	}
    break;
  }
  }
}

void setupRTC() {// reset the RTC
	bool parse=false;
	bool config=false;
	timeNow=now(); // get current system time
	tm.Hour=hour(timeNow);
	tm.Minute=minute(timeNow);
	tm.Second=second(timeNow);
	tm.Year=year(timeNow)-30;
  tm.Month=month(timeNow);
	tm.Day=day(timeNow);
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  
  delay(200);
  if(DEBUGTIME){
	  if (config) {
		Serial.print("setupRTC: DS1307 configured Time=");
		Serial.println(String(tm.Hour)+":"+String(tm.Minute)+":"+String(tm.Second));
		Serial.print(", Date=");
		Serial.println(String(tm.Day)+"/"+String(tm.Month)+"/"+String(tm.Year));
	  } else {
		Serial.println("DS1307 Communication Error :-{");
		Serial.println("Please check your circuitry");
	  } 
	}
}

void scani2c(){
	int error;
  int address;
  Serial.println("Scanning I2C...");
  i2cdevicecount = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
		i2caddresses[i2cdevicecount]=address;  // store the found device address

		i2cdevicecount++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (i2cdevicecount == 0)
    Serial.println("No valid I2C devices found\n");
  else
    Serial.println("I2C Scan complete \n");

}

void GetGPSData(){
		int lf = 10;
		char inchar;
		
		inchar = Serial3.read();
		// Serial.println("GPS:"+String(inchar));
		// strip out nasties
		if(inchar>32){
			if(inchar<128){
				gpsmsg[gpsmsgindex]=inchar;
				gpsmsgindex++;
			}
		}
		if(inchar=='$')  // first char of gps message is a $
		{
			gpsmsg[0]=inchar;
			gpsmsgindex=1;
			trigger=1;
		}

      // gpsmsgindex=gpsstring.length();
		if (gpsmsgindex>(GPSBUFFERLEN-1)){
			// gps message buffer over-run (no CR seen)
			Serial.print("GPS Message Buffer Overrun");
			if(DEBUGGPS)PrintGPSMessage();
			gpsmsgindex=0;
			exit;
		}
	
		if(inchar==lf){
			//check for valid message
			if(trigger==1){
				// end of message
				// copy to String
				gpsmsg[gpsmsgindex]='\0';  // null terminate
				gpsstring=gpsmsg;
				// Process GPS Message 
				if(DEBUGGPS=='Y'){
					Serial.print("GPS:");
					Serial.println(gpsstring);
				}
				// pass GPS data if pass-thru enabled
				if(GPSPASSTHRU)Serial.println(gpsstring);
				GPSParser();
				GPSProcess();
			}
			gpsmsgindex=0; // reset the buffer
			trigger=0;
			exit;
		}
	
	}

void GPSParser(){
	// parse GPS message into string array
	int i;
	int pc; // parm count
	int gpsmsglength;
	int cp1; // comma pointer1
	int cp2; // comma pointer2
	int lastcomma=0;
    char c1;
	String s1=gpsstring;
	int mlen=s1.length();
	if(DEBUGGPS){
		Serial.print("Parsing:");
        Serial.print(gpsmsgindex,DEC);
		Serial.print(" Characters: ");
    	 
		Serial.print(mlen);
		Serial.println(s1);
	}
	cp2=0; 
	for(pc=0;pc<10;pc++){
		Arg[pc]="";
	}
	pc=0; // reset parm counter
	
	int lastcommapos=s1.lastIndexOf(',');
	// do we have commas?
	if(lastcommapos>0){
		while(lastcomma==0)
		{
			cp1=s1.indexOf(',',cp2);
			if(cp1>0){
				cp2=s1.indexOf(',',cp1+1);
				if(cp2>0){
					// Serial.println(cp1,DEC);
					Arg[pc]=s1.substring(cp1+1,cp2);
				pc++;
				} else {
				lastcomma=1;
				}
			}
			else
			{
				lastcomma=1;
			}
			if(pc>(ARGCOUNT-1)){
				lastcomma=1;
			}
		}
		if(DEBUGGPS){
			Serial.print("Found ");
			Serial.print(pc,DEC);
			Serial.println(" parameters");
			for(i=0;i<pc;i++)
			{
				Serial.println(String(String(i)+':'+Arg[i]));
			}
		}
	}
	argcount = pc;
}

void GPSProcess(){
	// GSP messages
	String gpscmd = gpsstring.substring(1,6);
	if(DEBUGGPS)	Serial.println("Processing "+gpscmd);

	if(gpscmd=="GPGLL"){ 
		if(DEBUGGPS){
		Serial.println("LL - Latitude:"+Arg[0]+" "+ Arg[1]+ "Longitude:"+Arg[2]+" "+Arg[3]);
		}
		/* 
		$GPGLL,4916.45,N,12311.12,W,225444,A,*1D

		Where:
			 GLL          Geographic position, Latitude and Longitude
		0,1	 4916.46,N    Latitude 49 deg. 16.45 min. North
		2,3	 12311.12,W   Longitude 123 deg. 11.12 min. West
		4	 225444       Fix taken at 22:54:44 UTC
		5	 A            Data Active or V (void)
			 *iD          checksum data
		*/
		// PrintGPSMessage();		
		gpslat=Arg[0];
		gpslatort=Arg[1];
		gpslong=Arg[2];
		gpslongort=Arg[3];
		gpsfixtime=Arg[4];
		exit;
	}
	if(gpscmd=="GPRMC"){
		/*
		$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

Where:
     RMC          Recommended Minimum sentence C
    0- 123519       Fix taken at 12:35:19 UTC
    1-  A            Status A=active or V=Void.
    2-  4807.038   Latitude 48 deg 07.038'
	3-  N				orthagonal
    4-  01131.000  Longitude 11 deg 31.000' 
	5-  E				orthagonal
    6 - 022.4        Speed over the ground in knots
    7 - 084.4        Track angle in degrees True
    8 - 230394       Date - 23rd of March 1994
    9 - 003.1,W      Magnetic Variation
    10 - *6A          The checksum data, always begins with *
		*/
		gpsfixvalid=Arg[1];
		if(gpsfixvalid=="A"){
			gpsfixtime=Arg[0];
			gpslat=Arg[2];
			gpslatort = Arg[3];
			gpsfixdate=Arg[8];
			gpslong=Arg[4]; // gps longitude
			gpslongort = Arg[5];  // gps longitude compass
			
		} 
		if(gpsfixvalid=="V"){
			gpsfixtime=Arg[0];
			gpslat=Arg[2];
			gpslatort = Arg[3];
			if(Arg[8]!="")gpsfixdate=Arg[8];
			gpslong=Arg[4]; // gps longitude
			gpslongort = Arg[5];  // gps longitude compass
			
		} 
		exit;
	}
	if(gpscmd=="GPGGA"){
		// Serial.print("GA Message:");	
		/*
		$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
		string gpsfixtime="000000";
		float gpslat=0.00;
		string gpslatort = "N";
		float gpslong=0.00;
		string gpslongort = "E";
			Where:
				 GGA          Global Positioning System Fix Data
				 123519       Fix taken at 12:35:19 UTC
				 4807.038,N   Latitude 48 deg 07.038' N
				 01131.000,E  Longitude 11 deg 31.000' E
				 1            Fix quality: 0 = invalid
										   1 = GPS fix (SPS)
										   2 = DGPS fix
										   3 = PPS fix
							   4 = Real Time Kinematic
							   5 = Float RTK
										   6 = estimated (dead reckoning) (2.3 feature)
							   7 = Manual input mode
							   8 = Simulation mode
				 08           Number of satellites being tracked
				 0.9          Horizontal dilution of position
				 545.4,M      Altitude, Meters, above mean sea level
				 46.9,M       Height of geoid (mean sea level) above WGS84
								  ellipsoid
				 (empty field) time in seconds since last DGPS update
				 (empty field) DGPS station ID number
				 *47          the checksum data, always begins with *
				*/
		if(gpsstring[7]!=0x2c){  // no comma in position 7 so have fix details
			if(DEBUGGPS){
				// copy the fix time
				Serial.print("GPS Fix Time:"+Arg[0]);
				Serial.print(" Fix Status:" + gpsfixvalid);
				Serial.print(" -  Latitude:"+Arg[1]+" "+Arg[2]);
				Serial.println(" -  Longitude:"+Arg[3]+" "+Arg[4]);
			}
			gpsfixtime=gpsstring.substring(7,13);
			// Serial.println(gpsfixtime);
		} exit;
	}
	
	if(gpscmd=="GPZDA"){
		if(DEBUGGPS)Serial.print("ZDA Message:");	
		/*
		ZDA - Data and Time

  $GPZDA,hhmmss.ss,dd,mm,yyyy,xx,yy*CC
  $GPZDA,201530.00,04,07,2002,00,00*60

where:
	hhmmss    HrMinSec(UTC)
        dd,mm,yyy Day,Month,Year
        xx        local zone hours -13..13
        yy        local zone minutes 0..59
        *CC       checksum
				*/
		if(gpsstring[7]!=0x2c){  // no comma in position 7 so have time details
			gpstimeokFlag=true;
			GPShhmmssdss=Arg[0];
			GPSdd=Arg[1];
			GPSmm=Arg[2];
			GPSyy=Arg[3];
			GPSzh=Arg[4];
			GPSzm=Arg[5];
		
				
		} 
		else {
		gpstimeokFlag=false;	
		}
		exit;
	}
}	

void PrintGPSMessage(){
	// dump the current GPS buffer to the serial port 
	int i;
	for(i=0;i<gpsmsgindex;i++){
		Serial.print(gpsmsg[i]);
	}
	Serial.println();
}

int I2C_ClearBus() {
	/**
 * This routine turns off the I2C bus and clears it
 * on return SCA and SCL pins are tri-state inputs.
 * You need to call Wire.begin() after this to re-enable I2C
 * This routine does NOT use the Wire library at all.
 *
 * returns 0 if bus cleared
 *         1 if SCL held low.
 *         2 if SDA held low by slave clock stretch for > 2sec
 *         3 if SDA held low after 20 clocks.
 */
	if(DEBUGI2C)Serial.println("Clearing I2C");
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}
