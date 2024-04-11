/*
 * Project L14_04_LoRaGPS
 * Description: Starter Code for utilizing LoRa
 * Author: Brian Rashap and Christian Chavez & Elsmen Aragon
 * Date: 24-MAR-2023
 */

// NOTE: The Adafruit_GPS header file has been modified to work with the Particle environment DO NOT INSTALL Adafruit_GPS USE the library from 14_04

#include "Particle.h"
#include "Adafruit_GPS.h"
#include "IoTTimer.h"
#include "credentials.h"
#include "Adafruit_BME280.h"

#define SENSOR_ADDRESS 0x40 // Change this to the address obtained from the I2C scan

Adafruit_BME280 myReading; //Defining bme object mine is called myReading
Adafruit_GPS GPS(&Wire);
IoTTimer sampleTimer;// timer for led onboard
// Define User and Credentials
String password = "AA4104132968BA2224299079021594AB"; // AES128 password
String myName = "borrowed";

// Define Constants
const int RADIOADDRESS = 0xA2; // it will be a value between 0xA1 and 0xA9 for my case
const int TIMEZONE = -6;
const unsigned int UPDATE = 10000;
const int RADIONETWORK = 1;    // range of 0-16
const int SENDADDRESS = 0xA1;   // address of radio to be sent to

//Delcare variable for air quality sensor
byte data[29];
int aqOne;
int aqOneResult;
const int AQTIMER = 10000;

// Declare Variables for gps
float lat, lon, alt;
int sat;
unsigned int lastGPS;

//Declare varibles for BME280
const int HEXADDRESS = 0X76;
bool status;
float tempResult;

// Declare Functions
float getBme();
int getAirQuality();
void getGPS(float *latitude, float *longitude, float *altitude, int *satellites);
void sendData(float temp, int aq, float latt);
void reyaxSetup(String password);


SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);
  Serial1.begin(115200);
  reyaxSetup(password);
  //Initialize GPS
  GPS.begin(0x10);  // The I2C address to use is 0x10 from I2C scan
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);

  // Initialize I2C communication for air Quality HM3301
  Wire.beginTransmission(SENSOR_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.println("Sensor found at address 0x40");
  } 
  else {
    Serial.println("No sensor found at address 0x40.");
  }
  //BME Check
  status = myReading.begin(HEXADDRESS);
  if ( status == false ) {
    Serial.printf(" BME280 at address 0x%02x X failed to start ", HEXADDRESS );
  }
  //Timer for sample air quality + other data
  sampleTimer.startTimer(AQTIMER);
}

void loop() {
  //Get data from GPS unit (best if you do this continuously) 
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
       return;
    }   
  }
  if (millis() - lastGPS > UPDATE) {
    lastGPS = millis(); // reset the timer
    getGPS(&lat,&lon,&alt,&sat);
    Serial.printf("\n=================================================================\n");
    Serial.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.6f, Satellites: %i\n",lat, lon, alt, sat);
    Serial.printf("=================================================================\n\n");
  }
  if(sampleTimer.isTimerReady()){
    aqOneResult = getAirQuality();
    Serial.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.6f, Satellites: %i\n",lat, lon, alt, sat);
    tempResult = getBme();
    Serial.printf("TempF = %02f\n", tempResult);
    sampleTimer.startTimer(AQTIMER);
    sendData(tempResult, aqOneResult, lat);
  }
}

int getAirQuality(){
  //Serial.printf("im here");
  Wire.requestFrom(SENSOR_ADDRESS, 29); // Request 29 bytes of data
  delay(100); // Allow time for sensor to respond
    // Check if data is available
  if (Wire.available() >= 29) {  //??
  // Read response from sensor
    for (int i = 0; i < 29; i++) {
      data[i] = Wire.read();
    }
    aqOne = data[5] | data[6];
    Serial.printf("sensor# = %i\nPM 1.0 = %i\nPM 2.5 = %i\nPM c10 = %i\n", data [3] | data[4], data[5] | data[6], data[7] | data[8], data[11] | data[12]);
  }
  return aqOne;     
}
// Get GPS data
void getGPS(float *latitude, float *longitude, float *altitude, int *satellites) {
  int theHour;
  theHour = GPS.hour + TIMEZONE;
  if (theHour < 0) {
    theHour = theHour + 24;
  }

  Serial.printf("Time: %02i:%02i:%02i:%03i\n", theHour, GPS.minute, GPS.seconds, GPS.milliseconds);
  Serial.printf("Dates: %02i-%02i-%02i\n", GPS.month, GPS.day, GPS.year);
  Serial.printf("Fix: %i, Quality: %i", (int)GPS.fix, (int)GPS.fixquality);
  if (GPS.fix) {
    *latitude = GPS.latitudeDegrees;
    *longitude = GPS.longitudeDegrees;
    *altitude = GPS.altitude;
    *satellites = (int)GPS.satellites;
  
  }
}
float getBme(){
  float tempC, pressPA,humidRH, tempF, convertedPA;
  tempC = myReading.readTemperature ();
  pressPA = myReading.readPressure ();
  convertedPA = pressPA*0.00029530;
  humidRH = myReading.readHumidity ();
  tempF = (tempC*9/5)+32;
  return tempF;
}
// Send data to IoT Classroom LoRa basestation in format expected
void sendData(float temp, int aq, float latt) {
  char buffer[60];
  sprintf(buffer, "AT+SEND=%i,60,%0.2f,%i,%0.2f\r\n", SENDADDRESS, temp, aq, latt);
  Serial1.printf("%s",buffer);
  //Serial1.println(buffer); 
  delay(1000);
  if (Serial1.available() > 0)
  {
    Serial.printf("Awaiting Reply from send\n");
    String reply = Serial1.readStringUntil('\n');
    Serial.printf("Send reply: %s\n", reply.c_str());
  }
}

// Configure and Initialize Reyax LoRa module
void reyaxSetup(String password) {
  // following four paramaters have most significant effect on range
  // recommended within 3 km: 10,7,1,7
  // recommended more than 3 km: 12,4,1,7
  const int SPREADINGFACTOR = 10;
  const int BANDWIDTH = 7;
  const int CODINGRATE = 1;
  const int PREAMBLE = 7;
  String reply; // string to store replies from module

  Serial1.printf("AT+ADDRESS=%i\r\n", RADIOADDRESS); // set the radio address
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply from address\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Reply address: %s\n", reply.c_str());
  }

  Serial1.printf("AT+NETWORKID=%i\r\n", RADIONETWORK); // set the radio network
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply from networkid\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Reply network: %s\n", reply.c_str());
  }

  Serial1.printf("AT+CPIN=%s\r\n", password.c_str());
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply from password\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Reply: %s\n", reply.c_str());
  }

  Serial1.printf("AT+PARAMETER=%i,%i,%i,%i\r\n", SPREADINGFACTOR, BANDWIDTH, CODINGRATE, PREAMBLE);
  delay(200);
  if (Serial1.available() > 0) {
    reply = Serial1.readStringUntil('\n');
    Serial.printf("reply: %s\n", reply.c_str());
  }

  Serial1.printf("AT+ADDRESS?\r\n");
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Radio Address: %s\n", reply.c_str());
  }

  Serial1.printf("AT+NETWORKID?\r\n");
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Radio Network: %s\n", reply.c_str());
  }

  Serial1.printf("AT+PARAMETER?\r\n");
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("RadioParameters: %s\n", reply.c_str());
  }

  Serial1.printf("AT+CPIN?\r\n");
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Radio Password: %s\n", reply.c_str());
  }
}