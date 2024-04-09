#include "Particle.h"
#include "IoTTimer.h"
#include "credentials.h"

#define SENSOR_ADDRESS 0x40 // Change this to the address obtained from the I2C scan

byte data[29];
uint16_t pmOneZero;
String password = "AA4104132968BA2224299079021594AC"; // AES128 password
String myName = "Borrowed";
const int RADIOADDRESS = 0xA2; //it will be a value between 0xC1 - 0xCF can be anything i used 0XA1 BASE AND 0XA2 REMOTE
const int TIMEZONE = -6;
IoTTimer sampleTimer;// timer for led onboard

// Define Constants
const int RADIONETWORK = 1;    // range of 0-16
const int SENDADDRESS = 0XA1;   // address of radio to be sent to

void reyaxSetup(String password);
//void getGPS(float *latitude, float *longitude, float *altitude, int *satellites);
void sendData(u16_t partOneZero); //send particulates
void getdata();

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {

    Wire.begin();
    Serial.begin(9600);
    waitFor(Serial.isConnected, 1000);
    Serial1.begin(115200);
    reyaxSetup(password);

    // Initialize I2C communication
    Wire.beginTransmission(SENSOR_ADDRESS);
    if (Wire.endTransmission() == 0) {
        Serial.println("Sensor found at address 0x40");
    } else {
        Serial.println("No sensor found at address 0x40.");
    }
    sampleTimer.startTimer(10000);
    Serial.printf("timer started");
}

void loop() {
    // Request data from sensor
    if(sampleTimer.isTimerReady()){
      getdata();
      sampleTimer.startTimer(10000); 
    }
    
}

void getdata(){
  //Serial.printf("im here");
  Wire.requestFrom(SENSOR_ADDRESS, 29); // Request 29 bytes of data
  delay(100); // Allow time for sensor to respond
    // Check if data is available
  if (Wire.available() >= 29) {  //??
  // Read response from sensor
    for (int i = 0; i < 29; i++) {
      data[i] = Wire.read();
    }
    pmOneZero = data[5] | data[6];
    //Serial.printf("pmOneZero = %i\n", pmOneZero);
    Serial.printf("sensor# = %i\nPM 1.0 = %i\nPM 2.5 = %i\nPM c10 = %i\n", data [3] | data[4], data[5] | data[6], data[7] | data[8], data[11] | data[12]);
    delay(1000);
    sendData(pmOneZero); 
  }     
}

// Send data to IoT Classroom LoRa basestation in format expected
void sendData(u16_t partOneZero) {
  char buffer[60];
  sprintf(buffer, "AT+SEND=%i,60,%i\n", SENDADDRESS,partOneZero);
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

