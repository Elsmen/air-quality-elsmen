#include "Particle.h"
//#include "IoTTimer.h"


String password = "AA4104132968BA2224299079021594AB"; // AES128 password
String myName = "Thor2";
const int RADIOADDRESS = 0xA1; // It will be a value between 0xC1 - 0xCF can be anything though up to 255
const int TIMEZONE = -6;

// Define Constants
const int RADIONETWORK = 1;    // range of 0-16
const int SENDADDRESS = 0xA2;   // address of radio to be sent to if I send back to another LoRa device

void reyaxSetup(String password);
void sleepULP();

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {

    Wire.begin();
    Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);
    Serial1.begin(115200);
    delay(5000);
    reyaxSetup(password);
}

void loop() {
  
  if (Serial1.available())  { // full incoming buffer: +RCV=203,50,mydata,
    Serial.printf("here i am ");
    String parse0 = Serial1.readStringUntil('=');  //+RCV
    String parse1 = Serial1.readStringUntil(',');  // address received from
    String parse2 = Serial1.readStringUntil(',');  // buffer length
    String parse3 = Serial1.readStringUntil(',');  // data received from remote Temf
    String parse4 = Serial1.readStringUntil(',');  // data received from remote Air Quality PM1.0
    String parse5 = Serial1.readStringUntil('\n');  // data received from remote Latitude
    Serial.printf("TempF: %s\nAir Quality: %s\nLatitude: %s", parse3.c_str(), parse4.c_str(), parse5.c_str());
  }
  sleepULP();
}

void sleepULP() {
  SystemSleepConfiguration config ;
  config.mode(SystemSleepMode :: ULTRA_LOW_POWER).usart(Serial1);
  //config.mode (SystemSleepMode :: ULTRA_LOW_POWER).gpio(D0, RISING ).duration( sleepDuration );
  SystemSleepResult result = System.sleep (config) ;
  delay (1000) ;
  if (result.wakeupReason() == SystemSleepWakeupReason:: BY_USART) {
    Serial.printf (" Awakened by Serial1 %i\n", result.wakeupPin() );
  }
  if (result.wakeupReason() == SystemSleepWakeupReason:: BY_RTC ) {
    Serial.printf (" Awakened by RTC \n");
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

