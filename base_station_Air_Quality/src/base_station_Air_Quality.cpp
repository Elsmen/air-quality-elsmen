#include "Particle.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"
#include <JsonParserGeneratorRK.h>
#include "IoTTimer.h"


String password = "AA4104132968BA2224299079021594AB"; // AES128 password
String myName = "Thor2";
const int RADIOADDRESS = 0xA1; // It will be a value between 0xC1 - 0xCF can be anything though up to 255
const int TIMEZONE = -6;
unsigned int last, lastTime;
IoTTimer publishTimer;
const int PUBTIME = 12000;

// Define Constants
const int RADIONETWORK = 1;    // range of 0-16
const int SENDADDRESS = 0xA2;   // address of radio to be sent to if I send back to another LoRa device

void reyaxSetup(String password);
//void sleepULP();

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Publish tempDrone = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/capTemp");
Adafruit_MQTT_Publish humidityDrone = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/capHumidity");
Adafruit_MQTT_Publish airOneDrone = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/capAirOne");
Adafruit_MQTT_Publish airTwoDrone= Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/capAirTwo");
Adafruit_MQTT_Publish airTenDrone = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/capAirTen");
Adafruit_MQTT_Publish gpsDrone = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/capGPS");


/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();

//Declare function for json collector
void createEventPayLoad (float lat, float lon, float alt);

// Let Device OS manage the connection to the Particle Cloud comment out or put in manual when connecting to mqtt
//SYSTEM_MODE(AUTOMATIC);

void setup() {

    Wire.begin();
    Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);
    Serial1.begin(115200);
    delay(5000);
    reyaxSetup(password);

    WiFi.on();
    WiFi.connect();
    while(WiFi.connecting()) {
      Serial.printf(".");
    }
    Serial.printf("\n\n");
    publishTimer.startTimer(PUBTIME);
}

void loop() {
  MQTT_connect();
  MQTT_ping();
  
  if (Serial1.available())  { // full incoming buffer: +RCV=203,50,mydata,
    //Serial.printf("here i am ");
    String parse0 = Serial1.readStringUntil('=');  //+RCV
    String parse1 = Serial1.readStringUntil(',');  // address received from device
    String parse2 = Serial1.readStringUntil(',');  // buffer length
    String parse3 = Serial1.readStringUntil(',');  // data received from remote Tempf
    String parse4 = Serial1.readStringUntil(',');  // data received from remote Air Quality PM1.0
    String parse5 = Serial1.readStringUntil(',');  // data received from remote Latitude
    String parse6 = Serial1.readStringUntil(',');  // data received from remote Longitude
    String parse7 = Serial1.readStringUntil(',');  // data received from remote altutude
    String parse8 = Serial1.readStringUntil(','); // data received from humidity
    String parse9 = Serial1.readStringUntil(',');  // data received from remote Air Quality PM2.5
    String parse10 = Serial1.readStringUntil('n');  // data received from remote Air Quality PM10.0

    if(publishTimer.isTimerReady()) {
    if (parse3.length() > 0 && parse4.length() > 0 && parse5.length() > 0) {
        // Print only if all parsed variables contain data
      Serial.printf("TempF: %s\nAir Quality1: %s\nLatitude: %s\nLongitude: %s\nAltutude: %s\nAQ2: %s\nAQten: %s\n", parse3.c_str(), parse4.c_str(), parse5.c_str(), parse6.c_str(), parse7.c_str(), parse8.c_str(), parse9.c_str(), parse10.c_str());
      createEventPayLoad (atof((char *)parse5.c_str()), atof((char *)parse6.c_str()), atof((char *)parse7.c_str())); //json package for GPS
      if(mqtt.Update()) {
        tempDrone.publish(atof((char *)parse3.c_str()));
        airOneDrone.publish(atof((char *)parse4.c_str()));
        airTwoDrone.publish(atof((char *)parse9.c_str()));
        airTenDrone.publish(atof((char *)parse10.c_str()));
        humidityDrone.publish(atof((char *)parse8.c_str()));
        //Serial.printf("Publishing %0.2f \n",atof((char *)parse3.c_str())); 
      }
    }
      publishTimer.startTimer(PUBTIME); 
      lastTime = millis();
    }
  }
  //sleepULP(); apparently this is not supported by the Photon2 on serial comm
}
  
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
void createEventPayLoad (float lat, float lon, float alt) {
  JsonWriterStatic <256> jw ;{
    JsonWriterAutoObject obj (&jw);
    jw.insertKeyValue ("lat", lat);
    jw.insertKeyValue ("lon", lon);
    jw.insertKeyValue ("alt", alt);
  }
  if (mqtt.Update()) {
    gpsDrone.publish(jw.getBuffer());
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


// tried using with photon 2 but does not work with a wake up from serial1 from LoRa
// void sleepULP() {
//   SystemSleepConfiguration config;
//   config.mode(SystemSleepMode :: ULTRA_LOW_POWER).usart(Serial1).duration(120000);
//   //config.mode (SystemSleepMode :: ULTRA_LOW_POWER).gpio(D0, RISING ).duration( sleepDuration );
//   SystemSleepResult result = System.sleep (config);
//   delay (1000) ;
//   if (result.wakeupReason() == SystemSleepWakeupReason:: BY_USART) {
//     Serial.printf (" Awakened by Serial1 %i\n", result.wakeupPin() );
//   }
//   if (result.wakeupReason() == SystemSleepWakeupReason:: BY_RTC ) {
//     Serial.printf (" Awakened by RTC \n");
//   }
//}  
