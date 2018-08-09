
/*  ESP8266-12E / NodeMCU LocoNet Gateway

 *  Source code can be found here: https://github.com/tanner87661/LocoNet-MQTT-Gateway
 *  Copyright 2018  Hans Tanner IoTT
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 *  in compliance with the License. You may obtain a copy of the License at:
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
 *  on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License
 *  for the specific language governing permissions and limitations under the License.
 */

#define sendLogMsg

//#define useIOTAppStory   //uncomment this to use OTA mechanism via IOTAppStory.com. If commented out, WifiManager will be used instead

#ifdef sendLogMsg
  #include "EspSaveCrash.h" //standard library, install using library manager
#endif

#ifndef useIOTAppStory
  #define useWifiManager
#endif


#ifdef useIOTAppStory
  #define APPNAME "LNGateway"
  #define VERSION "V0.9.1"
  #define COMPDATE __DATE__ __TIME__
  #define MODEBUTTON 10
#endif


#define ESP12LED D4 //onboard LED on EP8266 Chip Board
//#define LED_BUILTIN D0  //onboard LED on NodeMCU development board. 
#define pinRx    D6  //pin used to receive LocoNet signals
#define pinTx    D7  //pin used to transmit LocoNet signals

#include <LocoNetESPSerial.h> //this is a modified version of SoftwareSerial, which includes LocoNet CD Backoff and Collision detection
LocoNetESPSerial lnSerial(pinRx, pinTx, true, 256); //true is inverted signals; Note that the original SoftwareSerial library does not properly support this for writing, which was corrected in LocoNetESPSerial.cpp

#include <ArduinoJson.h> //standard JSON library, can be installed in the Arduino IDE

#include <FS.h> //standard File System library, can be installed in the Arduino IDE

#include <TimeLib.h> //standard library, can be installed in the Arduino IDE
#include <NTPtimeESP.h> //NTP time library from Andreas Spiess, download from https://github.com/SensorsIot/NTPtimeESP
#include <EEPROM.h> //standard library, can be installed in the Arduino IDE

#include <ESP8266WiFi.h> //standard library installed with ESP8266
#include <ESP8266WebServer.h> //standard library installed with ESP8266
#include <ESP8266mDNS.h> //standard library installed with ESP8266

#include <RollAvgSmall.h> //used to calculate LocoNet network load

#ifdef useIOTAppStory //OTA library, can be installed using library manager
  #include <IOTAppStory.h> //standard library, can be installed in the Arduino IDE. See https://github.com/iotappstory for more information
  IOTAppStory IAS(APPNAME, VERSION, COMPDATE, MODEBUTTON);
#endif

#ifdef useWifiManager
  #include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager or install using library manager
#endif

const char versionID[] = "1.0.0";
// SET YOUR NETWORK MODE TO USE WIFI
const char* ssid = ""; //add your ssid and password. If left blank, ESP8266 will enter AP mode and let you enter this information from a browser
const char* password = "";
String NetBIOSName = "LocoNetGateway";

#include <PubSubClient.h> //standard library, install using library manager

char mqtt_server[100] = ""; // = Mosquitto Server IP "192.168.xx.xx" as loaded from mqtt.cfg
uint16_t mqtt_port = 1883; // = Mosquitto port number, standard is 1883, 8883 for SSL connection;
char mqtt_user[100] = "";
char mqtt_password[100] = "";

#ifdef sendLogMsg
char lnLogMsg[] = "lnLog";
#endif
char lnPingTopic[] = "lnPing";  //ping topic, do not change. This is helpful to find Gateway IP Address if not known. 
char lnBCTopic[100] = "lnBC";  //default topic, can be specified in mqtt.cfg. Useful when sending messages from 2 different LocoNet networks
char lnEchoTopic[100] = "lnEcho"; //default topic, can be specified in mqtt.cfg
char lnOutTopic[100] = "lnOut"; //default topic, can be specified in mqtt.cfg

char mqttMsg[800]; //buffer used to publish messages via mqtt

String ntpServer = "us.pool.ntp.org"; //default server for US. Change this to the best time server for your region
NTPtime NTPch(ntpServer); 
int ntpTimeout = 5000; //ms timeout for NTP update request

//WiFiClientSecure wifiClient;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//intervall NTP server is contacted to update time/date
const uint32_t ntpIntervallDefault = 1800000; //half hour in milliseconds
const uint32_t ntpIntervallShort = 10000; //10 Seconds
const uint32_t lnLoadIntervall = 1000; //1 Seconds to measure network load
const uint32_t lnPingIntervall = 30000; //30 Seconds to send ping string to MQTT server on Topic lnPing

#ifdef useIOTAppStory
//intervall IOTAppStory is contacted to check for new firmware version
  const uint32_t callHomeIntervall = 86400000; //change to 86400000 before deployument, check every 24 hours
#endif

strDateTime dateTime;
int timeZone = -5;

//Ajax Command sends a JSON with information about internal status
char ajaxCmdStr[] = "/ajax_inputs";

//constants
const uint32_t updateEEIntervall = 60000; //wait 1 Minutes before updating EEPROM data in case more changes are coming

// OTHER VARIALBES
uint32_t ntpTimer = millis();
uint32_t lnLoadTimer = millis();
uint32_t lnPingTimer = millis();
//uint32_t updateEETimer = millis();

#ifdef useIOTAppStory
  int callHomeEntry = millis();
#endif

int millisRollOver = 0; //used to keep track of system uptime after millis rollover
unsigned long lastMillis = 0;

//bool saveEEData = false;
//const int   memBase = 1024; //EEPROM location where data can be stored. Addresses below are reserved for OTA settings

RollAvgSmall networkLoad(20); //last 20 busy percentages, updated after each received message
unsigned long lastMessage = micros();
uint32_t  bytesReceived = 0;
uint32_t  bytesLastReceived = 0; //used for LocoNet Load Calculation
uint32_t  bytesTransmitted = 0;

File uploadFile;

ESP8266WebServer server(80);

bool    ntpOK = false;
bool    useNetworkMode = false;
bool    useNTP = false;
bool    useTimeStamp = true;

const int lnMaxMsgSize = 48; //max length of LocoNet message
const int lnOutBufferSize = 10; //Size of LN messages that can be buffered after receipt from MQTT before sending to LocoNet. Useful if network is congested

typedef struct {  //LocoNet receive buffer structure to receive messages from LocoNet
    bool    lnIsEcho = false;   //true: Echo; false: Regular message; 
    byte    lnStatus = 0;    //0: waiting for OpCode; 1: waiting for package data
    byte    lnBufferPtr = 0; //index of next msg buffer location to read
    byte    lnXOR = 0;
    byte    lnExpLen = 0;
    byte    lnData[lnMaxMsgSize];
} lnReceiveBuffer;    

lnReceiveBuffer lnInBuffer;

typedef struct {
    byte    lnMsgSize = 0;
    byte    lnData[48];
} lnTransmitMsg;

typedef struct { //LocoNet transmit buffer structure to receive messages from MQTT and send to LocoNet
    byte commStatus; //busy, ready, transmit, collision, 
    uint32_t lastBusyEvent = 0;
    byte readPtr = 0;
    byte writePtr = 0;
    lnTransmitMsg lnOutData[lnOutBufferSize]; //max of 10 messages in queue  
} lnTransmitQueue;

//Note: Messages sent to LocoNet will be echoed back into Receive buffer after successful transmission
lnTransmitQueue lnOutQueue;

void setup()
{
  Serial.begin(115200);

  WiFi.hostname(NetBIOSName);

#ifdef useIOTAppStory
  IAS.serialdebug(true);                                             // 1st parameter: true or false for serial debugging. Default: false
  IAS.preSetConfig("LNGateway");                               // preset Boardname
  IAS.preSetConfig("ssid","password",true);                        // preset Wifi & automaticUpdate
//  IAS.addField(NetBIOSName, "NetBIOS", "NetBIOS Name", 21);
  IAS.begin(true, false);                                                    // 1st parameter: true or false to view BOOT STATISTICS | 2nd parameter: true or false to erase eeprom on first boot of the app
#endif

#ifdef useWifiManager
  WiFiManager wifiManager; //WiFi manager allows for setup of AP information via Cell Phone
  wifiManager.autoConnect("LocoNetGatewayAP");
#endif

  Serial.println("Init SPIFFS");
  SPIFFS.begin(); //File System. Size is set to 1 MB during compile time

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ESP12LED, OUTPUT);
  digitalWrite(LED_BUILTIN, 1); //switch LEDs off (negative logic)
  digitalWrite(ESP12LED, 1);

  readNodeConfig(); //Reading configuration files from File System
  readMQTTConfig();

  //Setup and Start of the onboard web server
  server.on("/delete", HTTP_POST, handleDelete);
  server.on("/edit", HTTP_POST, [](){ returnOK(); }, handleFileUpload);
  server.onNotFound(handleNotFound); //this is the default handler
  server.begin();
  Serial.println("Server started");
  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://"); Serial.print(WiFi.localIP()); Serial.println("/");
  Serial.println(WiFi.macAddress());
#ifdef sendLogMsg
  MQTT_connect();
  sendLogMessage("Device Restart completed");
  File dataFile = SPIFFS.open("/crash.txt", "a");
  if (dataFile)
  {
    SaveCrash.print(dataFile);
    dataFile.close();
    SaveCrash.clear();
    Serial.println("Writing Crash Dump File complete");
  }
#endif
  
//start Watchdog Timer
  Serial.println("Init WDT");
  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_8S);
  ESP.wdtFeed();
}

void getInternetTime()
{
  int thisIntervall = ntpIntervallDefault;
  if (!ntpOK)
    thisIntervall = ntpIntervallShort;
  if (millis() > (ntpTimer + thisIntervall))
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("getInternetTime");
      uint32_t NTPDelay = millis();
      dateTime = NTPch.getNTPtime(timeZone, 2);
      ntpTimer = millis();
#ifdef sendLogMsg
      sendLogMessage("call getInternetTime()");
#endif
      while (!dateTime.valid)
      {
        delay(100);
        Serial.println("waiting for Internet Time");
        dateTime = NTPch.getNTPtime(timeZone, 2);
        if (millis() > NTPDelay + ntpTimeout)
        {
          ntpOK = false;
          Serial.println("Getting NTP Time failed");
          return;
        }
      }
      NTPDelay = millis() - NTPDelay;
#ifdef sendLogMsg
      sendLogMessage("call getInternetTime() complete");
#endif
      setTime(dateTime.hour, dateTime.minute, dateTime.second, dateTime.day, dateTime.month, dateTime.year);
      ntpOK = true;
      NTPch.printDateTime(dateTime);

      String NTPResult = "NTP Response Time [ms]: ";
      NTPResult += NTPDelay;
      Serial.println(NTPResult);
    }
    else
    {
#ifdef sendLogMsg
      sendLogMessage("No Internet when calling getInternetTime()");
#endif
      
    }
  }
}

#ifdef sendLogMsg
void sendLogMessage(const char* logMsg)
{
  if (!mqttClient.connected())
    MQTT_connect();
  if (mqttClient.connected())
  {
    if (!mqttClient.publish(lnLogMsg, logMsg))
    {
      Serial.println(F("Log Failed"));
    } else {
//      Serial.println(F("Log OK!"));
    }
//    mqttClient.loop();
  }
}
#endif

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  // Loop until we're reconnected --  no, not anymore, see below
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    digitalWrite(ESP12LED, false);
    // Create a random client ID
    String clientId = "LNGW";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) 
    {
      Serial.println("connected");
      // ... and resubscribe
      if (useNetworkMode)
        mqttClient.subscribe(lnBCTopic);
      else
        mqttClient.subscribe(lnOutTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      return; //break the loop to make sure web server can be accessed to enter a valid MQTT server address
    }
  }
#ifdef sendLogMsg
  sendLogMessage("MQTT Connected");
#endif
  Serial.println("MQTT Connected!");
  digitalWrite(ESP12LED, true);
}

//called when mqtt message with subscribed topic is received
void mqttCallback(char* topic, byte* payload, unsigned int length) {
//  Serial.print("Message arrived [");
//  Serial.print(topic);
//  Serial.print("] ");
//  for (int i = 0; i < length; i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println();
  StaticJsonBuffer<800> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(payload);
  if (!root.success())
  {
    Serial.println("nodeMCU parseObject() failed");
    return;
  }
  if (root.containsKey("From"))
    if (root["From"] == NetBIOSName)
    {
//      Serial.println("Refuse processing this message");
      return;
    }
//  Serial.println("Processing LnIN Message");
  if (root.containsKey("Data"))
  { 
    int hlpPtr = (lnOutQueue.writePtr + 1) % lnOutBufferSize;
//    if (hlpPtr >= lnOutBufferSize)
//      hlpPtr = 0; //Overflow
    lnOutQueue.lnOutData[hlpPtr].lnMsgSize = root["Data"].size();
    for (int j=0; j < lnOutQueue.lnOutData[hlpPtr].lnMsgSize; j++)  
      lnOutQueue.lnOutData[hlpPtr].lnData[j] = root["Data"][j];
    lnOutQueue.writePtr = hlpPtr;
  }
}

void handlePingMessage()
{
  String hlpStr = handlePingJSON();
  hlpStr.toCharArray(mqttMsg, hlpStr.length()+1);
  if (!mqttClient.connected())
    MQTT_connect();
  if (mqttClient.connected())
  {
    if (!mqttClient.publish(lnPingTopic, mqttMsg)) 
    {
      Serial.println(F("Ping Failed"));
    } else {
      Serial.println(F("Ping OK!"));
    }
//    mqttClient.loop();
  }
}

void processLNError()
{
  StaticJsonBuffer<800> jsonBuffer;
  String jsonOut = "";
  JsonObject& root = jsonBuffer.createObject();
  root["From"] = NetBIOSName;
  root["Valid"] = 0;
  if (useTimeStamp)
    root["Time"] = millis();
  JsonArray& data = root.createNestedArray("Data");
  for (byte i=0; i <= lnInBuffer.lnBufferPtr; i++)
  {
    data.add(lnInBuffer.lnData[i]);
  }
    
//  bytesReceived += lnInBuffer.lnBufferPtr + 1;
  lnInBuffer.lnStatus = 0;
  root.printTo(mqttMsg);
  if (!mqttClient.connected())
    MQTT_connect();
  if (mqttClient.connected())
  {
    if ((lnInBuffer.lnIsEcho) && (useNetworkMode))  //send echo message if echo flag is set and network mode is active
      if (!mqttClient.publish(lnEchoTopic, mqttMsg))
      {
        Serial.println(F("lnEcho Failed"));
      } else {
        Serial.println(F("lnEcho OK!"));
      }
    else  //otherwise send BC message (in direct mode, meaning the command came in via lnOutTopic)
      if (!mqttClient.publish(lnBCTopic, mqttMsg))
      {
        Serial.println(F("lnIn Failed"));
      } else {
//        Serial.println(F("lnIn OK!"));
      }
//    mqttClient.loop();
  }
}

void processLNValidMsg()
{
#ifdef sendLogMsg
      sendLogMessage("processLNValidMsg()");
#endif
//  Serial.println("Process Valid Message");
  StaticJsonBuffer<800> jsonBuffer;
  String jsonOut = "";
  JsonObject& root = jsonBuffer.createObject();
  root["From"] = NetBIOSName;
  root["Valid"] = 1;
  if (useTimeStamp)
    root["Time"] = millis();
//  bytesReceived += lnInBuffer.lnBufferPtr + 1;
  JsonArray& data = root.createNestedArray("Data");
  for (byte i=0; i <= lnInBuffer.lnBufferPtr; i++)
  {
    data.add(lnInBuffer.lnData[i]);
//    Serial.println(lnInBuffer.lnData[i]);
  }
  root.printTo(mqttMsg);
  lnInBuffer.lnStatus = 0;
  Serial.println(mqttMsg);
//  Serial.print("Echo: ");
//  Serial.println(lnInBuffer.lnIsEcho);
  if (!mqttClient.connected())
    MQTT_connect();
  if (mqttClient.connected())
  {
    if ((lnInBuffer.lnIsEcho) && (useNetworkMode))  //send echo message if echo flag is set and network mode is active
      if (!mqttClient.publish(lnEchoTopic, mqttMsg))
      {
        Serial.println(F("lnEcho Failed"));
      } else {
//        Serial.println(F("lnEcho OK!"));
      } 
    else  //echo flag set
      if (!mqttClient.publish(lnBCTopic, mqttMsg))
      {
        Serial.println(F("lnIn Failed"));
      } else {
//        Serial.println(F("lnIn OK!"));
      }
//  mqttClient.loop();
  }
#ifdef sendLogMsg
      sendLogMessage("processLNValidMsg() done");
#endif
}

//read message byte from serial interface and process accordingly
void handleLNIn(uint16_t nextInt)
{
  byte nextByte = nextInt & 0x00FF;
  byte nextFlag = (nextInt & 0xFF00) >> 8;
//  Serial.print("New Int: ");
//  Serial.print(nextInt);
//  Serial.print(" New Byte: ");
//  Serial.print(nextByte);
//  Serial.print(" New Flag: ");
//  Serial.println(nextFlag);
  if ((nextByte & 0x00FF) >= 0x80) //start of new message
  {
    if (lnInBuffer.lnStatus == 1)
      processLNError();
    lnInBuffer.lnIsEcho = ((nextFlag & 0x01) > 0);  
//    Serial.println(lnInBuffer.lnIsEcho);
    lnInBuffer.lnStatus = 1;
    lnInBuffer.lnBufferPtr = 0;
    byte swiByte = (nextByte & 0x60) >> 5;
    switch (swiByte)
    {
      case 0: lnInBuffer.lnExpLen  = 2; break;
      case 1: lnInBuffer.lnExpLen  = 4; break;
      case 2: lnInBuffer.lnExpLen  = 6; break;
      case 3: lnInBuffer.lnExpLen  = 0xFF; break;
      default: lnInBuffer.lnExpLen = 0;
    }
    lnInBuffer.lnXOR  = nextByte;
    lnInBuffer.lnData[0] = nextByte;
  }
  else
    if (lnInBuffer.lnStatus == 1) //collecting data
    {
      lnInBuffer.lnBufferPtr++; 
      lnInBuffer.lnData[lnInBuffer.lnBufferPtr] = nextByte;
      if ((lnInBuffer.lnBufferPtr == 1) && (lnInBuffer.lnExpLen == 0xFF))
      {
        lnInBuffer.lnExpLen  = nextByte & 0x007f;
      }
      if (lnInBuffer.lnBufferPtr == (lnInBuffer.lnExpLen - 1))
      {
        if ((lnInBuffer.lnXOR ^ 0xFF) == nextByte)
          processLNValidMsg();
        else
          processLNError();
        lnInBuffer.lnStatus = 0; //awaiting OpCode
      }  
      else
        lnInBuffer.lnXOR = lnInBuffer.lnXOR ^ nextByte;
    }
    else
    {
      //unexpected data byte. Ignore for the moment
      Serial.print("Unexpected Data: ");
      Serial.println(nextByte);
    }
      
}

//send message reeived from mqtt topic to LocoNet. Note that the message will e echoed back and then be processed as incoming message
void handleLNOut()
{
  int next = (lnOutQueue.readPtr + 1) % lnOutBufferSize;
  byte firstOut = lnOutQueue.lnOutData[next].lnData[0];
  int msgSize = lnOutQueue.lnOutData[next].lnMsgSize;
  if (lnSerial.cdbackoff() == lnAvailable) //keep time between test and writing first byte as short as possible
  {
    if (lnSerial.write(firstOut) > 0)
    {
      for (int j=1; j < msgSize; j++)  
      {
        if (lnSerial.write(lnOutQueue.lnOutData[next].lnData[j]) == 0)
        {
          //Collision Handling
          lnSerial.flush();
          Serial.println("Collision detected");
          return;
        }
      }
      lnOutQueue.readPtr = next;
      bytesTransmitted += msgSize;
    }
    else
    {
      //Collision Handling
      lnSerial.flush();
      Serial.println("Collision detected");
      return;
    }
  }
  else
    Serial.print(".");
}

void loop()
{
  ESP.wdtFeed();
  if (millis() < lastMillis)
  {
    millisRollOver++;
  //in case of Rollover, update all other affected timers
#ifdef useIOTAppStory
    callHomeEntry = millis();
#endif
    ntpTimer = millis();
    lnLoadTimer = millis();
    lnPingTimer = millis();
#ifdef sendLogMsg
      sendLogMessage("millis() rollover");
#endif
  }
  else
    lastMillis = millis(); 
     
#ifdef useIOTAppStory
  IAS.buttonLoop();                                                 // this routine handles the reaction of the MODEBUTTON pin. If short press (<4 sec): update of sketch, long press (>7 sec): Configuration

  if (millis() > callHomeEntry + callHomeIntervall)                           // only for development. Please change it to at least 2 hours in production
  {
    IAS.callHome();
    callHomeEntry = millis();
  }
#endif
  //-------- Your Sketch starts from here ---------------

  if (mqttClient.connected())
    mqttClient.loop();
  else
    MQTT_connect();

  if (useNTP)
    getInternetTime();

  server.handleClient();

  if (millis() > lnPingTimer + lnPingIntervall)                           // only for development. Please change it to longer interval in production
  {
    handlePingMessage();
    lnPingTimer = millis();
  }

  if (millis() > lnLoadTimer + lnLoadIntervall)                           // here we measure network load of LocoNet
  {
    float bytesDiff = bytesReceived - bytesLastReceived;
    networkLoad.update(bytesDiff);
    bytesLastReceived = bytesReceived;
    lnLoadTimer = millis();
  }

  while (lnSerial.available()) 
  {
    uint16_t thisByte = lnSerial.read();
    bytesReceived++;
    handleLNIn(thisByte);
  }

  if (lnOutQueue.readPtr != lnOutQueue.writePtr) //something to send, so process it
  {
    handleLNOut();
  }
  switch (lnSerial.cdbackoff()) //update onboard LED based on LocoNet status. This may be slightly delayed, but who cares...
  {
    case lnBusy:
    {
      digitalWrite(LED_BUILTIN, 0);
      break;
    }
    case lnAvailable:
    {
      digitalWrite(LED_BUILTIN, 1);
      break;
    }
    case lnAwaitBackoff:
    {
      digitalWrite(LED_BUILTIN, 0);
      break;
    }
  }
} //loop

// function to check existence of nested key see https://github.com/bblanchon/ArduinoJson/issues/322
bool containsNestedKey(const JsonObject& obj, const char* key) {
    for (const JsonPair& pair : obj) {
        if (!strcmp(pair.key, key))
            return true;

        if (containsNestedKey(pair.value.as<JsonObject>(), key)) 
            return true;
    }

    return false;
}

//read the MQTT config file with server address etc.
int readMQTTConfig()
{
  StaticJsonBuffer<800> jsonBuffer;
  if (SPIFFS.exists("/mqtt.cfg"))
  {
    File dataFile = SPIFFS.open("/mqtt.cfg", "r");
    if (dataFile)
    {
      Serial.print("Reading MQTT Config File ");
      Serial.println(dataFile.size());
      String jsonData;
      while (dataFile.position()<dataFile.size())
      {
        jsonData = dataFile.readStringUntil('\n');
        jsonData.trim();
        Serial.println(jsonData);
      } 
      dataFile.close();
      
      JsonObject& root = jsonBuffer.parseObject(jsonData);
      if (root.success())
      {
        if (root.containsKey("lnBCTopic"))
        {
          strcpy(lnBCTopic, root["lnBCTopic"]);
        }
        if (root.containsKey("lnEchoTopic"))
        {
          strcpy(lnEchoTopic, root["lnEchoTopic"]);
        }
        if (root.containsKey("lnOutTopic"))
        {
          strcpy(lnOutTopic, root["lnOutTopic"]);
        }
        if (root.containsKey("modeNetwork"))
        {
            useNetworkMode = bool(root["modeNetwork"]);       
        }
        if (root.containsKey("useTimeStamp"))
          useTimeStamp = bool(root["useTimeStamp"]);
        if (root.containsKey("mqttServer"))
        {
          if (containsNestedKey(root, "ip"))
            strcpy(mqtt_server, root["mqttServer"]["ip"]);
          if (containsNestedKey(root, "port"))
            mqtt_port = uint16_t(root["mqttServer"]["port"]);
          if (containsNestedKey(root, "user"))
            strcpy(mqtt_user, root["mqttServer"]["user"]);
          if (containsNestedKey(root, "password"))
            strcpy(mqtt_password, root["mqttServer"]["password"]);
          Serial.print(mqtt_server);
          Serial.print(" Port ");
          Serial.println(mqtt_port);
        }
      }
      else
        Serial.println("Error Parsing JSON");
    }
  }
}

int writeMQTTConfig()
{
  StaticJsonBuffer<800> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["lnBCTopic"] = lnBCTopic;
  root["lnEchoTopic"] = lnEchoTopic;
  root["lnOutTopic"] = lnOutTopic;
  root["modeNetwork"] = int(useNetworkMode);
  root["useTimeStamp"] = int(useTimeStamp);
  JsonObject& data = root.createNestedObject("mqttServer");
  data["ip"] = mqtt_server;
  data["port"] = mqtt_port;
  data["user"] = mqtt_user;
  data["password"] = mqtt_password;
  String newMsg = "";
  root.printTo(newMsg);
  Serial.println(newMsg);
  Serial.println("Writing MQTT Config File");
  File dataFile = SPIFFS.open("/mqtt.cfg", "w");
  if (dataFile)
  {
    dataFile.println(newMsg);
    dataFile.close();
    Serial.println("Writing Config File complete");
  }
}


//read node config file with variable settings
int readNodeConfig()
{
  StaticJsonBuffer<500> jsonBuffer;
  if (SPIFFS.exists("/node.cfg"))
  {
    File dataFile = SPIFFS.open("/node.cfg", "r");
    if (dataFile)
    {
      Serial.print("Reading Node Config File ");
      Serial.println(dataFile.size());
      String jsonData;
      while (dataFile.position()<dataFile.size())
      {
        jsonData = dataFile.readStringUntil('\n');
        jsonData.trim();
        Serial.println(jsonData);
      } 
      dataFile.close();
      
      JsonObject& root = jsonBuffer.parseObject(jsonData);
      if (root.success())
      {
        if (root.containsKey("NetBIOSName"))
        {
          String hlpStr = root["NetBIOSName"];
          NetBIOSName = hlpStr;
        }
        if (root.containsKey("useNTP"))
          useNTP = bool(root["useNTP"]);
        if (root.containsKey("NTPServer"))
        {
          String hlpStr = root["NTPServer"];
          ntpServer = hlpStr;
          NTPch.setNTPServer(ntpServer);
        }
        if (root.containsKey("ntpTimeZone"))
          timeZone = int(root["ntpTimeZone"]);
      }
    }
  } 
}

int writeNodeConfig()
{
  StaticJsonBuffer<800> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["NetBIOSName"] = NetBIOSName;
  root["useNTP"] = int(useNTP);
  root["NTPServer"] = ntpServer;
  root["ntpTimeZone"] = timeZone;
  String newMsg = "";
  root.printTo(newMsg);
  Serial.println(newMsg);
  Serial.println("Writing Node Config File");
  File dataFile = SPIFFS.open("/node.cfg", "w");
  if (dataFile)
  {
    dataFile.println(newMsg);
    dataFile.close();
    Serial.println("Writing Config File complete");
  }
}

//==============================================================Web Server=================================================

//this allows for AJAX commands from any web page. Note that this is open to be used for web pages not loaded from the ESP
bool handleAjaxCommand(String path)
{
  Serial.print("Handle Ajax Command ");
  Serial.println(path);

  if (server.args() > 0)
  {
    for (int i = 0; i < server.args(); i++)
    {
      Serial.print("Name: "); Serial.println(server.argName(i));
      Serial.print("Value: "); Serial.println(server.arg(i));
    }
  }

//step 1: analyze and handle requests from webpage
  handleAjaxRequests(path);
  
//step 2: prepare and send data update
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "POST, GET");
  server.sendHeader("Access-Control-Max-Age", "3600");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Access-Control-Allow-Headers, Authorization, X-Requested-With");
  server.send(200, "application/json", handleJSON());
  return true;
}

String extractValue(String keyWord, String request)
{
  int startPos = request.indexOf(keyWord);
  int endPos = -1;
  if (startPos >= 0)
  {
    startPos = request.indexOf("=", startPos) + 1;
//    Serial.println(startPos);
    endPos = request.indexOf("&", startPos);
//    Serial.println(endPos);
    if (endPos < 0)
      endPos = request.length();
//    Serial.println(request.substring(startPos, endPos));   
    return request.substring(startPos, endPos);  
  }
  else
    return("");
}

void handleAjaxRequests(String request) 
{
  Serial.println(request);
  bool changedNodeData = false;
  bool changedMQTTData = false;
  bool needReboot = false;
  String hlpStr;
  if (request.indexOf("MQTTServerIP=") != -1)
  {
    hlpStr = extractValue("MQTTServerIP=", request);
    hlpStr.toCharArray(mqtt_server, hlpStr.length()+1);
    mqttClient.disconnect();
    changedMQTTData = true;
  }
  if (request.indexOf("MQTTPort=") != -1)
  {
    mqtt_port = extractValue("MQTTPort=", request).toInt();
    mqttClient.disconnect();
    changedMQTTData = true;
  }
  if (request.indexOf("MQTTUser=") != -1)
  {
    hlpStr = extractValue("MQTTUser=", request);
    hlpStr.toCharArray(mqtt_user, hlpStr.length()+1);
    mqttClient.disconnect();
    changedMQTTData = true;
  }
  if (request.indexOf("MQTTPassword=") != -1)
  {
    hlpStr = extractValue("MQTTPassword=", request);
    hlpStr.toCharArray(mqtt_password, hlpStr.length()+1);
    mqttClient.disconnect();
    changedMQTTData = true;
  }
  if (request.indexOf("lnBCTopic=") != -1)
  {
    hlpStr = extractValue("lnBCTopic=", request);
    hlpStr.toCharArray(lnBCTopic, hlpStr.length()+1);
    mqttClient.disconnect();
    changedMQTTData = true;
  }
  if (request.indexOf("lnEchoTopic=") != -1)
  {
    hlpStr = extractValue("lnEchoTopic=", request);
    hlpStr.toCharArray(lnEchoTopic, hlpStr.length()+1);
    mqttClient.disconnect();
    changedMQTTData = true;
  }
  if (request.indexOf("lnOutTopic=") != -1)
  {
    hlpStr = extractValue("lnOutTopic=", request);
    hlpStr.toCharArray(lnOutTopic, hlpStr.length()+1);
    mqttClient.disconnect();
    changedMQTTData = true;
  }
  if (request.indexOf("useTimeStamp=") != -1)
  {
    useNetworkMode = bool(extractValue("modeNetwork=", request).toInt());
    changedMQTTData = true;
  }

  if (request.indexOf("ResetCtr") != -1)
  {
    bytesReceived = 0;
    bytesTransmitted = 0;
    bytesLastReceived = 0;
  }
  if (request.indexOf("useTimeStamp=") != -1)
  {
    useTimeStamp = bool(extractValue("useTimeStamp=", request).toInt());
    changedMQTTData = true;
  }
  if (request.indexOf("useNTP=") != -1)
  {
    useNTP = bool(extractValue("useNTP=", request).toInt());
    changedNodeData = true;
  }
  if (request.indexOf("ntpTimeZone=") != -1)
  {
    timeZone = extractValue("ntpTimeZone=", request).toInt();
    changedNodeData = true;
  }
  if (request.indexOf("NTPServer=") != -1)
  {
    ntpServer = extractValue("NTPServer=", request);
    NTPch.setNTPServer(ntpServer);
    changedNodeData = true;
  }
  if (request.indexOf("NetBIOSName=") != -1)
  {
    if (NetBIOSName != extractValue("NetBIOSName=", request))
    {
      NetBIOSName = extractValue("NetBIOSName=", request);
      changedNodeData = true;
      needReboot = true;
    }
  }
  if (changedNodeData)
    writeNodeConfig();
  if (changedMQTTData)
    writeMQTTConfig();
  if (needReboot | (request.indexOf("RebootNow") != -1))
    ESP.restart();
}

String handleJSON()
{
  String response;
  float float1;
  long curTime = now();
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["IP"] = WiFi.localIP().toString();
  if (WiFi.status() == WL_CONNECTED)
  {
    long rssi = WiFi.RSSI();
    root["SigStrength"] = rssi;
  }
  root["SWVersion"] = versionID;
  root["NetBIOSName"] = NetBIOSName;
  root["MQTTServerIP"] = mqtt_server;
  root["MQTTPort"] = mqtt_port;
  root["MQTTUser"] = mqtt_user;
  root["MQTTPassword"] = mqtt_password;
  root["modeNetwork"] = useNetworkMode;
  root["lnOutTopic"] = lnOutTopic;
  root["lnBCTopic"] = lnBCTopic;
  root["lnEchoTopic"] = lnEchoTopic;
  root["useTimeStamp"] = int(useTimeStamp);
  root["useNTP"] = int(useNTP);
  root["NTPServer"] = ntpServer;  
  root["ntpTimeZone"] = timeZone;
  root["mem"] = ESP.getFreeHeap();
  root["BytesReceived"] = bytesReceived;
  root["BytesTransmitted"] = bytesTransmitted;
  root["LNLoadBps"] = networkLoad.average();
  root["LNLoad100"] = 100 * networkLoad.average() / (lnLoadIntervall / 0.6);
  float1 = (millisRollOver * 4294967.296) + millis()/1000;
  root["uptime"] = round(float1);
  if (ntpOK && useNTP)
  {
    if (NTPch.daylightSavingTime(curTime))
      curTime -= (3600 * (timeZone+1));
    else
      curTime -= (3600 * timeZone);
    root["currenttime"] = curTime;  //seconds since 1/1/1970
  }
  
  root.printTo(response);
  Serial.println(response);
  return response;
}

String handlePingJSON()
{
  String response;
  float float1;
  long curTime = now();
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["IP"] = WiFi.localIP().toString();
  if (WiFi.status() == WL_CONNECTED)
  {
    long rssi = WiFi.RSSI();
    root["SigStrength"] = rssi;
  }
  root["NetBIOSName"] = NetBIOSName;
    root["mem"] = ESP.getFreeHeap();
  float1 = (millisRollOver * 4294967.296) + millis()/1000;
  root["uptime"] = round(float1);
  root.printTo(response);
  Serial.println(response);
  return response;
}

// Standard Web Server Code starts here

void returnOK() {
  server.send(200, "text/plain", "");
}

void returnFail(String msg) {
  server.send(500, "text/plain", msg + "\r\n");
}

//loading of web pages available in SPIFFS. By default, this is upload.htm and delete.htm. You can add other pages as needed
//note that ESP8266 has some difficulties in timing with Chrom browser when delivering larger pages. Normally no problem from Smart Phone browser
bool loadFromSdCard(String path){
  String dataType = "text/plain";
  if(path.endsWith("/")) path += "index.htm";

  Serial.print("Load from SPIFFS - Path: ");
  Serial.println(path);

  
  if(path.endsWith(".src")) path = path.substring(0, path.lastIndexOf("."));
  else if(path.endsWith(".htm")) dataType = "text/html";
  else if(path.endsWith(".css")) dataType = "text/css";
  else if(path.endsWith(".js")) dataType = "application/javascript";
  else if(path.endsWith(".png")) dataType = "image/png";
  else if(path.endsWith(".gif")) dataType = "image/gif";
  else if(path.endsWith(".jpg")) dataType = "image/jpeg";
  else if(path.endsWith(".ico")) dataType = "image/x-icon";
  else if(path.endsWith(".xml")) dataType = "text/xml";
  else if(path.endsWith(".pdf")) dataType = "application/pdf";
  else if(path.endsWith(".zip")) dataType = "application/zip";

  File dataFile = SPIFFS.open(path.c_str(), "r");

  if (!dataFile)
  {
    Serial.println("File not found");
    return false;
  }

  if (server.hasArg("download")) dataType = "application/octet-stream";
  Serial.println(dataFile.size());
  int siz = dataFile.size();

  int i = server.streamFile(dataFile, dataType);
  if (i != dataFile.size()) 
  {
    Serial.println(i);
    Serial.println("Sent less data than expected!");
  }
    Serial.println("all sent");
  dataFile.close();
  return true;
}

void handleFileUpload()
{
  Serial.println("Handle Upload");
  Serial.println(server.uri());
  if(server.uri() != "/edit") 
    return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START)
  {
    if(SPIFFS.exists((char *)upload.filename.c_str())) 
    {
      Serial.print("Upload selected file "); Serial.println(upload.filename);
    }
    String hlpStr = "/" + upload.filename;
    uploadFile = SPIFFS.open(hlpStr, "w");
    if (!uploadFile)
      Serial.print("Upload of file failed");
    else
      Serial.print("Upload: START, filename: "); Serial.println(upload.filename);
  } 
  else 
    if(upload.status == UPLOAD_FILE_WRITE)
    {
      if (uploadFile) 
      {
        uploadFile.write(upload.buf, upload.currentSize);
        Serial.print("Upload: WRITE, Bytes: "); Serial.println(upload.currentSize);
      }
      else
        Serial.println("Write operation failed");
    } 
    else 
      if(upload.status == UPLOAD_FILE_END)
      {
        if (uploadFile)
        { 
          uploadFile.close();
          Serial.print("Upload: END, Size: "); Serial.println(upload.totalSize);
        }
        else
          Serial.println("Closing failed");
      }
      else
      {
        Serial.print("Unknown File Status "); Serial.println(upload.status);
      }
}

//recuersive deletion not implemented
/*

void deleteRecursive(String path){
  File file = SPIFFS.open((char *)path.c_str(), "r");
  if(!file.isDirectory()){
    file.close();
    SPIFFS.remove((char *)path.c_str());
    return;
  }

  file.rewindDirectory();
  while(true) {
    File entry = file.openNextFile();
    if (!entry) break;
    String entryPath = path + "/" +entry.name();
    if(entry.isDirectory()){
      entry.close();
      deleteRecursive(entryPath);
    } else {
      entry.close();
      SPIFFS.remove((char *)entryPath.c_str());
    }
    yield();
  }

  SD.rmdir((char *)path.c_str());
  file.close();
  
}
*/

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}

void handleDelete()
{
  Serial.println("Handle Delete");
  Serial.println(server.uri());
  if(server.uri() != "/delete") 
    return;
  String path = server.arg(0);
  Serial.print("Trying to delete ");
  Serial.println((char *)path.c_str());
  if(server.args() == 0) return returnFail("BAD ARGS");
  if(path == "/" || !SPIFFS.exists((char *)path.c_str())) {
    returnFail("BAD PATH");
    return;
  }
  deleteFile(SPIFFS, (char *)path.c_str());
//  deleteRecursive(path);
  returnOK();
}

//file creation not implemented
void handleCreate(){
/*
  if(server.args() == 0) return returnFail("BAD ARGS");
  String path = server.arg(0);
  if(path == "/" || SD.exists((char *)path.c_str())) {
    returnFail("BAD PATH");
    return;
  }

  if(path.indexOf('.') > 0){
    File file = SD.open((char *)path.c_str(), FILE_WRITE);
    if(file){
      file.write((const char *)0);
      file.close();
    }
  } else {
    SD.mkdir((char *)path.c_str());
  }
  */
  returnOK();
}

//print directory not implemented
void printDirectory() {
/*
  if(!server.hasArg("dir")) return returnFail("BAD ARGS /list");
  String path = server.arg("dir");
  if(path != "/" && !SD.exists((char *)path.c_str())) return returnFail("BAD PATH");
  File dir = SD.open((char *)path.c_str());
  path = String();
  if(!dir.isDirectory()){
    dir.close();
    return returnFail("NOT DIR");
  }
  dir.rewindDirectory();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/json", "");
  WiFiClient client = server.client();

  server.sendContent("[");
  for (int cnt = 0; true; ++cnt) {
    File entry = dir.openNextFile();
    if (!entry)
    break;

    String output;
    if (cnt > 0)
      output = ',';

    output += "{\"type\":\"";
    output += (entry.isDirectory()) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += entry.name();
    output += "\"";
    output += "}";
    server.sendContent(output);
    entry.close();
 }
 server.sendContent("]");
 dir.close();
 */
}

//all calls for web pages start here
void handleNotFound(){
//this is the hook to handle async requests
  Serial.println(server.uri());
  if ((server.uri().indexOf(ajaxCmdStr) != -1) && handleAjaxCommand(server.uri())) {return; }
//this is the default file handler
  if(loadFromSdCard(server.uri())) {return;}
  String message = "SPIFFS Not available or File not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " NAME:"+server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

