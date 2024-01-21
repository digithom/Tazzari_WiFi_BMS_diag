#include <hspi_slave.h>
#include <SPISlave.h>


#include <ESP8266WiFiMulti.h>
//#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266mDNS.h>
#include <Timezone.h>

#include <EEPROM.h>
//#include <SPI.h>
#include <Ethernet.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <mcp_can.h>
//#include <mcp2515.h>

#define DEMO 1

extern "C" {
#include "user_interface.h"
}

#define MAX_NTP_RETRIES 5
#ifdef DEMO
#define HOSTNAME "Tazzari-DIAG-Devel-"
#else
#define HOSTNAME "Tazzari-DIAG-"
#endif
#include "Tazzari_Diagnosis.h"

os_timer_t myTimer;
 
//SSID and Password of your WiFi router

//#define TEST  1


const char* ssid_2 = "thombeam";
const char* password_2 = "thombeamcentinarola";
const char* ssid_1 = "thombeamEXTD_2";
const char* password_1 = "thombeamcentinarola";
const char* ssid = "thombeamGround0";
const char* password = "thombeammamma";
const char* ssid_3 = "DoryROD";
const char* password_3 = "digithom2020";



char str_time[10];
char str_date[12];
char str_date_mysql[12];
char str_load_time_begin[10];
char str_load_date_begin[12];

#define MAX_UNHANDLED_TELEGRAMS 20

unsigned short Vcelle[24];
int           unhandledTelegramsCounter = 0;
bmsTelegrams  BMSTelegrams[6];
bmsTelegrams  BMSunhandledTelegrams[MAX_UNHANDLED_TELEGRAMS];
short         BMScurrent_tmp;
float         BMScurrent;
long          tmpVBattTot = 0;
float         BMSTensioneBatteria = 0;
float         BMSkWh=0;

int timer_counter=0;
bool tickOccured=false;
bool step_change_pulse = false;
bool wifi_connected = false;
bool timezone_acquired = false;
bool ntp_req_sent = false;

int timezone_countdown_acquire = 0;
int tmp_x;

bool tick_1s = false;
int  counter_1s = 0;

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString1[128];                        // Array to store serial string
char msgString2[128];                        // Array to store serial string
String msgStringS;
bool  telegramReceived=false;
unsigned int telegram_counter = 0;
bool can_initialized = false;



unsigned int localPort = 8888;       // local port to listen for UDP packets
char timeServer[] = "it.pool.ntp.org"; // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 
                                // 48 bytes of the message
 
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming 
                                     //and outgoing packets

unsigned long epoch;
unsigned long timestamp;
bool ntptime_acquired = false;
int  ntp_retries = 0;
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 26, 120}; // Central European Summer Time, 2 hrs offset
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 29, 60}; // Central European Standard Time, 1 hr offset
Timezone CE(CEST, CET);
TimeChangeRule * tcr;

//MDNSResponder mdns;
//ESP8266WiFiMulti WiFiMulti;
ESP8266WiFiMulti WiFiMulti;
ESP8266WebServer server(80); //Server on port 80
//WebSocketsServer webSocket = WebSocketsServer(81);
WiFiUDP Udp;

HTTPClient http;

#define CAN0_INT D2                              // Set INT to pin D2
//#define CAN0_INT 19                              // Set INT to pin D2
//MCP_CAN CAN0(D8);                               // Set CS to pin D8
//MCP_CAN CAN0(16);                               // Set CS to pin D8
//MCP_CAN CAN0(&SPI, 16);                               // Set CS to pin D8
MCP_CAN CAN0(&SPI, D8);                               // Set CS to pin D8
//MCP2515 CAN0();


bool handleFileRead(String path) { // send the right file to the client (if it exists)
  if (path.endsWith("/")) path += "index.html";         // If a folder is requested, send the index file
  String contentType = getContentType(path);            // Get the MIME type
  if (SPIFFS.exists(path)) {                            // If the file exists
    File file = SPIFFS.open(path, "r");                 // Open it
    size_t sent = server.streamFile(file, contentType); // And send it to the client
    file.close();                                       // Then close the file again
    return true;
  }
  return false;                                         // If the file doesn't exist, return false
}

String getContentType(String filename) { // convert the file extension to the MIME type
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".jpeg")) return "image/jpeg";
  else if (filename.endsWith(".png")) return "image/png";
  return "text/plain";
}

// start of timerCallback
void timerCallback(void *pArg) {

    
      counter_1s++;
      if (counter_1s >=10) {
        tick_1s=true;
        counter_1s=0;
      }
      tickOccured = true;
      timer_counter++;
      timezone_countdown_acquire++;
      // Refresh NTP time every 24 hours or 10 minutes if never acquired before
      if ((timezone_acquired && timezone_countdown_acquire > 60*60*24*10) || (!timezone_acquired && timezone_countdown_acquire > 60*10*1)) {
        timezone_acquired = false;
        ntp_req_sent = false;
        ntp_retries = 0;
      }

// Calcolo l'incremento di kWh ogni 1/10 di secondo aggiungendo ogni volta i kWmS calcolati

      BMSkWh -= (BMScurrent * BMSTensioneBatteria) / 36000000;

      

} // End of timerCallback

void handleTelegram (unsigned int id, unsigned char len, unsigned char *telegramData) {
    int base_idx;
    bool newTelegram;
 
    switch (id) {
        case 0x165:         // BMS 1 ADDR 1
            base_idx = 0;
            break;
 
        case 0x1e5:         // BMS 1 ADDR 2
            base_idx = 4;
            break;
 
        case 0x166:         // BMS 2 ADDR 1
            base_idx = 8;
            break;
 
        case 0x1e6:         // BMS 2 ADDR 2
            base_idx = 12;
            break;
 
        case 0x167:         // BMS 3 ADDR 1
            base_idx = 16;
            break;
 
        case 0x1e7:         // BMS 3 ADDR 2
            base_idx = 20;
            break;

        case 0x267:
            BMScurrent_tmp = *(telegramData+4) << 8;
            BMScurrent_tmp = BMScurrent_tmp + *(telegramData+5);
            BMScurrent = (float)BMScurrent_tmp / 10;
            base_idx = -1;
            break;
 
        default:
            base_idx = -1;
            break;
    }
 
    if (base_idx >= 0) {
      unsigned int tmp_telegram_idx;
      tmp_telegram_idx = base_idx >> 2;
      // Copio il telegramma per lo storico      
      BMSTelegrams[tmp_telegram_idx].Id = id;
      BMSTelegrams[tmp_telegram_idx].len = len;      
      for (int x=0; x<8; x++) BMSTelegrams[tmp_telegram_idx].tdata[x] = telegramData[x];
      
      
      for (int x=0; x<4; x++) {
      // Siccome c'è la questione Big Endian - Little Endian forse serve quest'altra versione
        Vcelle[base_idx + x] = *(telegramData+(x*2)) << 8;
        Vcelle[base_idx + x] = Vcelle[base_idx + x] + *(telegramData+(x*2)+1);
       }

       tmpVBattTot = 0;
       for (int x=0; x<24; x++) tmpVBattTot += (float)Vcelle[x];
       BMSTensioneBatteria = (float)tmpVBattTot/1000;
    } else {
      newTelegram = true;
      // Verifico se l'ho già ricevuto in passato
      if (unhandledTelegramsCounter > 0) {  // lo faccio solo se il buffer non è vuoto ovviamente
        for (int i=0; i<unhandledTelegramsCounter; i++) {
          // Se l'ho già ricevuto in passato aggiorno i dati ricevuti
          if (BMSunhandledTelegrams[i].Id == id) {
            BMSunhandledTelegrams[i].len = len;
            for (int x=0; x<len; x++) BMSunhandledTelegrams[i].tdata[x] = telegramData[x];
            newTelegram = false;
            break;
          }
        }
      }
      // Se non l'ho mai ricevuto lo metto in coda agli altri (se ho ancora spazio nel buffer allocato)
      if (newTelegram) {
        if (unhandledTelegramsCounter < MAX_UNHANDLED_TELEGRAMS) {
          BMSunhandledTelegrams[unhandledTelegramsCounter].Id = id;
          BMSunhandledTelegrams[unhandledTelegramsCounter].len = len;
          for (int x=0; x<len; x++) BMSTelegrams[unhandledTelegramsCounter].tdata[x] = telegramData[x];
          unhandledTelegramsCounter++;
          newTelegram = false;
        }
      }
    }
}

void handleCommands() {
  
}


// Questa funzione carica nel database remoto i dati di caricamento del vaso d'espansione
// e i dati del grafico


void handleBMSData () {
  unsigned int x;
  unsigned int m;
  unsigned int slot;
  
  slot = 0;
  String s = "";

  s="<html>";
  s+="Telegrammi ricevuti ";
  s+=int(telegram_counter);
  s+="<br><br>\n";
  s+="Corrente ";
  s+=BMScurrent;
  s+="A";  
  s+="<br>\n";
  s+="Tensione Batteria ";
  s+=String(BMSTensioneBatteria, 3);
  s+="V";  
  s+="<br>\n";
  s+="Potenza ";
  s+=String(BMSTensioneBatteria * BMScurrent, 3);
  s+="W";  
  s+="<br>\n";
  s+="Energia ";
  s+=String(BMSkWh, 6);
  s+="kWh";  
  s+="<br><br>\n";
  s+="<table>\n<tr><th>Cella</th><th>Tensione</th></tr>";
  for (int i=0; i<24; i++) {
    s+="<tr>\n";
    s+="<td><center>";
    s+=i+1;
    s+="</td><td> ";
    s+=String((float)Vcelle[i]/1000.f, 3);
    s+="</td>";
//    s+="<br>\n";
  }
  s+="</table>";
  s+="<br>";
   s+="<table>\n<tr><th>Id</th><th>data</th></tr>";
  for (int i=0; i<6; i++) {
    s+="<tr>\n";
    s+="<td><center>";
    s+=String(BMSTelegrams[i].Id, HEX);
    s+="</td><td> ";
    for (int m=0; m<8; m++) {
      s+=String(BMSTelegrams[i].tdata[m], HEX);
      s+=" ";
    }
    s+="</td>";
  }
  s+="</table><br><br>";

  if (unhandledTelegramsCounter > 0) {
    s+="Telegrammi non gestiti";
    s+="<br>\n";
    s+="<table>\n<tr><th>Id</th><th>len</th><th>data</th></tr>";
    for (int i=0; i<unhandledTelegramsCounter; i++) {
      s+="<tr>\n";
      s+="<td><center>";
      s+=String(BMSunhandledTelegrams[i].Id, HEX);
      s+="</td>";
      s+="<td><center>";
      s+=BMSunhandledTelegrams[i].len;
      s+="</td><td>";
      for (int m=0; m<8; m++) {
        s+=String(BMSunhandledTelegrams[i].tdata[m], HEX);
        s+=" ";
      }
      s+="</td></tr>\n";
    }
    s+="</table>";
  }

  s+="</html>";


  
  server.send(200, "text/html", s);
  
}

void handleSendTestTelegram() {
  byte sndStat;
  byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
  
  sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
  
  if(sndStat == CAN_OK){
    server.send(200, "text/html", "Telegram sent");
  } else {
    server.send(200, "text/html", "error: Telegram not sent");
  }
  server.send(200, "text/html", "");
}

void handleCMD_Restart() {
  server.send(200, "text/html", "");
  ESP.restart();
}



//===============================================================
// This routine sends to server last collected pressure logs
//===============================================================
/*
void sendLog () {
  HTTPClient http;
  http.begin("mc.digithom.it", 80, "/rest/vars/set/1/12/999");
  http.addHeader("Content-Type", "text/plain");
  http.addHeader("Authorization", "Basic dXNlcm5hbWU6cGFzc3dvcmQ=");
  auto httpCode = http.POST(payload);
}
*/

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(char* address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
 
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void canDataReceivedISR () {
  telegramReceived = true;
}

//==============================================================
//                  SETUP
//==============================================================
void setup(void){
  
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  os_timer_setfn(&myTimer, timerCallback, NULL);
  os_timer_arm(&myTimer, 100, true);


  SPI.setClockDivider(SPI_CLOCK_DIV128);
  /*
  while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Error Initializing MCP2515...");
    delay (100);
  }
  Serial.println("Error Initializing MCP2515...");
*/
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    can_initialized = true;
  } else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
//  Serial.println("MCP2515 Library Receive Example...");

//  attachInterrupt(digitalPinToInterrupt(CAN0_INT), *canDataReceivedISR, FALLING);


  SPIFFS.begin();

  network_setup();

#ifdef DEMO
  BMScurrent = -10.8;
  Vcelle[0] = 3330;
  Vcelle[1] = 3331;
  Vcelle[2] = 3332;
  Vcelle[3] = 3333;
  Vcelle[4] = 3334;
  Vcelle[5] = 3335;
  Vcelle[6] = 3336;
  Vcelle[7] = 3337;
  Vcelle[8] = 3338;
  Vcelle[9] = 3340;
  Vcelle[10] = 3341;
  Vcelle[11] = 3342;
  Vcelle[12] = 3343;
  Vcelle[13] = 3344;
  Vcelle[14] = 3345;
  Vcelle[15] = 3346;
  Vcelle[16] = 3347;
  Vcelle[17] = 3320;
  Vcelle[18] = 3321;
  Vcelle[19] = 3322;
  Vcelle[20] = 3323;
  Vcelle[21] = 3324;
  Vcelle[22] = 3325;
  Vcelle[23] = 3326;
  for (int i=0; i<24; i++) BMSTensioneBatteria += (float)Vcelle[i]/1000;
#endif


}

void network_setup () {
  timezone_acquired = false;
  ntp_req_sent = false;

  Serial.print("Chip ID: 0x");
  Serial.println(ESP.getChipId(), HEX);

  // Set Hostname.
  String hostname(HOSTNAME);
// Con questa parte, viene aggiunto al nome il Chip ID  
//  hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(hostname);

  // Print hostname.
  Serial.println("Hostname: " + hostname);
  //Serial.println(WiFi.hostname());
  WiFi.disconnect();
  WiFi.mode (WIFI_STA);
  WiFiMulti.addAP(ssid, password);
  WiFiMulti.addAP(ssid_1, password_1);
  WiFiMulti.addAP(ssid_2, password_2);
  WiFiMulti.addAP(ssid_3, password_3);
//  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");
 
  // Wait for connection
//  while (WiFi.status() != WL_CONNECTED) {
  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP
  wifi_connected = true;
  digitalWrite(LED_BUILTIN, LOW);

  server.on("/bms_data.json", handleBMSData);
  server.on("/index.html", handleBMSData);
  /*server.on("/send_test", handleSendTestTelegram);
  server.on("/cmd_disable", handleCMD_Disable);
  server.on("/cmd_enable", handleCMD_Enable);
  server.on("/cmd_load", handleCMD_Load);*/
  server.on("/cmd_restart", handleCMD_Restart);
  server.onNotFound([]() {                              // If the client requests any URI
    if (!handleFileRead(server.uri()))                  // send it if it exists
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
  });

 
  server.begin();                  //Start server
  Serial.println("HTTP server started");
  Serial.println("WebSocket server started");

  Udp.begin(8888);
  
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  
}


//==============================================================
//                     LOOP
//==============================================================
void loop(void){
  ArduinoOTA.handle();
  server.handleClient();          //Handle client requests
//  webSocket.loop();
//  if (tick_1s  && !timezone_acquired && (ntp_retries <= MAX_NTP_RETRIES)) {
  if (tick_1s) {
    
    tick_1s = false;
    if (!can_initialized) {
      if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
        Serial.println("MCP2515 Initialized Successfully!");
        can_initialized = true;
      } else Serial.println("Error Initializing MCP2515...");
    }
    
    if (!timezone_acquired && (ntp_retries <= MAX_NTP_RETRIES)) {
      if (!ntp_req_sent) {
        Serial.println("It is time to refresh NTP time, sending ntp request ... " );
        sendNTPpacket(timeServer);
        ntp_retries = 0;
        ntp_req_sent = true;
      } else if ( Udp.parsePacket() ) {
        Serial.println("NTP answer recieved, processing data ... " );
        // We've received a packet, read the data from it
        Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet 
                                                 // into the buffer
        //the timestamp starts at byte 40 of the received packet 
        // and is four bytes, or two words, long. First, 
        // esxtract the two words:
     
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        // combine the four bytes (two words) into a long integer
        // this is NTP time (seconds since Jan 1 1900):
        unsigned long secsSince1900 = highWord << 16 | lowWord;
    
    
    
        Serial.print("Seconds since Jan 1 1900 = " );
        Serial.println(secsSince1900);
     
        // now convert NTP time into everyday time:
        Serial.print("Unix time = ");
        // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
        const unsigned long seventyYears = 2208988800UL;
        // subtract seventy years:
    
    //    unsigned long epoch = secsSince1900 - seventyYears;
        epoch = CE.toLocal(secsSince1900 - seventyYears, &tcr);
        setTime(epoch);
        // print Unix time:
        Serial.println(epoch);
     
     
        // print the hour, minute and second:
        Serial.print("The UTC time is ");       // UTC is the time at Greenwich 
                                                // Meridian (GMT)
    
        Serial.print(hour());
        Serial.print(":");
        Serial.print(minute());
        Serial.print(":");
        Serial.print(second());
        Serial.print("  ");
        Serial.print(day());
        Serial.print("/");
        Serial.print(month());
        Serial.print("/");
        Serial.print(year());
        
        Serial.println();
        Serial.println(now());
                                                  
        ntp_retries = MAX_NTP_RETRIES + 1;
        ntp_req_sent = false;
        timezone_acquired = true;
      } else {
        Serial.println("No ntp received\n");
        ntp_retries++;
      }
      timezone_countdown_acquire = 0;
    }
  }
  /*  

  if ((pump_step == ST_LOADING) || (pump_step == ST_GRACE_PERIOD) || timer_counter >=10) {
    timer_counter = 0;
    Serial.print(" Analog RAW VAL ");
    Serial.print(pressure);
    Serial.print("  Step ");
    Serial.println(pump_step);

    tickOccured = false;

 }
   */  
  if (!digitalRead(CAN0_INT)) telegramReceived = true;
  if (telegramReceived) {
    telegramReceived = false;
    telegram_counter++;
    
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    
    if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString1, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString1, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  
    msgStringS = "";
  
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString2, " REMOTE REQUEST FRAME");
      msgStringS = msgString2;

    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString2, " 0x%.2X", rxBuf[i]);
        msgStringS+=msgString2;
      }
    }

    handleTelegram (rxId, len, rxBuf);
        
  /*  sprintf(msgString1, "ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
    msgStringS = "";
    if (len >0 ) {
      for(byte i = 0; i<len; i++){
          sprintf(msgString2, " 0x%.2X", rxBuf[i]);
          msgStringS+=msgString2;
      }
    } */
    

  
  }
  

  if (tickOccured) {
    timestamp = now(); 
    tickOccured = false;
    sprintf (str_time, "%02d:%02d:%02d",hour(),minute(),second());
    sprintf (str_date, "%d/%d/%d",day(),month(),year());
    sprintf (str_date_mysql, "%d/%d/%d",year(),month(),day());
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println ("Not Connected\nCalling Network Setup ...\n");
      network_setup();
    }
/*    if (cmd_load_data_to_web) {
      cmd_load_data_to_web = false;
      handleLoadHistory ();
    } */
  }
}
