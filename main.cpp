// copyright c-e 

#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <string>
#include "esp_task_wdt.h"
#include <math.h>

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
std::string displayzeile[6]; // zeile 0 unbenutzt--> displayzeile[6] ist 5. zeile!
// 1. zeile: Power 
// 2. zeile: In / Out
// 3. zeile: per Lora gesendeter Wert
// 4. zeile: Data Age
// 5. zeile: Sendeabstand in ms
unsigned long long int Data_Age = 0;
unsigned long _lastmillis = 0;
unsigned long _lastmillisblink = 0;
unsigned long _lastmillisdisplay = 0;
bool display_on; // für display blinkfunktion
const char identifier[6] = { '1', '2', '3', '4', '5', '6' }; // identifier für lora
bool statusTaster = HIGH;         // aktueller Status des display_taster_gpio
bool statusTasterLetzter = HIGH;  // vorheriger Status des display_taster_gpio
bool displayPowerSave; // true = display aus
int32_t powerSaldo = 0; 
int32_t Saldomerker[100]; 
int zaehler = 0;
int display_taster_gpio = 4;
int RX_gpio = 3;
uint32_t currentMillis;
uint32_t prevMillis = 0;
uint32_t prev_smlMillis = 0;
uint32_t prev_loraMillis = 0;
const String smlBegin               = "1b1b1b1b01010101";
const String smlEnd                 = "1b1b1b1b1a";
const String searchStr_Leistung     = "77070100100700ff"; //Aktuelle Leistung, OBIS-Kennzahl 1-0:16.7.0255*
String smlTemp = ""; 
String smlMsg = "";
byte crcHex[2];
byte allcrcBytes[5000];
int bytesWritten;
uint16_t claimedCRC;
float leistung = 0.0;
bool foundStart = false;
bool foundEnd = false;
int indexBegin = -1;
int indexEnd = 0;
bool toggleLED = true;

#define RADIO_SCLK_PIN              5
#define RADIO_MISO_PIN              19
#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DIO0_PIN              26
#define RADIO_RST_PIN               23
#define I2C_SDA                     21
#define I2C_SCL                     22

//##########################################################################
//                  displayText
//##########################################################################
void displayText() {
   oled.clearDisplay();
   oled.setTextColor(SSD1306_WHITE);
   oled.setCursor(0, 0);
   oled.setTextSize(2);
   oled.println(displayzeile[1].c_str());
   oled.setTextSize(1);
   oled.println(displayzeile[2].c_str());
   oled.println(displayzeile[3].c_str());
   oled.println(displayzeile[4].c_str());
   oled.println(displayzeile[5].c_str());
   oled.display();
}

void displayblinken(){
   if ((millis() - _lastmillisblink) > (500)) {
      _lastmillisblink = millis();
      if (display_on == true){
         oled.ssd1306_command(SSD1306_DISPLAYOFF);
         display_on = false;
      } else {
         oled.ssd1306_command(SSD1306_DISPLAYON);
         display_on = true;
      }
   }
}
//##########################################################################
//                  LoraSenden
// laut LoRa Air-Time Calculator https://iftnt.github.io/lora-air-time/index.html
// payload 9, preamble 8, spreading 6, 125kHz, coding 4/5, implicit header mode: Air-Time = 18.048 ms
// ==> alle 2 Sekunden senden hält 1 % Regel ein
//##########################################################################
void LoraSenden(){
   unsigned long loranow = millis();     // JETZT-Zeit
   if(loranow - prev_loraMillis < 1900){ // wenn seit letztem Senden weniger als 2 s --> nichts senden
      return;
   }     
   displayzeile[5] = "(Sendeabstand: " + std::to_string(loranow - prev_loraMillis) + " ms)";
   prev_loraMillis = loranow; 
   char payload[9];
   memcpy(payload, identifier, 6);  // Fill identifier (6 bytes)
   // Encode powerSaldo as signed 3-byte integer, powerSaldo kann ca. -1000 bis +20000 Watt sein
   payload[6] = (powerSaldo >> 16) & 0xFF;
   payload[7] = (powerSaldo >> 8) & 0xFF;
   payload[8] = powerSaldo & 0xFF;
   // --------------------------------- lora send packet -----------------------------
   LoRa.beginPacket(true); // `true` enables implicit header mode 
   LoRa.write((uint8_t*)payload, 9);
   LoRa.endPacket();
   // --------------------------------------------------------------------------------
   displayzeile[2] = "";
   displayzeile[3] = "Lora: " + std::to_string(powerSaldo);
}
//##########################################################################
//                  resetValues
//##########################################################################
void resetValues() {
  indexBegin = -1;
  indexEnd = 0;
  foundStart = false;
  foundEnd = false;
  smlTemp = "";                  // start with empty temporary SML message
  prevMillis = currentMillis;
}
//##########################################################################
//                  parse_smlMsg
//##########################################################################
void parse_smlMsg() {
   unsigned long long tmp = 0;
   int64_t value = 0;
   String hexStr = "";
   String searchStr = searchStr_Leistung;  // suche nach OBIS-Code für momentane Gesamt-Leistung
   uint16_t pos = smlMsg.indexOf(searchStr);
   if (pos >= 0) {
      pos = pos + searchStr.length() + 14;
      hexStr = smlMsg.substring(pos,  pos + 16);
      tmp = strtoull(hexStr.c_str(), nullptr, 16);
      value = static_cast<long long>(tmp);
      leistung = (float)value/100; // Rohwert in Watt umrechnen
   } else {
      Serial.println("OBIS-Code für Momentanleistung NICHT gefunden");
      leistung = -9999;
   }
}
//##########################################################################
//                  SMLoutput
//##########################################################################
void SMLoutput() {
   if (toggleLED) { // led-umschaltung bei jedem telegramm
     digitalWrite(LED_BUILTIN, HIGH);
     toggleLED = false;
   } else {
     digitalWrite(LED_BUILTIN, LOW);
     toggleLED = true;
   }
   smlMsg = smlTemp.substring(indexBegin, indexEnd + smlEnd.length());
   //Serial.println("start smlMsg");
   //Serial.println(smlMsg);
   //Serial.println("end smlMsg");
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   parse_smlMsg();
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if (leistung == -9999){ // fehlmessung - nichts über lora senden
       resetValues();
       return;
   }
   Serial.print("Zeit in ms: ");
   unsigned long now = millis();             // JETZT-Zeit
   Serial.println(now - prev_smlMillis);     // Differenz seit letztem Aufruf
   prev_smlMillis = now; 
   //Serial.println(currentMillis - prev_smlMillis);
   //prev_smlMillis = currentMillis;
   Serial.print("Leistung = ");
   Serial.print(leistung);
   Serial.println(" W");
   powerSaldo = lround(leistung);
   Serial.println(String("Leistung gerundet (powerSaldo) = ") + String(powerSaldo) + " W");
   displayzeile[1] = "P: " + std::to_string(powerSaldo) + " W";

   zaehler++;
   //Serial.print("Zaehler: ");
   //Serial.println(zaehler);
   Saldomerker[zaehler] = powerSaldo;
   if (zaehler == 1){
      Saldomerker[0] = powerSaldo;
   }
   powerSaldo = (Saldomerker[zaehler - 1] + (powerSaldo * 4)) / 5; // gewichteter mittelwert über 2 messungen
   if (zaehler == 100){ // zähler rücksetzen, damit zaehler unendlich zählt
      zaehler = 0;
      Saldomerker[0] = Saldomerker[99];
   }
   Serial.print("PowerSaldo, gewichtet, per Lora gesendet: ");
   Serial.println(powerSaldo);
   LoraSenden();
   powerSaldo = 0;
   _lastmillis = millis(); // data-age-sekunden-zähler auf 0 setzen
   displayText();
   Serial.println("########## SML-Telegramm verarbeitet ####################");
   resetValues();
}

// --------------- CRC VALIDATION --------------------------------------------------------------------
uint16_t crc16_x25(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0x8408;  // reflektiertes Polynom von 0x1021
            } else {
                crc >>= 1;
            }
        }
    }
    crc = ~crc; // Final XOR mit 0xFFFF
    crc = (crc << 8) | (crc >> 8); // Bytes vertauschen (Endian-Swap)
    return crc;
}
void ValidateCRC(int len) {
   //Serial.print("CRC: ");
   // SerialPrintHex(allcrcBytes, len);
   //for (size_t i = 0; i < len; i++){
   //   if (allcrcBytes[i] < 0x10) Serial.print("0");
   //   Serial.print(allcrcBytes[i], HEX);
   //Serial.print(' ');
   //}
   //Serial.println(' ');

   //Serial.print("claimedCRC: "); // nur für debug
   //Serial.println(claimedCRC); // nur für debug
   uint16_t crc = crc16_x25(allcrcBytes, len); // berechnung der crc
   //Serial.print("Calculated CRC: "); // nur für debug
   //Serial.println(crc); // nur für debug
   if (claimedCRC == crc) {
         //Serial.println("CRC Valid");
         SMLoutput(); // wenn EasyM60 jede Sekunde ein Telegramm sendet --> jede Sekunde aufgerufen

   } else {
         //Serial.println(telegram); // nur für debug
         Serial.println("CRC Invalid! --> Datensatz verworfen!");
   }
   return;
}

//##########################################################################
//                  bytetoHEX
//##########################################################################
String bytetoHEX(byte onebyte) {
  String str = "";
  if (onebyte < 16) str += String(0, HEX);
  str += String(onebyte, HEX);
  return str;
}
//##########################################################################
//                  string to byte
//##########################################################################
size_t hexStringToBytes(String hex, byte *outBuffer) {
    size_t len = hex.length();
    bytesWritten = 0;
    if (len % 2 != 0) return 0; // sicherstellen, dass die Länge gerade ist
    for (size_t i = 0; i < len; i += 2) {
        String byteString = hex.substring(i, i + 2);
        byte value = static_cast<byte>(strtoul(byteString.c_str(), nullptr, 16));
        outBuffer[bytesWritten++] = value;
    }
    return bytesWritten;
}
//##########################################################################
//                   LED blinker für Diagnose
//##########################################################################
void blink(int an, int aus, int anzahl){
  while(anzahl>0){
    digitalWrite(LED_BUILTIN, LOW);
    delay(an);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(aus);
    anzahl = anzahl - 1;
  }
}
//##########################################################################
//                  ReadSerialData
//##########################################################################
void ReadSerialData() {
    currentMillis = millis();
    uint8_t inByte;
    while (Serial1.available() > 0) {
      esp_task_wdt_reset();
      inByte = Serial1.read(); // read serial buffer
      smlTemp += bytetoHEX(inByte);
      //Serial.print(bytetoHEX(inByte)); // nur für debug
      indexBegin = smlTemp.indexOf(smlBegin);
      if (indexBegin >= 0) {
         //if (!foundStart) Serial.println(smlBegin + " gefunden! - Start");
         foundStart = true;
         indexEnd = smlTemp.lastIndexOf(smlEnd);
         if (indexEnd > indexBegin) {                     // end of temporary SML message reached and complete now
            //Serial.println(smlEnd + " gefunden! - End");
            foundEnd = true;
            // Read next 3 bytes, byte 2 and 3 = CRC
            while (Serial1.available() < 3) {
                delay(1);
            }
            String temp = "";
            for (int i = 0; i < 3; ++i) {
                inByte = Serial1.read();
                if (i == 0){smlTemp += bytetoHEX(inByte);} //smlTemp enthält jetzt alles in textform für CRC 
                if (i == 1){temp = bytetoHEX(inByte);}
                if (i == 2){temp += bytetoHEX(inByte);}
            }
            hexStringToBytes(smlTemp, allcrcBytes); // allcrcBytes enthält jetzt alles für CRC
            int anzbytes = bytesWritten;
            hexStringToBytes(temp, crcHex); // crcHex enthält die CRC-Bytes
            //Serial.print("CRC: "); // nur für debug
            //Serial.println(temp); // nur für debug
            claimedCRC = (static_cast<uint16_t>(crcHex[0]) << 8) | crcHex[1];
            String readCRC = smlTemp.substring(indexEnd + smlEnd.length() + 2);
            //Serial.print("CRC im Telegramm: "); // nur für debug
            //Serial.println(readCRC); // nur für debug
            //Serial.println(smlTemp); // nur für debug
            ValidateCRC(anzbytes); // von ValidateCRC() wird dann SMLoutput() aufgerufen
         }
      }
    }
}

// --------------- SETUP ---------------
// --------------- SETUP ---------------
// --------------- SETUP ---------------
void setup() {
   delay(1500);
   Serial.begin(115200);
   delay(1500);
   Serial.println(""); //neue zeile
   Serial.println("Serial Monitor started.");

   pinMode(display_taster_gpio, INPUT_PULLUP); // pin für display-ein-aus-taster
   pinMode(LED_BUILTIN, OUTPUT);    // LED als Output definieren
   digitalWrite(LED_BUILTIN, HIGH); // LED Ausschalten

   SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
   Wire.begin(I2C_SDA, I2C_SCL);
   delay(1500);
   LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);

   if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println("SSD1306 OLED-Display allocation failed.");
      return;
   } else {
      //oled.setRotation(2); // bildschirm upside-down
      Serial.println("SSD1306 OLED-Display okay.");
      displayzeile[1] = "OLED-Display okay.";
      displayText();
   }

   if (!LoRa.begin(868E6)) {
      Serial.println("Starting LoRa failed!");
      displayzeile[2] = "Starting LoRa failed!";
      displayText();
      delay(2000);
      return;
   } else {
      LoRa.setSpreadingFactor(6);  // Supported values are between `6` and `12`. defaults to `7`
      // If a spreading factor of `6` is set, implicit header mode must be used to transmit and receive packets.
      LoRa.setCodingRate4(5); // default `5`, Supported values `5` - `8`, correspond to coding rates of `4/5` and `4/8
      LoRa.disableCrc();  // by default a CRC is not used
      LoRa.setPreambleLength(8); // default 8, Supported values are between `6` and `65535`
      LoRa.setSyncWord(0x12); // byte value to use as the sync word, defaults to `0x12`
      LoRa.setSignalBandwidth(125E3);  // signal bandwidth in Hz, defaults to `125E3`
      // Supported values are `7.8E3`, `10.4E3`, `15.6E3`, `20.8E3`, `31.25E3`, `41.7E3`, `62.5E3`, `125E3`, `250E3`, and `500E3`.
      Serial.println("LoRa was started.");
      displayzeile[2] = "LoRa was started.";
      displayText();
   }

   // Format for setting a serial port is: Serial1.begin(baud-rate, protocol, RX pin, TX pin);
   // Serial RX-Pin set to RX_gpio (TX but not used, therefore set to -1)
   // Telegramm Protokoll nach SML 1.04: Baudrate: 9600 Bit/s Byte Format: (8,N,1)
    pinMode(RX_gpio, INPUT);
   // pinMode(RX_gpio, INPUT_PULLDOWN);
   // pinMode(RX_gpio, INPUT_PULLUP);
   Serial1.begin(9600, SERIAL_8N1, RX_gpio, -1);
   Serial1.setTimeout(3000);
   Serial.println("Serialport 1 was started.");
   Serial.println("Setup done.");
}

// --------------- LOOP ---------------
void loop() {

  ReadSerialData();

   // Taster abfragen, wenn Taster gedrückt wurde, Display ein oder ausschalten
   statusTaster = digitalRead(display_taster_gpio); 
   if (statusTaster == !statusTasterLetzter) { // Wenn aktueller Tasterstatus anders ist als der letzte Tasterstatus
      if (statusTaster == LOW) { // Wenn Taster gedrückt
         if (displayPowerSave == true){ // Display an- bzw. ausschalten
            displayPowerSave = false;
            oled.ssd1306_command(SSD1306_DISPLAYON);
            Serial.println("Display an.");

         } else {
            displayPowerSave = true;
            oled.ssd1306_command(SSD1306_DISPLAYOFF);
            Serial.println("Display aus.");
         } 
      }            
   }
   statusTasterLetzter = statusTaster; // merken des letzten Tasterstatus

   Data_Age = (millis() - _lastmillis)/1000;
   displayzeile[4] = "Data Age: " + std::to_string(Data_Age) + " s";

   if ((millis() - _lastmillisdisplay) > (1000)) { // display jede sekunde aktualisieren
      displayText();
      _lastmillisdisplay = millis();
      Serial.print("Data Age: ");
      Serial.print(Data_Age);
      Serial.println(" s");

   }
   if (Data_Age > 30){ // wenn länger als 30 s keine daten --> display blinken
      displayblinken();
   } else{
      if (display_on = false){ // damit, wenn blink-phase endet, display an ist
         oled.ssd1306_command(SSD1306_DISPLAYON);
         display_on = true;
      }            
   }
}