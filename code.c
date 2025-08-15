/******************************************************
 * Fire Safety IoT — ESP8266 (NodeMCU)
 * Sensors: DHT22 (temp), MQ-2 (smoke), Vibration switch
 * Outputs: I2C LCD 16x2, Relay (FM-200), Wi-Fi HTTP POST
 * Optional: GPS (TinyGPS++) on SoftwareSerial
 ******************************************************/

// ------------ Libraries ------------
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>   // ok for https (setInsecure for demo)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  // install: "LiquidCrystal I2C" by Frank de Brabander
#include <DHT.h>                // install: "DHT sensor library" by Adafruit
#include <TinyGPSPlus.h>        // install: "TinyGPSPlus"
#include <SoftwareSerial.h>
#include <ArduinoJson.h>        // install: "ArduinoJson" by Benoit Blanchon

// ------------ User Config ------------
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASS";
const char* POST_URL      = "https://example.com/api/fire-telemetry"; // replace with your cloud endpoint

// Trigger thresholds (tune for your hardware/environment)
float  TEMP_HIGH_C         = 55.0;  // °C - typical fire indicator (adjust)
int    SMOKE_RAW_TRIGGER   = 600;   // 0–1023 (MQ-2 via A0) -> tune via calibration
int    VIBRATION_DEBOUNCE  = 50;    // ms debounce for vibration pulse

// Safety timings
unsigned long ALARM_LATCH_MS = 30000; // keep relay ON for at least 30s once triggered
unsigned long POST_INTERVAL   = 5000; // ms between cloud updates
unsigned long LCD_PAGE_MS     = 1500; // ms per LCD screen

// ------------ Pins (NodeMCU labels) ------------
#define PIN_DHT       D5   // DHT22 data
#define PIN_VIBE      D6   // vibration digital out (active HIGH)
#define PIN_MQ2       A0   // analog smoke (0..1023)
#define PIN_RELAY     D0   // relay IN (active LOW/HIGH depends on module)
#define PIN_BTN_RESET D3   // manual reset button to clear alarm (GND to trigger)
#define I2C_SDA       D2   // LCD SDA
#define I2C_SCL       D1   // LCD SCL
#define GPS_RX        D7   // ESP8266 RX <= GPS TX
#define GPS_TX        D8   // ESP8266 TX => GPS RX (often unused)

// ------------ Objects ------------
LiquidCrystal_I2C lcd(0x27, 16, 2); // change 0x27 to 0x3F if needed
DHT dht(PIN_DHT, DHT22);
SoftwareSerial gpsSS(GPS_RX, GPS_TX);
TinyGPSPlus gps;

// ------------ State ------------
bool alarmActive = false;
unsigned long alarmSince = 0;
unsigned long lastPost = 0;
unsigned long lastLCD = 0;
int lcdPage = 0;

// vibration debounce
volatile bool vibeFlag = false;
unsigned long lastVibeMs = 0;

// ------------ Helpers ------------
void IRAM_ATTR onVibeChange() {
  unsigned long now = millis();
  if (now - lastVibeMs > (unsigned long)VIBRATION_DEBOUNCE) {
    vibeFlag = digitalRead(PIN_VIBE);
    lastVibeMs = now;
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(250);
  }
}

String fix(float v, int digits=6) {
  char b[16];
  dtostrf(v, 0, digits, b);
  return String(b);
}

void postToCloud(float tC, int smokeRaw, bool vib, bool alarm,
                 double lat, double lng, int sats, float hdop) {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure(); // for demo; use client.setFingerprint or proper cert in production
  HTTPClient http;

  if (!http.begin(client, POST_URL)) return;

  StaticJsonDocument<512> doc;
  doc["device"]   = "ESP8266-FireNode-01";
  doc["temp_c"]   = tC;
  doc["smoke"]    = smokeRaw;       // ADC 0..1023
  doc["vibration"]= vib;
  doc["alarm"]    = alarm;
  doc["rssi"]     = WiFi.RSSI();
  doc["ip"]       = WiFi.localIP().toString();

  JsonObject gpsObj = doc.createNestedObject("gps");
  gpsObj["lat"]  = lat;
  gpsObj["lng"]  = lng;
  gpsObj["sats"] = sats;
  gpsObj["hdop"] = hdop;

  String payload;
  serializeJson(doc, payload);

  http.addHeader("Content-Type", "application/json");
  int code = http.POST(payload);
  // Optionally read response:
  // String resp = http.getString();
  http.end();
}

void setRelay(bool on) {
  // Adjust logic if your relay is active LOW
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
}

void showLCD(const char* l1, const char* l2) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(l1);
  lcd.setCursor(0,1); lcd.print(l2);
}

// ------------ Setup ------------
void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false);

  pinMode(PIN_VIBE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_VIBE), onVibeChange, CHANGE);

  pinMode(PIN_BTN_RESET, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("FireNode Boot...");
  
  dht.begin();
  gpsSS.begin(9600);

  connectWiFi();
  delay(500);
}

// ------------ Loop ------------
void loop() {
  // --- Read sensors ---
  float tC = dht.readTemperature();        // °C
  if (isnan(tC)) tC = -127.0;

  int smokeRaw = analogRead(PIN_MQ2);      // 0..1023
  bool vib = vibeFlag;                     // latched on debounce ISR
  vibeFlag = false;

  // --- Read GPS (non-blocking) ---
  while (gpsSS.available()) gps.encode(gpsSS.read());
  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lng = gps.location.isValid() ? gps.location.lng() : 0.0;
  int sats   = gps.satellites.isValid() ? gps.satellites.value() : 0;
  float hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;

  // --- Determine fire condition ---
  bool tempHigh  = (tC >= TEMP_HIGH_C && tC < 120.0); // sanity cap
  bool smokeHigh = (smokeRaw >= SMOKE_RAW_TRIGGER);
  bool fireDetected = (smokeHigh && (tempHigh || vib)); // AND + (OR) logic

  // --- Alarm latch ---
  unsigned long now = millis();
  if (!alarmActive && fireDetected) {
    alarmActive = true;
    alarmSince = now;
    setRelay(true);             // trigger FM-200 via relay
  }

  if (alarmActive) {
    // keep latched for minimum duration; allow manual reset after latch time
    if (digitalRead(PIN_BTN_RESET) == LOW && (now - alarmSince) > ALARM_LATCH_MS) {
      alarmActive = false;
      setRelay(false);
    }
  } else {
    setRelay(false);
  }

  // --- LCD pages ---
  if (now - lastLCD > LCD_PAGE_MS) {
    lastLCD = now;
    lcdPage = (lcdPage + 1) % 4;

    if (lcdPage == 0) {
      char l1[17], l2[17];
      snprintf(l1, 17, "T:%.1fC Smoke:%4d", tC, smokeRaw);
      snprintf(l2, 17, "Vib:%s WIFI:%s", vib ? "Y" : "N",
               WiFi.status()==WL_CONNECTED ? "OK" : "—");
      showLCD(l1, l2);
    } else if (lcdPage == 1) {
      char l1[17], l2[17];
      snprintf(l1, 17, "GPS:%s Sats:%2d", gps.location.isValid()?"OK":"—", sats);
      snprintf(l2, 17, "Lat:%.3f", lat);
      showLCD(l1, l2);
    } else if (lcdPage == 2) {
      char l1[17], l2[17];
      snprintf(l1, 17, "Lng:%.3f", lng);
      snprintf(l2, 17, "HDOP:%.1f", hdop);
      showLCD(l1, l2);
    } else {
      showLCD(alarmActive ? "!! FIRE ALARM !!" : "Status: NORMAL",
              "Hold BTN=Reset");
    }
  }

  // --- Cloud post ---
  if (now - lastPost > POST_INTERVAL) {
    lastPost = now;
    postToCloud(tC, smokeRaw, vib, alarmActive, lat, lng, sats, hdop);
  }

  delay(10);
}

/*************** Wiring (NodeMCU) *****************
 * DHT22    -> D5 (data), 3V3, GND (+4.7k pullup)
 * Vibration switch/module -> D6 (use module with digital out), 3V3, GND
 * MQ-2 (analog) -> A0 (use divider so output <= 1.0V to protect ESP8266)
 * Relay IN -> D0   (Vcc=5V or 3V3 per module, GND shared)
 * I2C LCD -> SDA=D2, SCL=D1 (Vcc=5V if module supports; share GND)
 * GPS     -> GPS TX -> D7 ; GPS RX -> D8 (optional), Vcc (per module), GND
 * Reset Button -> D3 to GND (INPUT_PULLUP enabled)
 **************************************************/
