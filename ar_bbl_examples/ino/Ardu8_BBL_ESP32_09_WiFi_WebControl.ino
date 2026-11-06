#include "WiFi.h"
#include "ESPAsyncWebServer.h"
String page = "";
const char* ssid = "Kiber-ESP32";
const char* password = "12345678";
AsyncWebServer asyncServer(80);
void setup() {
  	board_init();
  	Serial.begin(115200);
  	page = String("<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>Кибернетический</title><style>body{font-family:sans-serif;text-align:center;background:#f4f4f4}a{display:inline-block;margin:6px;padding:14px 22px;border-radius:10px;color:#fff;text-decoration:none;font-size:18px}.r{background:#d33}.g{background:#2a2}.b{background:#36c}.k{background:#555}.y{background:#c80}</style></head><body><h2>Робототехнический контроллер «Кибернетический»</h2><p>ESP32: светодиоды и пищалка</p><p><a class=\"y\" href=\"/led_on\">LED1 вкл</a><a class=\"k\" href=\"/led_off\">LED1 выкл</a></p><p><a class=\"r\" href=\"/red\">Красный</a><a class=\"g\" href=\"/green\">Зелёный</a><a class=\"b\" href=\"/blue\">Синий</a><a class=\"k\" href=\"/rgb_off\">RGB выкл</a></p><p><a class=\"y\" href=\"/beep\">Бип</a></p></body></html>");
  	WiFi.softAP("Kiber-ESP32", "12345678");
  	asyncServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", (String)page);});
  	asyncServer.on("/led_on", HTTP_GET, [](AsyncWebServerRequest *request) {
    	digitalWrite(27, LOW);
    request->send(200, "text/html", (String)page);});
  	asyncServer.on("/led_off", HTTP_GET, [](AsyncWebServerRequest *request) {
    	digitalWrite(27, HIGH);
    request->send(200, "text/html", (String)page);});
  	asyncServer.on("/red", HTTP_GET, [](AsyncWebServerRequest *request) {
    	digitalWrite(15, LOW);
    	digitalWrite(2, HIGH);
    	digitalWrite(18, HIGH);
    request->send(200, "text/html", (String)page);});
  	asyncServer.on("/green", HTTP_GET, [](AsyncWebServerRequest *request) {
    	digitalWrite(15, HIGH);
    	digitalWrite(2, LOW);
    	digitalWrite(18, HIGH);
    request->send(200, "text/html", (String)page);});
  	asyncServer.on("/blue", HTTP_GET, [](AsyncWebServerRequest *request) {
    	digitalWrite(15, HIGH);
    	digitalWrite(2, HIGH);
    	digitalWrite(18, LOW);
    request->send(200, "text/html", (String)page);});
  	asyncServer.on("/rgb_off", HTTP_GET, [](AsyncWebServerRequest *request) {
    	digitalWrite(15, HIGH);
    	digitalWrite(2, HIGH);
    	digitalWrite(18, HIGH);
    request->send(200, "text/html", (String)page);});
  	asyncServer.on("/beep", HTTP_GET, [](AsyncWebServerRequest *request) {
    	tone(19, 1000, 150);
    request->send(200, "text/html", (String)page);});
  	asyncServer.begin();
  	Serial.println("Подключитесь к Wi-Fi Kiber-ESP32 (пароль 12345678) и откройте http://192.168.4.1");
}
void loop() {
  	delay(1000);
}

void board_init() {
  	pinMode(34, INPUT);
  	pinMode(35, INPUT);
  	pinMode(25, INPUT);
  	pinMode(26, INPUT);
  	pinMode(27, OUTPUT);
  	pinMode(14, OUTPUT);
  	pinMode(12, OUTPUT);
  	pinMode(13, OUTPUT);
  	pinMode(15, OUTPUT);
  	pinMode(2, OUTPUT);
  	pinMode(18, OUTPUT);
  	pinMode(19, OUTPUT);
  	leds_off();
}

void leds_off() {
  	digitalWrite(27, HIGH);
  	digitalWrite(14, HIGH);
  	digitalWrite(12, HIGH);
  	digitalWrite(13, HIGH);
  	digitalWrite(15, HIGH);
  	digitalWrite(2, HIGH);
  	digitalWrite(18, HIGH);
}
