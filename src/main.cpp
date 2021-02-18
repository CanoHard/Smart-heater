#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include "config.h"
#include "main.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <OneButton.h>
#include <Chrono.h>
#include <CanoMqtt.h>

//Made by Pablo Cano v1.0
Adafruit_SSD1306 display(128, 64, &Wire, -1);

OneWire oneWire(TEMPPIN);
DallasTemperature temp(&oneWire);

CanoMqtt mqttClient(WIFI_SSID, WIFI_PASSWORD, MQTT_HOST, MQTT_USER, MQTT_PASSWORD, NAME, STATUS_TOPIC, "offline");

OneButton buttonplus(BUTTONIPIN, true);
OneButton buttonminus(BUTTONDPIN, true);
OneButton buttononoff(BUTTONOPIN, true);

Chrono sendtemp;
Chrono checktemp;

bool onoff = false;
bool heater = false;
bool away = false;
float current_temp = 20.3;
float set_temp = 19;
float set_atemp = 18;

// 'heat n', 16x8px
const unsigned char myBitmap[] PROGMEM = {
    0x20, 0x82, 0x41, 0x04, 0x82, 0x08, 0x41, 0x04, 0x20, 0x82, 0x10, 0x41, 0x20, 0x82, 0x41, 0x04};

// 'encasa', 8x8px
const unsigned char casa[] PROGMEM = {
    0x18, 0x3c, 0x7e, 0xff, 0x7e, 0x7e, 0x7e, 0x7e};

void IncreaseClick()
{
  Serial.println("Button increase temp clicked");
  if (onoff)
  {
    if (away)
    {
      changetemp(set_atemp + 0.5, true);
    }
    else
    {
      changetemp(set_temp + 0.5, true);
    }
  }
}
void DecreaseClick()
{
  Serial.println("Button decrease temp clicked");
  if (onoff)
  {
    if (away)
    {
      changetemp(set_atemp - 0.5, true);
    }
    else
    {
      changetemp(set_temp - 0.5, true);
    }
  }
}
void PowerClick()
{
  Serial.println("Button power clicked");
  if (onoff)
  {
    if (away)
    {
      setAway(false, true);
    }
    else
    {
      setAway(true, true);
    }
  }
  else
  {
    Setonoff(true, true);
  }
}
void HoldIncreaseClick()
{
  if (onoff)
  {
    if (away)
    {
      changetemp(set_atemp + 0.5, true);
    }
    else
    {
      changetemp(set_temp + 0.5, true);
    }
  }
}
void HoldDecreaseClick()
{
  if (onoff)
  {
    if (away)
    {
      changetemp(set_atemp - 0.5, true);
    }
    else
    {
      changetemp(set_temp - 0.5, true);
    }
  }
}

void HoldPower()
{
  if (onoff)
  {
    Setonoff(false, true);
  }
  else
  {
    Setonoff(true, true);
  }
}

//Network

void onMqttConnect()
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");

  mqttClient.Publish(STATUS_TOPIC, 0, true, "online");
  sendtemp.restart();
  sendData();
  mqttClient.Subscribe(mode_command_topic, 0);
  mqttClient.Subscribe(temperature_command_topic, 0);
  mqttClient.Subscribe(away_mode_command_topic, 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  sendtemp.restart();
  sendtemp.stop();
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);

  String topicStr = topic;
  String msj;
  msj = "";
  for (int i = 0; i < len; i++)
  { //Le el msj
    msj = msj + (char)payload[i];
  }

  if (topicStr == mode_command_topic)
  {

    if (msj == "heat")
    {
      Setonoff(true, false);
    }
    else
    {
      Setonoff(false, false);
    }
  }
  if (topicStr == temperature_command_topic)
  {
    changetemp(msj.toFloat(), false);
  }
  if (topicStr == away_mode_command_topic)
  {
    if (msj == "ON")
    {
      setAway(true, false);
    }
    else
    {
      setAway(false, false);
    }
  }
}

void onOtaevent(CanoMqtt::OtaEvent e, int p){
  switch (e)
  {
  case CanoMqtt::OTA_ONSTART:
    display.ssd1306_command(SSD1306_DISPLAYON);
    break;

  case CanoMqtt::OTA_ONPROGRESS:
    display.clearDisplay();
    display.setTextSize(1);  // Normal 1:1 pixel scale
    display.setCursor(0, 0); // Start at top-left corner
    display.println("     Actualizando");
    display.setTextSize(3);   // Normal 1:1 pixel scale
    display.setCursor(0, 20); // Start at top-left corner
    display.print("  ");
    display.print(p);
    display.print("%");
    display.display();
    break;

  case CanoMqtt::OTA_ONEND:
    display.clearDisplay();
    display.setTextSize(2);   // Normal 1:1 pixel scale
    display.setCursor(0, 16); // Start at top-left corner
    display.print("Instalando");
    display.display();  
    break;

  case CanoMqtt::OTA_ONERROR:
    display.clearDisplay();
    display.setTextSize(2);   // Normal 1:1 pixel scale
    display.setCursor(0, 16); // Start at top-left corner
    display.print("ERROR");
    display.display();  
    break;         
  
  default:
    break;
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  EEPROM.begin(512);
  get_data();
  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, HIGH);
  sendtemp.restart();
  sendtemp.stop();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3D (fr the 128x64)
  display.display();
  Serial.println("Booting");

  WiFi.mode(WIFI_STA);
  WiFi.hostname(NAME);

  mqttClient.SetOnMqttConnect(onMqttConnect);
  mqttClient.SetOnMqttMessage(onMqttMessage);
  mqttClient.SetOnOtaEvent(onOtaevent);
  mqttClient.Init();


  temp.begin();
  temp.setWaitForConversion(false); // makes it async

  buttonminus.attachClick(DecreaseClick);
  buttonplus.attachClick(IncreaseClick);
  buttononoff.attachClick(PowerClick);
  buttonminus.attachDuringLongPress(HoldDecreaseClick);
  buttonplus.attachDuringLongPress(HoldIncreaseClick);
  buttononoff.attachLongPressStart(HoldPower);

  bool temp = away;
  away = false;
  if (set_temp > MAXTEMP || set_temp < MINTEMP)
  {
    changetemp(MINTEMP, true);
  }
  away = true;
  if (set_atemp > MAXTEMP || set_atemp < MINTEMP)
  {
    changetemp(MINTEMP, true);
  }
  away = temp;
}

void onoffheater(bool on)
{
  if (on != heater)
  {
    if (on)
    {
      digitalWrite(RELAYPIN, LOW);
      heater = true;
      mqttClient.Publish(action_topic, 0, false, "heating");
    }
    else
    {
      digitalWrite(RELAYPIN, HIGH);
      heater = false;
      mqttClient.Publish(action_topic, 0, false, "idle");
    }
  }
}

void Setonoff(bool on, bool sync)
{
  if (on != onoff)
  {
    if (on)
    {
      onoff = true;
      checktemp.restart();
      mqttClient.Publish(mode_state_topic, 0, true, "heat");
      if (SYNC_ENABLE == 1 && sync)
      {
        mqttClient.Publish(SYNC_mode_command_topic, 0, false, "heat");
      }
    }
    else
    {
      onoffheater(false);
      onoff = false;
      checktemp.restart();
      checktemp.stop();
      mqttClient.Publish(action_topic, 0, true, "off");
      mqttClient.Publish(mode_state_topic, 0, true, "off");
      if (SYNC_ENABLE == 1 && sync)
      {
        mqttClient.Publish(SYNC_mode_command_topic, 0, false, "off");
      }
    }
    EEPROM.write(0, onoff);
    EEPROM.commit();
  }
}

void changetemp(float t, bool sync)
{
  if (away)
  {
    if (t != set_atemp && (t <= MAXTEMP && t >= MINTEMP))
    {
      set_atemp = t;
      char temperature[10];
      dtostrf(set_atemp, 4, 1, temperature);
      mqttClient.Publish(temperature_state_topic, 0, true, temperature);
      EEPROM.write(1, away);
      int atempb = set_atemp * 10;
      EEPROM.write(2, highByte(atempb));
      EEPROM.write(3, lowByte(atempb));
      EEPROM.commit();
      if (SYNC_ENABLE == 1 && sync)
      {
        mqttClient.Publish(SYNC_temperature_command_topic, 0, false, temperature);
      }
    }
  }
  else
  {
    if (t != set_temp && (t <= MAXTEMP && t >= MINTEMP))
    {
      set_temp = t;
      char temperature[10];
      dtostrf(set_temp, 4, 1, temperature);
      mqttClient.Publish(temperature_state_topic, 0, true, temperature);
      int tempb = set_temp * 10;
      EEPROM.write(4, highByte(tempb));
      EEPROM.write(5, lowByte(tempb));
      EEPROM.commit();
      if (SYNC_ENABLE == 1 && sync)
      {
        mqttClient.Publish(SYNC_temperature_command_topic, 0, false, temperature);
      }
    }
  }
}

void setAway(bool on, bool sync)
{
  if (on != away)
  {
    away = on;
    if (away)
    {
      char temperature[10];
      dtostrf(set_atemp, 4, 1, temperature);
      mqttClient.Publish(temperature_state_topic, 0, true, temperature);
      mqttClient.Publish(away_mode_state_topic, 0, true, "ON");
      if (SYNC_ENABLE == 1 && sync)
      {
        mqttClient.Publish(SYNC_away_mode_command_topic, 0, false, "ON");
      }
    }
    else
    {
      char temperature[10];
      dtostrf(set_temp, 4, 1, temperature);
      mqttClient.Publish(temperature_state_topic, 0, true, temperature);
      mqttClient.Publish(away_mode_state_topic, 0, true, "OFF");
      if (SYNC_ENABLE == 1 && sync)
      {
        mqttClient.Publish(SYNC_away_mode_command_topic, 0, false, "OFF");
      }
    }
    EEPROM.write(1, away);
    EEPROM.commit();
  }
}

void handleheater()
{
  if (current_temp > -20)
  {
    if (checktemp.hasPassed(TEMPCHECK))
    {
      float set;
      if (away)
      {
        set = set_atemp;
      }
      else
      {
        set = set_temp;
      }
      if (current_temp - set <= -1 * TEMPOFFSET)
      {
        onoffheater(true);
      }
      else if (current_temp >= set)
      {
        onoffheater(false);
      }
      checktemp.restart();
    }
  }
  else
  {
    onoffheater(false);
  }
}

void sendData() //Sends all the data to the mqtt broker
{
  if (away)
  {
    char temperature[10];
    dtostrf(set_atemp, 4, 1, temperature);
    mqttClient.Publish(temperature_state_topic, 0, true, temperature);
    mqttClient.Publish(away_mode_state_topic, 0, true, "ON");
  }
  else
  {
    char temperature[10];
    dtostrf(set_temp, 4, 1, temperature);
    mqttClient.Publish(temperature_state_topic, 0, true, temperature);
    mqttClient.Publish(away_mode_state_topic, 0, true, "OFF");
  }
  if (onoff)
  {
    mqttClient.Publish(mode_state_topic, 0, true, "heat");
    if (heater)
    {
      mqttClient.Publish(action_topic, 0, false, "heating");
    }
    else
    {
      mqttClient.Publish(action_topic, 0, false, "idle");
    }
  }
  else
  {
    mqttClient.Publish(action_topic, 0, true, "off");
    mqttClient.Publish(mode_state_topic, 0, true, "off");
  }
  char temperature[10];
  dtostrf(current_temp, 4, 1, temperature);
  mqttClient.Publish(current_temperature_topic, 0, false, temperature);
}

void loop()
{

  temp.requestTemperatures();
  current_temp = temp.getTempCByIndex(0);
  handleheater();
  refresholed();
  mqttClient.NetworkLoop();
  if (sendtemp.hasPassed(UPDATETEMPI))
  {
    char temperature[10];
    dtostrf(current_temp, 4, 1, temperature);
    mqttClient.Publish(current_temperature_topic, 0, false, temperature);

    sendtemp.restart();
  }
  buttonminus.tick();
  buttononoff.tick();
  buttonplus.tick();
}

void get_data()
{
  onoff = EEPROM.read(0);
  away = EEPROM.read(1);
  byte high1 = EEPROM.read(4);
  byte lw1 = EEPROM.read(5);
  set_temp = ((word(high1, lw1)) / 10.0);
  byte high2 = EEPROM.read(2);
  byte lw2 = EEPROM.read(3);
  set_atemp = ((word(high2, lw2)) / 10.0);
  if (!onoff)
  {
    checktemp.restart();
    checktemp.stop();
  }
  else
  {
    checktemp.restart();
  }
}

void refresholed()
{
  int bars;
  if (onoff)
  {
    display.ssd1306_command(SSD1306_DISPLAYON);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(1, 4);
    display.setTextSize(1);
    char temperatureac[10];
    dtostrf(current_temp, 2, 1, temperatureac);
    display.print(temperatureac);
    display.print(" ");
    display.drawCircle(28, 4, 1, WHITE);
    display.print("C");
    if (heater)
    {
      display.drawBitmap(47, 4, myBitmap, 16, 8, WHITE);
    }
    if (!away)
    {
      display.drawBitmap(75, 4, casa, 8, 8, WHITE);
    }
    if (mqttClient.IsMqttConnected())
    {
      display.setCursor(93, 5);
      display.println("#");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      int RSSI = WiFi.RSSI();
      if (RSSI >= -55)
      {
        bars = 5;
      }
      else if (RSSI >= -65)
      {
        bars = 4;
      }
      else if (RSSI >= -70)
      {
        bars = 3;
      }
      else if (RSSI >= -78)
      {
        bars = 2;
      }
      else if (RSSI >= -90)
      {
        bars = 1;
      }
      else
      {
        bars = 0;
      }
    }
    else
    {
      bars = 0;
    }

    for (int b = 0; b <= bars; b++)
    { //Dibuja barras
      int i = 0;
      int u = 0;
      if (b != 5)
      {
        i = (b * 2);
        u = 12 - (b * 2) + 1;
      }
      else
      {
        i = b * 2;
        u = 12 - (b * 2) + 1;
      }

      display.fillRect(105 + (b * 4), u, 3, i, WHITE);
    }

    display.setCursor(0, 25); //En el pixel 8 empieza la otra zona
    if (away)
    {
      int tempin = set_atemp;
      display.setTextSize(4);
      display.print(tempin);
      display.fillCircle(53, 50, 3, WHITE);
      display.print(" ");
      display.setTextSize(3);
      display.setCursor(65, 29); //En el pixel 8 empieza la otra zona
      display.print((int)((set_atemp - tempin) * 10));
      display.setTextSize(4);
      display.drawCircle(94, 27, 4, WHITE);
      display.drawCircle(94, 27, 3, WHITE);
      display.setCursor(100, 26); //En el pixel 8 empieza la otra zona
      display.print("C");
    }
    else
    {

      int tempin = set_temp;
      display.setTextSize(4);
      display.print(tempin);
      display.fillCircle(53, 50, 3, WHITE);
      display.print(" ");
      display.setTextSize(3);
      display.setCursor(65, 29); //En el pixel 8 empieza la otra zona
      display.print((int)((set_temp - tempin) * 10));
      display.setTextSize(4);
      display.drawCircle(94, 27, 4, WHITE);
      display.drawCircle(94, 27, 3, WHITE);
      display.setCursor(100, 26); //En el pixel 8 empieza la otra zona
      display.print("C");
    }
    display.display();
  }
  else
  {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}
