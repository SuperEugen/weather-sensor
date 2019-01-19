/*
 * weather-sensor
 * 
 * An outside temperature, humidity and pressure sensor reports data to
 * a MQTT broker.
 * 
 * created by Ingo Hoffmann,    6. January 2019,  initial tests
 * 
 * Components:
 * BME280:  connects via I2C
 * 
 * Power:
 * powered by a LiFePo4 accu with 1400mAh at 3.2V
 * a 1000 µF capacitor stabilizes the voltage especially during boot
 * voltage meassured via ADC input with a voltage divider between GND and 
 * 
 * Libraries:
 * WiFi.h             
 * Wire.h             Arduino I2C library
 * Adafruit_BME280.h  Adafruit library for the Bosch sensor BME280
 * esp_deep_sleep.h   Espressif library 
 * 
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "esp_deep_sleep.h"   // for ESP deep sleep
#include <esp_wifi.h>         // for WiFi power down
#include <PubSubClient.h>

// PIN definitions
#define ADCInputPin 36

// mqtt topics
const char* mqttTempTopic ="weather-sensor/temp";
const char* mqttHumidityTopic ="weather-sensor/humidity";
const char* mqttPressureTopic ="weather-sensor/pressure";
const char* mqttVoltageTopic ="weather-sensor/voltage";

// specific constants for this weather sensor
const int updateIntervalInMinutes = 15;
const int sensorAltitudeInMeters =  55; // 40m at street level + 15m within building

// MQTT value is a string
char* mqttValue;
char* mqttClientID = "S-I-Wettersensor";

// global object variables
WiFiClient wifiClient; // wifi client object
PubSubClient mqttClient(wifiClient);    
Adafruit_BME280 bme;

// include either a secrets.h file or put your secrets her
// const char* ssid          = "your_ssid";
// const char* wifiPassword  = "your_WLAN_password";
// const char* mqttUser      = "your_MQTT_user";
// const char* mqttPassword  = "your_MQTT_password";
#include "secrets.h"

void setup() {
  int wifiRetries;
  
  Serial.begin(115200);
  WiFi.begin(ssid, wifiPassword);

  while (WiFi.status() != WL_CONNECTED ) {Serial.print(".");delay(500); }
  Serial.println();

  Wire.begin(21, 22); // (sda, scl) 
  if (!bme.begin()) {
    Serial.println("no sensor");
  }
  else
  {
    while (isnan(bme.readPressure())) {};
  }

  float temperature = bme.readTemperature();
  float humidity    = bme.readHumidity();
  float pressure    = BME2SealevelhPA(bme.readPressure());
  float vBat        = analogRead(ADCInputPin) / 4095 * 3.3;
  
  mqttClient.setServer(mqttBroker, mqttPort);

  wifiRetries = 0;
  
  while (!mqttClient.connected() && wifiRetries < 5) {
    // Attempt to connect
    if (mqttClient.connect(mqttClientID, mqttUser, mqttPassword)) {
      // MQTT messages with retain flag set
      mqttClient.publish(mqttTempTopic, String(temperature).c_str(), true);
      mqttClient.publish(mqttHumidityTopic, String(humidity).c_str(), true);
      mqttClient.publish(mqttPressureTopic, String(pressure).c_str(), true);
      mqttClient.publish(mqttVoltageTopic, String(vBat).c_str(), true);
    } else { 
      Serial.print("failed, rc="); 
      Serial.print(mqttClient.state()); 
      Serial.println(" try again in 5 seconds"); 
      // Wait 5 seconds before retrying
      wifiRetries += 1;
      delay(5000); 
    } 
  } 

/*
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity:    ");
  Serial.println(humidity);
  Serial.print("Pressure:    ");
  Serial.println(pressure);
  Serial.print("Voltage:     ");
  Serial.println(vBat);
*/
  delay(50);

  BME280_Sleep();
  esp_wifi_stop();
  esp_deep_sleep_enable_timer_wakeup(updateIntervalInMinutes * 60 * 1000000); // timer value in µ-seconds
  esp_deep_sleep_start();
}

void loop() {
  // this loop will not be reached
}

void BME280_Sleep() {
  Wire.beginTransmission(0x77);
  Wire.write((uint8_t)BME280_REGISTER_CONTROL);
  Wire.write((uint8_t)0b00);
  Wire.endTransmission();
}

// Keisan Calculation for Sea Level Pressure from current BME280 readings
// http://keisan.casio.com/exec/system/1224575267
double BME2SealevelhPA(double meassuredValue) {
  float adjustedAltitude = sensorAltitudeInMeters * 0.0065; // altitude multiplier
  
  return meassuredValue / 100.0F * (pow ((1 - (adjustedAltitude / (bme.readTemperature() + adjustedAltitude + 273.15))), -5.257)) ;
}
