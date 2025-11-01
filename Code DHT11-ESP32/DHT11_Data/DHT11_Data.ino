#include "DHT11.h"

#define DHTPIN 4

float temp, humi;
DHT11 dht11(DHTPIN);

void setup() {
  Serial.begin(115200);
  Serial.println(F("DHT_Data"));
  dht11.setDelay(500);

}
void loop() {

  temp= dht11.readTemperature();
  humi= dht11.readHumidity();
   if (isnan(humi) || isnan(temp)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.print("Temperature (ºC): ");
  Serial.print(temp);
  Serial.print("\t|| Humidity(%)");
  Serial.println(humi);
}
