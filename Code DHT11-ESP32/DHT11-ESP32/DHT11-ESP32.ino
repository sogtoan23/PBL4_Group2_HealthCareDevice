#include "DHT11.h"
#include <WiFi.h>

#define DHTPIN 4
// DHT11
float temp, humi;
DHT11 dht11(DHTPIN);
// Wifi
const char *ssid = "Tigon";          
const char *password = "Huongloc";
// Thinkspeak
const char *host = "api.thingspeak.com";       
const int httpPort = 80;                     //443_https;80_http, https mã hóa tín hiệu để gửi   
const String channelID = "3117024";             
const String writeApiKey = "VZIZCAUFEW7NZG2Q";  
const String readApiKey = "3JNVXFM3RIKLE7IK"; 

void setup() {
  Serial.begin(115200);
  Serial.println(F("DHT_Data"));
  dht11.setDelay(500);
  Serial.println();
  Serial.println("******************************************************");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

}
void readResponse(NetworkClient *client) {
  unsigned long timeout = millis();
  while (client->available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client->stop();
      return;
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  while (client->available()) {
    String line = client->readStringUntil('\r');
    Serial.print(line);
  }

  Serial.printf("\nClosing connection\n\n");
}
void loop() {

  temp= dht11.readTemperature();
  humi= dht11.readHumidity();
   if (isnan(humi) || isnan(temp)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  NetworkClient client;
  String footer = String(" HTTP/1.1\r\n") + "Host: " + String(host) + "\r\n" + "Connection: close\r\n\r\n";

  // WRITE --------------------------------------------------------------------------------------------
  if (!client.connect(host, httpPort)) {
    return;
  }

  String req = "GET /update?api_key=" + writeApiKey +"&field1=" + String(temp, 2) + "&field2=" + String(humi, 2) + footer;

  client.print(req);
  Serial.println("\n--- HTTPS Request ---");
  Serial.println(req);

  readResponse(&client);


  // Serial.print("Temperature (ºC): ");
  // Serial.print(temp);
  // Serial.print("\t|| Humidity(%)");
  // Serial.println(humi);

  delay(20000);
}
