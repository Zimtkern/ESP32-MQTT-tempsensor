#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

#define wifi_ssid "ssid"         //wifi ssid
#define wifi_password "password"     //wifi password

#define mqtt_server "ip"  // server name or IP
#define mqtt_user "user"      // username
#define mqtt_password "password"   // password

#define temperature_topic "esp32/dht/temperature"
#define humidity_topic "esp32/dht/humidity"
#define voltage_topic "esp32/battery/voltage"
#define chargingLevel_topic "esp32/battery/chargingLevel"
#define debug_topic "debug"

bool debug = true;

#define DHTPIN 15
#define DHTTYPE DHT22
#define sleepPIN 4

int counter = 0;
int connectCounter = 0;

DHT dht(DHTPIN, DHTTYPE);     
WiFiClient espClient;
PubSubClient client(espClient);

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(115200);  
  pinMode(sleepPIN, OUTPUT);
  digitalWrite(sleepPIN, LOW);   
  setup_wifi();
    client.setServer(mqtt_server, 1883);
  if (!client.connected()) {
    reconnect();
  }
  dht.begin();

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    int r1 = 20; //kΩ
    int r2 = 68; //kΩ
    float vMax = 4.2;
    float vMin = 2.8;
    int analogValue = analogRead(36);
    float analogMax = 3.3;

    float vIn = floatMap(analogValue, 0, 4095, 0, 3.3);
    float analogMin = (vMin*r2)/(r1+r2);    
    float voltage = vIn*(r1+r2)/r2;
    float vB = vIn;
    if(vIn<analogMin) vB=analogMin;
    int capacity = floatMap(vB , analogMin, analogMax, 0, 100);

    if ( isnan(t) || isnan(h)) {
      Serial.println("[ERROR] Please check the DHT sensor!");
      client.publish(debug_topic, "[ERROR] Please check the DHT sensor!", true);
	    delay(50);
      Serial.println("Going to sleep now because of ERROR");
      digitalWrite(sleepPIN, HIGH);
      return;
    }

    if (debug) {
      Serial.print("temperature: "); Serial.print(t); Serial.println("°C"); 
      Serial.print("humidity: "); Serial.print(h); Serial.println("%");
      Serial.print("battery voltage: "); Serial.print(voltage); Serial.println("V");
      Serial.print("battery capacity: "); Serial.print(capacity); Serial.println("%");
    } 
    delay(50);
    client.publish(temperature_topic, String(t).c_str(), true);
    if (debug) {    
      Serial.println("Temperature sent to MQTT.");
    }
    delay(50);
    client.publish(humidity_topic, String(h).c_str(), true);
    if (debug) {
    Serial.println("Humidity sent to MQTT.");
    }   
    delay(50);
    client.publish(voltage_topic, String(voltage).c_str(), true);
    if (debug) {
    Serial.println("Voltage sent to MQTT.");
    }   
    delay(50);
    client.publish(capacity_topic, String(capacity).c_str(), true);
    if (debug) {
    Serial.println("Battery capacity sent to MQTT.");
      if (capacity < 20){
        Serial.println("Please charge the battery!");
        client.publish(debug_topic, "The battery capacity is less than 20%. Please charge the battery!", true);
      }
    }   
  delay(50);
  Serial.println("Going to sleep as normal now.");
  digitalWrite(sleepPIN, HIGH);
}

void setup_wifi() {
  delay(20);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    if (connectCounter < 100) {
    delay(100);
    Serial.print(".");
    connectCounter = connectCounter +1;
    } else {
      Serial.println("Going to sleep now because of ERROR. Can not connected to WiFi.");
      digitalWrite(sleepPIN, HIGH);
    }
  }

 Serial.println("");
 Serial.println("The device has connected to the WiFi.");
 Serial.print("IP address: "); 
 Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT broker ...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("The device has connected to the MQTT broker.");
    } else if (counter == 2) {
      Serial.println("Going to sleep now because of ERROR. Can not connected to MQTT broker.");
      digitalWrite(sleepPIN, HIGH);
    } else {
      Serial.println("Unable to connect to MQTT broker. Wait 5 seconds before retry.");
      counter = counter +1;
      delay(5000);
    }
  }
}

void loop() { 
}