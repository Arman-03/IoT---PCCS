#include <PubSubClient.h>
#include <WiFi.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <ESP32_Servo.h>
WiFiClient client;

#define BMP_SDA 21
#define BMP_SCL 22  // Air Pressure Pins

Adafruit_BMP280 bmp;

#define DHTTYPE DHT11

#define dht_dpin1 27  //DHT Pins
#define dht_dpin2 26

#define led_red 13  //LED Pins
#define led_green 14

Servo myServo;

#define SEALEVELPRESSURE_HPA (1013.25)

DHT dht1(dht_dpin1, DHTTYPE);
DHT dht2(dht_dpin2, DHTTYPE);

// Ensure correct credentials to connect to your WiFi Network.
char ssid[] = "iot";
char pass[] = "manian19092006";

// Ensure that the credentials here allow you to publish and subscribe to the ThingSpeak channel.
#define channelID 2469286
const char mqttUserName[] = "FgoSDAsmIhk2BzU5IhgQNgU";
const char clientID[] = "FgoSDAsmIhk2BzU5IhgQNgU";
const char mqttPass[] = "KgD6BlO7g6J0VaMxc1KbGwmw";

#define mqttPort 1883

const char* server = "mqtt3.thingspeak.com";
int status = WL_IDLE_STATUS;
int connectionDelay = 1;
PubSubClient mqttClient(client);

// Function to handle messages from MQTT subscription.
void mqttSubscriptionCallback(char* topic, byte* payload, unsigned int length) {
  // Print the details of the message that was received to the serial monitor.
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Subscribe to ThingSpeak channel for updates.
void mqttSubscribe(long subChannelID) {
  String myTopic = "channels/" + String(subChannelID) + "/subscribe";
  mqttClient.subscribe(myTopic.c_str());
}

// Publish messages to a ThingSpeak channel.
void mqttPublish(long pubChannelID, String message) {
  String topicString = "channels/" + String(pubChannelID) + "/publish";
  mqttClient.publish(topicString.c_str(), message.c_str());
}

// Connect to WiFi.
void connectWifi() {
  Serial.print("Connecting to Wi-Fi...");
  // Loop until WiFi connection is successful
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    delay(connectionDelay * 1000);
    Serial.print(WiFi.status());
  }
  Serial.println("Connected to Wi-Fi.");
}

// Connect to MQTT server.
void mqttConnect() {
  // Loop until connected.
  while (!mqttClient.connected()) {
    // Connect to the MQTT broker.
    if (mqttClient.connect(clientID, mqttUserName, mqttPass)) {
      Serial.println("Connected to MQTT broker...");
    } 
    else {
      Serial.print("MQTT connection failed, rc = ");
      Serial.print(mqttClient.state());
      Serial.println(" Will try again in a few seconds");
      delay(connectionDelay * 1000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  // Delay to allow serial monitor to come up.
  delay(3000);

  connectWifi();

  mqttClient.setServer(server, mqttPort);

  dht1.begin();
  dht2.begin();
  
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  digitalWrite(led_red, LOW);
  digitalWrite(led_green, LOW);

  myServo.attach(23); 

}

void loop() {
  // Reconnect to WiFi if it gets disconnected.
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }

  // Connect if MQTT client is not connected and resubscribe to channel updates.
  if (!mqttClient.connected()) {
    mqttConnect();
    mqttSubscribe(channelID);
  }

  // Call the loop to maintain connection to the server.
  mqttClient.loop();
  float temperature1 = dht1.readTemperature();
  float humidity1 = dht1.readHumidity();
  float temperature2 = dht2.readTemperature();
  float humidity2 = dht2.readHumidity();
  float pressure = bmp.readPressure();  // Read pressure in Pa
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  // digitalWrite(led_red,HIGH);

  if ((temperature1 > 33 && humidity1 > 45) || (temperature2 > 33 && humidity2 > 45)) {
    myServo.write(180);  // Rotate servo to 180 degrees
    digitalWrite(led_green, LOW);
    delay(50);
    digitalWrite(led_red, HIGH);
  } else {
    myServo.write(0);  // Rotate servo back to 0 degrees
    digitalWrite(led_red, LOW);
    delay(50);
    digitalWrite(led_green, HIGH);
  }
  Serial.print("Temperature1: ");
  Serial.print(temperature1);
  Serial.print(" °C ,");
  Serial.print(" ");
  Serial.print("Humidity1: ");
  Serial.print(humidity1);
  Serial.println(" %");
  Serial.print("Temperature2: ");
  Serial.print(temperature2);
  Serial.print(" °C ,");
  Serial.print(" ");
  Serial.print("Humidity2: ");
  Serial.print(humidity2);
  Serial.print(" %");
  Serial.print(" °C, Pressure: ");
  Serial.print(pressure / 100.0);  // Convert pressure to hPa
  Serial.print(" hPa, Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");

  String message1 = "field1=" + String(temperature1);
  String message2 = "field2=" + String(humidity1);
  String message3 = "field3=" + String(temperature2);
  String message4 = "field4=" + String(humidity2);
  String message5 = "field5=" + String(pressure / 100.0);
  String message6 = "field6=" + String(altitude);
  String topicString = "channels/" + String(channelID) + "/publish";
  mqttClient.publish(topicString.c_str(), message1.c_str());
  mqttClient.publish(topicString.c_str(), message2.c_str());
  mqttClient.publish(topicString.c_str(), message3.c_str());
  mqttClient.publish(topicString.c_str(), message4.c_str());
  mqttClient.publish(topicString.c_str(), message5.c_str());
  mqttClient.publish(topicString.c_str(), message6.c_str());

  delay(15000);
}