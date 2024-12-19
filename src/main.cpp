#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <ArduinoHttpClient.h>

// Motor driver pins
const int IN1 = 9;
const int IN2 = 10;
const int IN3 = 11;
const int IN4 = 12;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // UTC time

int counter = 0;

// WiFi credentials
const char* ssid = "iotroam";
const char* password = "qNeW73lAUG";
const int port = 5000;
WiFiServer server(port);

// MQTT Broker
const char* mqtt_server = "145.93.236.67";
const int mqtt_port = 1883;
const char* mqtt_topic = "sensor/data";

// Replace these with your actual API server details
const char* serverAddress = "145.93.236.67"; // Use the same host as the database
const int serverPort = 5000; // Your API's port (5000 is commonly used for development)

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// DS18B20 Setup
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// MQ-7 Sensor
const int airQualitySensorPin = 3;
int airQualityValue = 0;

// KY Humidity Sensor Setup
#define KY_SENSOR_PIN 8
#define KY_SENSOR_TYPE DHT11
DHT KYSensor(KY_SENSOR_PIN, KY_SENSOR_TYPE);

class SensorData {
private:
    float temperature;
    float humidity;
    int airQuality;

public:
    SensorData() : temperature(0), humidity(0), airQuality(0) {}

    void setTemperature(float temp) { temperature = temp; }
    void setHumidity(float hum) { humidity = hum; }
    void setAirQuality(int airQ) { airQuality = airQ; }

    float getTemperature() const { return temperature; }
    float getHumidity() const { return humidity; }
    int getAirQuality() const { return airQuality; }

    String toJSON() const {
        String json = "{\"temperature\":" + String(temperature) + ",";
        json += "\"humidity\":" + String(humidity) + ",";
        json += "\"airQuality\":" + String(airQuality) + "}";
        return json;
    }
};

void connectToWiFi() {
    Serial.println("Connecting to IoTRoam WiFi...");

    WiFi.disconnect();
    delay(1000);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WiFi!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi. Check credentials or network.");
    }
}

void connectToMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT broker...");
        if (mqttClient.connect("ESP32Client")) {
            Serial.println(" connected!");
        } else {
            Serial.print(" Failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" Retrying in 2 seconds...");
            delay(2000);
        }
    }
}

void sendToAPI(const String& table, float value) {
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient wifiClient;
        HttpClient client(wifiClient, serverAddress, serverPort);

        // Prepare JSON payload
        String payload = "{";
        payload += "\"table\":\"" + table + "\",";
        payload += "\"datetime\":\"" + timeClient.getFormattedTime() + "\",";
        payload += "\"waarde\":" + String(value, 2);
        payload += "}";

        Serial.println("Sending data to API: " + payload);

        client.beginRequest();
        client.post("/api/insertdata"); // API endpoint to handle database insertion
        client.sendHeader("Content-Type", "application/json");
        client.sendHeader("Content-Length", payload.length());
        client.beginBody();
        client.print(payload);
        client.endRequest();
        
    } else {
        Serial.println("Wi-Fi not connected. Can't send data to API.");
    }
}

void setup() {
    Serial.begin(115200);

    sensors.begin();
    KYSensor.begin();

    delay(2000);

    connectToWiFi();

    mqttClient.setServer(mqtt_server, mqtt_port);
    connectToMQTT();

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(airQualitySensorPin, INPUT);

    server.begin();
    Serial.println("Server started");

    timeClient.begin();
}

SensorData sensorData;

void loop() {
    delay(1000);
    Serial.println(WiFi.localIP());

    counter++;

    Serial.print("Looping");
    Serial.println(counter);
    // Reconnect to Wi-Fi if disconnected
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Disconnected from WiFi. Reconnecting...");
        connectToWiFi();
    }

    // Reconnect to MQTT broker if disconnected
    if (!mqttClient.connected()) {
        Serial.println("Disconnected from MQTT broker. Reconnecting...");
        connectToMQTT();
    }

    mqttClient.loop();

    // Read sensor data
    sensors.requestTemperatures();
    float tempDS18B20 = sensors.getTempCByIndex(0); // Temperature sensor
    airQualityValue = analogRead(airQualitySensorPin); // Air quality sensor
    float humidityKYSensor = KYSensor.readHumidity(); // Humidity sensor

    // Update sensor data object
    sensorData.setTemperature(tempDS18B20);
    sensorData.setAirQuality(airQualityValue);

    if (!isnan(humidityKYSensor)) {
        sensorData.setHumidity(humidityKYSensor);
    }

    // Publish and send to API every 30 iterations
    if (counter == 30) {
        counter = 0;

        Serial.print("Finished loop: ");
        Serial.println(counter);

        // Prepare payload for both MQTT and API
        String payload = sensorData.toJSON();
        timeClient.update();
        payload = payload.substring(0, payload.length() - 1) + ",\"time\":\"" + timeClient.getFormattedTime() + "\"}";

        // Publish to MQTT
        if (mqttClient.connected()) {
            Serial.print("Publishing to MQTT: ");
            Serial.println(payload);
            mqttClient.publish(mqtt_topic, payload.c_str());
        } else {
            Serial.println("Failed to publish to MQTT. MQTT client not connected.");
            connectToMQTT(); // Reconnect if disconnected
        }

        // Send to API
        sendToAPI("temperatuur", tempDS18B20); // Send temperature to API
        sendToAPI("luchtvocht", humidityKYSensor); // Send humidity to API
        sendToAPI("luchtkwaliteit", airQualityValue); // Send air quality to API

        // Verify execution reaches here
        Serial.println("Reset?");
    }

    // Handle incoming client messages
    if(!server.available())
    {
        server.begin(); 
    }
    WiFiClient carClient = server.available();
    if (carClient) {
        Serial.println("Client connected");

        if (carClient.connected()) {

            if (carClient.available()) {
                String message = carClient.readString();
                Serial.print("Message received: ");
                Serial.println(message);

                // Control motor based on received message
                if (message.indexOf('U') != -1) {
                    Serial.println("Forward");
                    digitalWrite(IN1, HIGH);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, HIGH);
                    digitalWrite(IN4, LOW);
                } else if (message.indexOf('L') != -1) {
                    Serial.println("Left");
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, HIGH);
                    digitalWrite(IN3, HIGH);
                    digitalWrite(IN4, LOW);
                } else if (message.indexOf('R') != -1) {
                    Serial.println("Right");
                    digitalWrite(IN1, HIGH);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, HIGH);
                } else if (message.indexOf('D') != -1) {
                    Serial.println("Down");
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, HIGH);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, HIGH);
                } else if (message.indexOf("stop") != -1) {
                    Serial.println("Stop");
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, LOW);
                }
            }
        }
        else{
            // Reset motor state when client disconnects
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
            Serial.println("Client disconnected.");
        }

    }
}