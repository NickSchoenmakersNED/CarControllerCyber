#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

// Motor driver pins
const int IN1 = 9;
const int IN2 = 10; 
const int IN3 = 11; 
const int IN4 = 12; 

// WiFi credentials
const char* ssid = "iotroam";
const char* password = "qNeW73lAUG";

// MQTT Broker
const char* mqtt_server = "145.93.236.62";
const int mqtt_port = 1883;
const char* mqtt_topic = "sensor/data";

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
    // Constructor
    SensorData() : temperature(0), humidity(0), airQuality(0) {}

    // Setters
    void setTemperature(float temp) {
        temperature = temp;
    }

    void setHumidity(float hum) {
        humidity = hum;
    }

    void setAirQuality(int airQ) {
        airQuality = airQ;
    }

    // Getters
    float getTemperature() const {
        return temperature;
    }

    float getHumidity() const {
        return humidity;
    }

    int getAirQuality() const {
        return airQuality;
    }

    // Method to get the all the data into a string.
    String toJSON() const {
        String json = "{";
        json += "\"temperature\":" + String(temperature) + ",";
        json += "\"humidity\":" + String(humidity) + ",";
        json += "\"airQuality\":" + String(airQuality);
        json += "}";
        return json;
    }
};

void connectToWiFi() {
    Serial.println("Connecting to IoTRoam WiFi...");

    WiFi.disconnect();
    delay(1000);
    WiFi.begin(ssid, password);

    unsigned long startAttemptTime = millis();
    const unsigned long timeout = 10000; // 10 seconds timeout

    while (WiFi.status() != WL_CONNECTED && (millis() - startAttemptTime) < timeout) {
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
        if (mqttClient.connect("ESP32Client")) { // MQTT client ID
            Serial.println(" connected!");
        } else {
            Serial.print(" Failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" Retrying in 2 seconds...");
            delay(2000);
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize the DS18B20 sensor
    sensors.begin();

    // Initialize the KY Humidity Sensor
    KYSensor.begin();

    delay(2000);

    // Connect to WiFi
    connectToWiFi();

    // Configure MQTT client
    mqttClient.setServer(mqtt_server, mqtt_port);
    connectToMQTT();

    // Set motor pins as outputs
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(airQualitySensorPin, INPUT);
}

// Create an instance of SensorData
SensorData sensorData;

void loop() {
    // Ensure WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Disconnected from WiFi. Reconnecting...");
        connectToWiFi();
    }

    // Ensure MQTT connection
    if (!mqttClient.connected()) {
        Serial.println("Disconnected from MQTT broker. Reconnecting...");
        connectToMQTT();
    }

    mqttClient.loop(); // Maintain MQTT connection

    // Request temperature from the DS18B20 sensor
    sensors.requestTemperatures();
    float tempDS18B20 = sensors.getTempCByIndex(0);

    // Read air quality sensor
    airQualityValue = analogRead(airQualitySensorPin);

    // Read humidity from the KY Humidity Sensor
    float humidityKYSensor = KYSensor.readHumidity();

    // Set values in the SensorData object
    sensorData.setTemperature(tempDS18B20);
    sensorData.setAirQuality(airQualityValue);

    if (isnan(humidityKYSensor)) {
        Serial.println("Failed to read from KY Sensor!");
    } else {
        sensorData.setHumidity(humidityKYSensor);
    }

    // Publish the sensor data to MQTT
    if (mqttClient.connected()) {
        String payload = sensorData.toJSON();
        Serial.print("Publishing message: ");
        Serial.println(payload);

        mqttClient.publish(mqtt_topic, payload.c_str());
    }

    // Example motor drive logic
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);
    delay(2000);
}