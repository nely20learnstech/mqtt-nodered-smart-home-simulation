#include "DHT.h" 
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <ESP32Servo.h>
// #include <LiquidCrystal_I2C.h>


#define WIFI_SSID "your_wifi_zone" // Nom du réseau Wi-Fi auquel nous sommes connectés
#define WIFI_PASSWORD "your_password" // Mot de passe du Wi-Fi 


// Adresse IP pour accéder à Mosquitto qui est le broker local 
#define MQTT_HOST IPAddress(192,168,147,99) 
#define MQTT_PORT 1883

// Définition des topics MQTT
#define MQTT_PUB_TEMP "home/dht/temperature" // Topic pour publier la température
#define MQTT_PUB_HUM  "home/dht/humidity" // Topic pour publier l’humidité
#define MQTT_PUB_LIGHTS "home/lights"  // Topic pour la lumière
#define MQTT_PUB_SERVO "home/servo"  // Topic pour le servo


// Définition des pins
#define DHTPIN 4  
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321 
#define LED_PIN 26
#define SERVO1_PIN 13
#define SERVO2_PIN 19

// LiquidCrystal_I2C lcd(0x27, 16, 2); 

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// Initialise le capteur DHT
DHT dht(DHTPIN, DHTTYPE);

// Initialise les deux servomoteurs
Servo servo1, servo2;

float temp; // Stocke la température
float hum; // Stocke l'humidité

unsigned long previousMillis = 0; // Stocke la dernière fois où la température a été publiée
const long interval = 10000;     // Intervalle auquel publier les mesures des capteurs

// Connecte ESP32 au routeur
void connectToWifi() {
  Serial.println("Connexion au Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempt = 0;

  // Teste la connexion au Wi-Fi
  while (WiFi.status() != WL_CONNECTED && attempt < 10) {
    delay(1000);
    Serial.print(".");
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connecté!");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Echec de la connexion au Wi-Fi.");
  }
}

// Connecte ESP32 au broker MQTT
void connectToMqtt() {
  Serial.println("Connexion à MQTT...");
  mqttClient.connect();
}

// Gère les événements Wi-Fi.
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connecté");
      Serial.println("Adresse IP: ");
      Serial.println(WiFi.localIP());
      connectToMqtt(); // Une fois connecté, nous nous connectons au broker
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Connexion WiFi perdue");
      xTimerStop(mqttReconnectTimer, 0); // Evite de tenter une reconnexion MQTT pendant la reconnexion Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// Callback de connexion MQTT
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connecté au broker MQTT.");
  Serial.print("Session présente : ");
  Serial.println(sessionPresent);
}

// Callback de déconnexion MQTT - Appelé lorsque ESP32 perd la connexion avec le broker MQTT
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Déconnecté du broker MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}


void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// Confirmation de publication MQTT - Appelé lorsque un message est publié à un topic MQTT
void onMqttPublish(uint16_t packetId) {
  Serial.print("Publication confirmée. ID du paquet : .");
  Serial.println(packetId);
}

// void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties,
//                    size_t len, size_t index, size_t total) {
//   String topicStr = String(topic);
//   String msg = "";
//   for (int i = 0; i < len; i++) {
//     msg += (char)payload[i];
//   }

  // Serial.printf("Received [%s]: %s\n", topic, msg.c_str());

  // if (topicStr == MQTT_SUB_LED) {
  //   if (msg == "ON") {
  //     digitalWrite(LED_STATUS, HIGH);
  //     lcd.clear();
  //     lcd.setCursor(0, 0);
  //     lcd.print("LED ON");
  //   } else {
  //     digitalWrite(LED_STATUS, LOW);
  //     lcd.clear();
  //     lcd.setCursor(0, 0);
  //     lcd.print("LED OFF");
  //   }
  // }

  // if (topicStr == MQTT_SUB_SERVO1) {
  //   int angle = msg.toInt();
  //   angle = constrain(angle, 0, 180);
  //   servo1.write(angle);
  //   lcd.clear();
  //   lcd.setCursor(0, 0);
  //   lcd.print("Servo1:");
  //   lcd.print(angle);
//     lcd.print(" deg");
//   }

//   if (topicStr == MQTT_SUB_SERVO2) {
//     int angle = msg.toInt();
//     angle = constrain(angle, 0, 180);
//     servo2.write(angle);
//     lcd.clear();
//     lcd.setCursor(0, 0);
//     lcd.print("Servo2:");
//     lcd.print(angle);
//     lcd.print(" deg");
//   }
// }

// Assigne une fonction de rappel (callback), donc quand l’ESP32 se connecte au Wi-Fi,
// il exécutera la fonction WiFiEvent() pour afficher les détails comme décrit précédemment.

void setup() {
  Serial.begin(115200);
  Serial.println();

  // Initialise les actionneurs et capteur
  dht.begin();
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  pinMode(LED_PIN, OUTPUT);

  // lcd.init();
  // lcd.backlight();
  // lcd.setCursor(0, 0);

  // Crée des minuteries (timers) qui permettront de se reconnecter au broker
  //  MQTT et au Wi-Fi en cas de perte de connexion  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, 
  (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, 
  (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  // Configure les callbacks pour le client MQTT
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  // mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  // Démarre la connexion au Wi-Fi
  connectToWifi();
}


void loop() {
  unsigned long currentMillis = millis();
  // Toutes les X secondes (ici 10 secondes),
  // une nouvelle donnée est publiée sur MQTT
  if (currentMillis - previousMillis >= interval) {
    // Enregistre le moment de la dernière publication
    previousMillis = currentMillis;
    
    // Nouvelles lectures du capteur DHT
    hum = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temp = dht.readTemperature();

    // Vérifie si la lecture a échoué, et quitte si c’est le cas
    if (isnan(temp) || isnan(hum)) {
      Serial.println(F("Échec de lecture du capteur DHT !"));
      return;
    }
    
    // Publication MQTT de la température sur le topic home/dht/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
    Serial.printf("Topic %s avec QoS 1", MQTT_PUB_TEMP);
    Serial.printf("Message : %.2f \n", temp);

    // Publication MQTT de l’humidité sur le topic home/dht/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
    Serial.printf("Topic %s avec QoS 1 ", MQTT_PUB_HUM);
    Serial.printf("Message : %.2f \n", hum);
    
  }
}
