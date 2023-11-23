/*********
  Wagner Becker
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiManager.h>

String ssid;

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "54.160.155.170";
const char* mqtt_username = "Embarcado"; // replace with your Credential
const char* mqtt_password = "Embarcado2023";
const int mqtt_port = 1883;
String modulo;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;


#define DHTPIN 5 
#define DHTTYPE DHT22 
#define CONNECTION_TIMEOUT 10
#define NTP_SERVER     "pool.ntp.org"
#define UTC_OFFSET     -10800
#define UTC_OFFSET_DST 0
#define sensorAgua 26
#define sensorNutrientes 27
#define sensorHumidadeSolo 39
#define sensorLuminosidade 32
#define sensorPH 35
#define VERIFICA 0
#define LIGA_BOMBA 1
#define ESPERA_TEMPO 2
#define bombaNutrientes 25
#define bombaAgua 13

DHT dht(DHTPIN, DHTTYPE);
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
String macAddress;
bool isConnected = false;
bool verifTime = false;
bool verifRcpHumidity = false;
bool verifRcpNutrient = false;
float rcpHumidity = 20;
float rcpNutrient = 2;
unsigned long lastMacPublishTime = 0;
const unsigned long macPublishInterval = 5000;
float _moisture,sensor_analog;


volatile char state;
unsigned long interval = 60000;
unsigned long previousMillis = 0;

bool isPumpActive = false;
unsigned long pumpStartTime = 0;
const unsigned long pumpDuration = 15 * 1000;
int lastActivationDay = 0;
int lastActivationHour = 0;
bool res;


void setup() {
  Serial.begin(115200);
  setup_wifi();
  while (!Serial) delay(1);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  dht.begin();
  configTime(UTC_OFFSET, UTC_OFFSET_DST, NTP_SERVER);
  macAddress = WiFi.macAddress();
  ssid = WiFi.SSID();
  pinMode(sensorAgua, INPUT_PULLUP);
  pinMode(sensorNutrientes, INPUT_PULLUP);
  pinMode(sensorHumidadeSolo, INPUT);
  pinMode(sensorLuminosidade, INPUT);
  pinMode(sensorPH, INPUT);
  pinMode(bombaNutrientes, OUTPUT);
  pinMode(bombaAgua, OUTPUT);
}

void setup_wifi() {
  WiFiManager wm;
  //wm.resetSettings();
  WiFiManagerParameter parameter("my_module", "Digite o nome do Módulo:"," ", 30);
  wm.addParameter(&parameter);
  res = wm.autoConnect("GreenWarden"); // password protected ap
  modulo = parameter.getValue();
  if(!res) {
    Serial.println("Failed to connect");
    // ESP.restart();
  } 
  else {
    //if you get here you have connected to the WiFi    
    Serial.println("WiFi connected");
    }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  char connectTopic[50];
  char rcpHumidityTopic[50];
  char rcpNutrientTopic[50];
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  snprintf(connectTopic, sizeof(connectTopic), "%s/CONFIG/Connected", macAddress.c_str());
  snprintf(rcpHumidityTopic, sizeof(rcpHumidityTopic), "%s/CONFIG/Connected/minSilHumidity", macAddress.c_str());
  snprintf(rcpNutrientTopic, sizeof(rcpNutrientTopic), "%s/CONFIG/Connected/timeNutrient", macAddress.c_str());

  if (String(topic) == rcpHumidityTopic) {
      Serial.println("Humidade do Solo Minima recebida");
      rcpHumidity = messageTemp.toFloat();
      verifRcpHumidity = true;
      if(verifRcpHumidity && verifRcpNutrient){
        isConnected = true;
      }
  }

  if (String(topic) == rcpNutrientTopic) {
      Serial.println("Tempo Nutrientes recebido");
      rcpNutrient = messageTemp.toFloat();
      verifRcpNutrient = true;
      if(verifRcpHumidity && verifRcpNutrient){
        isConnected = true;
      }
  }
}

void reconnect() {
  // Loop until we're reconnected
  char connectTopic[50];
  char rcpHumidityTopic[50];
  char rcpNutrientTopic[50];
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = macAddress;   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");

      
      snprintf(connectTopic, sizeof(connectTopic), "%s/CONFIG/Connected", macAddress.c_str());
      snprintf(rcpHumidityTopic, sizeof(rcpHumidityTopic), "%s/CONFIG/Connected/minSilHumidity", macAddress.c_str());
      snprintf(rcpNutrientTopic, sizeof(rcpNutrientTopic), "%s/CONFIG/Connected/timeNutrient", macAddress.c_str());
      client.subscribe(connectTopic);
      client.subscribe(rcpHumidityTopic);
      client.subscribe(rcpNutrientTopic);
     

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

bool publishMacAddress() {
  // Envie o endereço MAC para o tópico CONFIG/Connect
  char topic[100];
  char data[100];
  snprintf(topic, sizeof(topic), "CONFIG/Connect/%s", macAddress.c_str());
  snprintf(data, sizeof(data), "%s/%s", ssid, modulo);
  return client.publish(topic, data);
}

void publishSensorData(const char* sensorName, float value) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Connection Error");
    return;
  }

  char dataHora[20];
  snprintf(dataHora, sizeof(dataHora), "%02d-%02d-%04d %02d:%02d:%02d", 
    timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_year + 1900, 
    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

  char topic[100];
  snprintf(topic, sizeof(topic), "esp32/%s/%s/%s", macAddress.c_str(), sensorName, dataHora);
  
  char data[28];
  snprintf(data, sizeof(data), "%.2f", value);
  
  client.publish(topic, data);
  
  Serial.print(topic);
  Serial.print("/");
  Serial.println(data);
}

void publishString(const char* sensorName, char* value) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Connection Error");
    return;
  }

  char dataHora[20];
  snprintf(dataHora, sizeof(dataHora), "%02d-%02d-%04d %02d:%02d:%02d", 
    timeinfo.tm_mon + 1, timeinfo.tm_mday,  timeinfo.tm_year + 1900, 
    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

  char topic[100];
  snprintf(topic, sizeof(topic), "%s/%s/%s", sensorName, macAddress.c_str(), dataHora);
  
  char data[50];
  snprintf(data, sizeof(data), "%s", value);
  
  client.publish(topic, data);
  
  Serial.print(topic);
  Serial.print("/");
  Serial.println(data);
}

void publishDHT22() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (!isnan(humidity) && !isnan(temperature)) {
    publishSensorData("AirHumidity", humidity);
    publishSensorData("RmTemperature", temperature);
  } else {
    publishString("alert", "Erro ao ler os dados da temperatura ambiente");
  }
}

void publishBS18B20() {
  sensors.requestTemperatures();
  float soilTemperature = sensors.getTempCByIndex(0);
  publishSensorData("SilTemperature", soilTemperature);
}

void publishSoilHumidity() {
  sensor_analog = analogRead(sensorHumidadeSolo);
  _moisture = 100 - ((sensor_analog - 1800.0) / (4095.0 - 1800.0)) * 100; // Calcula a umidade baseado na nova relação
  if (_moisture < 0) {
    _moisture = 0;
  } else if (_moisture > 100) {
    _moisture = 100;
  }
  publishSensorData("SilHumidity", _moisture);
}

void publishPH() {
  float calibration_value = 25.81;
  unsigned long int avgval; 
  int buffer_arr[10],temp;
  
  for(int i=0;i<10;i++) 
  { 
  buffer_arr[i]=analogRead(35);
  delay(30);
  }
 for(int i=0;i<9;i++)
 {
  for(int j=i+1;j<10;j++)
  {
    if(buffer_arr[i]>buffer_arr[j])
    {
      temp=buffer_arr[i];
      buffer_arr[i]=buffer_arr[j];
      buffer_arr[j]=temp;
    }
  }
 }
 avgval=0;
 for(int i=2;i<8;i++)
 avgval+=buffer_arr[i];
  float volt=(float)avgval*3.3/4096.0/6;
  float ph_act = -5.70 * volt + calibration_value;

  if (ph_act < 0) {
    ph_act = 0;
  } else if (ph_act > 14){
    ph_act = 14;
  }

  publishSensorData("PH", ph_act);
}
  
void publishLuminosity() {
  float luminosity = analogRead(sensorLuminosidade);

  publishSensorData("Luminosity", luminosity);
}


void publishAgua() {
  int agua = digitalRead(sensorAgua);  // Leitura da boia de água
  Serial.println(agua);
  if (agua == LOW) { // Se o nível de água estiver baixo (LOW), publique
    publishString("alert", "Nível de Água Baixo, por favor, adicione água ao reservatório");  
    Serial.println("Agua baixa");
  }
}

void publishNutrientes() {
  float nutrientes = digitalRead(sensorNutrientes);  // Leitura da boia de nutrientes
  Serial.println(nutrientes);
  if (nutrientes == LOW) { // Se o nível de nutrientes estiver baixo (LOW), publique
    publishString("alert", "Nível de Nutrientes Baixo, por favor, adicione nutrientes ao reservatório"); 
    Serial.println("Nutrientes baixa");
  }
}

void publishBatteryLevel() {
  float vPow = 5.0;
  float r1 = 22000;
  float r2 = 10000;
  float v = (analogRead(33) * vPow) / 4095;
  float v2 = (((v / (r2 / (r1 + r2))) - 1.74)/4.2)*100;
  if (v2 < 0) {
    v2 = 0;
  } else if (v2 > 100){
    v2 = 100;
  }
  publishSensorData("BatteryLevel",v2);
}

void stateAgua() {
  unsigned long currentMillis = millis();
  // Leitura do sensor de umidade
  float analogHumidity = analogRead(sensorHumidadeSolo);
  float valueHumidity = 100 - ((analogHumidity - 1800.0) / (4095.0 - 1800.0)) * 100;
   // Calcula a umidade baseado na nova relação
  if (valueHumidity < 0) {
    valueHumidity = 0;
  } else if (_valueHumidity > 100) {
    valueHumidity = 100;
  }

  switch (state) {
    case VERIFICA:
      
      if (valueHumidity < rcpHumidity) {
        // Se a umidade estiver abaixo do limite, ligue a bomba
        digitalWrite(bombaAgua, HIGH);
        Serial.println("Bomba Agua Ligada");
        publishString("alert", "Bomba de Água Ligada");
        state = LIGA_BOMBA;
      }
      break;

    case LIGA_BOMBA:
      // Mantenha a bomba ligada por 5 segundos
      if (currentMillis - previousMillis >= 5000) {
        digitalWrite(bombaAgua, LOW); // Desliga a bomba
        Serial.println("Bomba Agua Desligada");
        publishString("alert", "Bomba de Água Desligada");
        previousMillis = currentMillis;
        state = ESPERA_TEMPO;
      }
      break;

    case ESPERA_TEMPO:
      // Espere 1 minuto antes de fazer a próxima leitura do sensor
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        state = VERIFICA;
      }
      break;
  }
}


void stateNutrientes() {
  struct tm currentTime;
  while(!getLocalTime(&currentTime)) {
    Serial.println("Not Connected to NTP");
    delay(200);
  }
  if(!verifTime){
    lastActivationDay = currentTime.tm_mday; // Dia do mês
    lastActivationHour = currentTime.tm_hour; // Hora
    Serial.print("Última ativação dos nutrientes: ");
    Serial.print(lastActivationDay);
    Serial.print("/");
    Serial.print(currentTime.tm_mon + 1); // Mês é baseado em zero
    Serial.print("/");
    Serial.print(currentTime.tm_year + 1900); // Ano é o ano atual - 1900
    Serial.print(" ");
    Serial.print(lastActivationHour);
    Serial.print(":");
    Serial.print(currentTime.tm_min);
    Serial.println();
    verifTime = true;
  }
  
    int hoursElapsed = hoursBetween(currentTime, lastActivationDay, lastActivationHour);
    
    if (hoursElapsed >= rcpNutrient) {
      // É hora de adicionar nutrientes, ative as bombas de nutrientes
      Serial.println("Bomba ativada");
      publishString("alert", "Bomba de Nutrientes Ligada");

      // Atualize o dia e a hora da última ativação
      lastActivationDay = currentTime.tm_mday;
      lastActivationHour = currentTime.tm_hour;

      // Imprima a próxima ativação
      Serial.print("Próxima ativação: ");
      Serial.print(lastActivationDay);
      Serial.print("/");
      Serial.print(currentTime.tm_mon + 1); // Mês é baseado em zero
      Serial.print("/");
      Serial.print(currentTime.tm_year + 1900); // Ano é o ano atual - 1900
      Serial.print(" ");
      Serial.print(lastActivationHour);
      Serial.print(":");
      Serial.print(currentTime.tm_min);
      Serial.println();
      digitalWrite(bombaNutrientes, HIGH);
      isPumpActive = true;
      pumpStartTime = millis();
    }

    if (isPumpActive && millis() - pumpStartTime >= pumpDuration) {

    digitalWrite(bombaNutrientes, LOW);
    publishString("alert", "Bomba de Nutrientes Desligada");
    isPumpActive = false;
  }
}

int hoursBetween(struct tm currentTime, int lastDay, int lastHour) {
  int daysDiff = currentTime.tm_mday - lastDay;
  int hoursDiff = currentTime.tm_hour - lastHour;

  // Considera os minutos para um cálculo mais preciso
  int minutesDiff = (daysDiff * 24 + hoursDiff) * 60 + currentTime.tm_min;

  return minutesDiff / 60; // Retorna a diferença em horas
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Enviar o endereço MAC apenas se a ESP32 estiver conectada
  unsigned long currentMillis = millis();
  if (!isConnected && client.connected() && currentMillis - lastMacPublishTime >= macPublishInterval) {
    lastMacPublishTime = currentMillis;  // Atualize o tempo da última publicação
    publishMacAddress();
    Serial.println("MAC Address published successfully");
  }
  if (isConnected) {
    // Executar rotinas enquanto estiver conectado
    struct tm timecalc;
    if (!getLocalTime(&timecalc)) {
      Serial.print("Not Connected to NTP");
      return;
    }
    int min = timecalc.tm_min;
    static bool executedThisMinute = false;
    unsigned long currentDay = timecalc.tm_min;

    if (min % 2 == 0 && !executedThisMinute) {
      executedThisMinute = true;

      //publishLocalTime();
      publishDHT22();
      publishBS18B20();
      publishNutrientes();
      publishAgua();
      publishPH();
      publishSoilHumidity();
      publishLuminosity();
      publishBatteryLevel();
      
    } else if  (min % 2 != 0) {
      executedThisMinute = false;  // Redefina a variável para permitir a execução no próximo minuto par
    }

    stateAgua();
    stateNutrientes();

    // Verifique se a conexão MQTT foi perdida ou o Wi-Fi desconectado
    if (!client.connected() || WiFi.status() != WL_CONNECTED) {
      isConnected = false; // Defina o estado de conexão como false
      publishString("alert", "Conexão MQTT ou Wi-Fi perdida. Reconectando...");
      Serial.println("Conexão MQTT ou Wi-Fi perdida. Reconectando...");
    }
  }
}