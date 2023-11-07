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


// Replace the next variables with your SSID/Password combination
//const char* ssid = "Nome da sua Rede";
//const char* password = "Senha da sua Rede";
String ssid;

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "54.160.155.170";
const char* mqtt_username = "Embarcado"; // replace with your Credential
const char* mqtt_password = "Embarcado2023";
const int mqtt_port = 1883;
String cpf;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;


/*static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";*/


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
const int oneWireBus = 2;
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
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  setup_wifi();
  while (!Serial) delay(1);
  //espClient.setCACert(root_ca);
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
  WiFiManagerParameter parameter("my_cpf", "Digite o CPF:","Obrigatório", 12);
  wm.addParameter(&parameter);
  res = wm.autoConnect("GreenWarden"); // password protected ap
  cpf = parameter.getValue();
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
      publishString("alert", "Dispositivo Conectado");

      
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
  snprintf(data, sizeof(data), "%s/%s", ssid, cpf);
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
  snprintf(topic, sizeof(topic), "esp32/%s/%s/%s", macAddress.c_str(), sensorName, dataHora);
  
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
    publishString("alert", "Failed to read DHT22 data");
  }
}

void publishBS18B20() {
  sensors.requestTemperatures();
  float soilTemperature = sensors.getTempCByIndex(0);
  publishSensorData("SilTemperature", soilTemperature);
}

void publishSoilHumidity() {
  sensor_analog = analogRead(sensorHumidadeSolo);
  _moisture = ( 100 - ( (sensor_analog/4095.00) * 100 ) );
  publishSensorData("SilHumidity", _moisture);
}

float calcSoilHumidity() {
  float analogSilHumidity = analogRead(sensorHumidadeSolo);
  float valSilHumidity = ( 100 - ( (sensor_analog/4095.00) * 100 ) );
  return valSilHumidity;
}

void publishPH() {
  float calibration_value = 8.3;
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
  

  publishSensorData("PH", ph_act);
}
  
void publishLuminosity() {
  float luminosity = analogRead(sensorLuminosidade);

  publishSensorData("Luminosity", luminosity);
}


void publishAgua() {
  int agua = digitalRead(sensorAgua);  // Leitura da boia de água

  if (agua == LOW) { // Se o nível de água estiver baixo (LOW), publique
    publishString("alert", "Nível de Água Baixo, por favor, adicione água ao reservatório");  
  }
}

void publishNutrientes() {
  float nutrientes = digitalRead(sensorNutrientes);  // Leitura da boia de nutrientes

  if (nutrientes == LOW) { // Se o nível de nutrientes estiver baixo (LOW), publique
    publishString("alert", "Nível de Nutrientes Baixo, por favor, adicione nutrientes ao reservatório"); 
  }
}

void publishBatteryLevel() {
  float vPow = 5.0;
  float r1 = 22000;
  float r2 = 10000;
  float v = (analogRead(33) * vPow) / 1024.0;
  float v2 = v / (r2 / (r1 + r2));
  publishSensorData("BatteryLevel",v2);
}

void stateAgua() {
  unsigned long currentMillis = millis();
  // Leitura do sensor de umidade
  float analogHumidity = analogRead(sensorHumidadeSolo);
  float valueHumidity = ( 100 - ( (sensor_analog/4095.00) * 100 ) );

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
      Serial.println("Bomba ativada");
      digitalWrite(bombaNutrientes, HIGH);
      isPumpActive = true;
      pumpStartTime = millis();
    }

    if (isPumpActive && millis() - pumpStartTime >= pumpDuration) {

    digitalWrite(bombaNutrientes, LOW);
    Serial.println("Bomba desativada");
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
