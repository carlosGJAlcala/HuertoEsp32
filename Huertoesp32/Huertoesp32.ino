#include <WiFi.h>
#include <HTTPClient.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "Timer.h"

#define PIN_ANALOGLUZ_IN 36
#define PIN_ANALOG_IN 35
#define PIN_ANALOG_HUMEDAD_IN 34
#define PIN_TRANSISTOR_HUMEDAD 18
#define PIN_BOMBAAGUA_IN 21

#define SDA 13
#define SCL 14

#define trigPin 32
#define echoPin 33
#define MAX_DISTANCE 700

LiquidCrystal_I2C lcd(0x3F, 16, 2);
const bool on_Wifi = true;
const char *ssid_Router = "MOVISTAR_37F0";
const char *password_Router = "Fqp4Gi5VF7KLKLAyjLcY";
const char *serverURL = "http://192.168.1.41:8080/auth/sensor";
const char *serverURLDepositoAgua = "http://192.168.1.41:8080/auth/sensorDepositoAgua";
const char *macetaURL = "http://192.168.1.41:8080/auth/maceta/huerto/humedad/1";
const int profundidadMaceta = 17;
int tiempo = 4000;
Timer timer;

struct Sensor {
  int id;
  String nombreSensor;
  String magnitudAMedir;
  float cantidadMedida;
  String unidades;

  String toJSON() {
    StaticJsonDocument<200> doc;
    doc["id"] = id;
    doc["nombreSensor"] = nombreSensor;
    doc["magnitudAMedir"] = magnitudAMedir;
    doc["cantidadMedida"] = cantidadMedida;
    doc["unidades"] = unidades;

    String output;
    serializeJson(doc, output);
    return output;
  }
};

struct Humedad {
  int idMaceta;
  float humedadMediaMinimaRequerida;
  float humedadMediaMaximaRequerida;
  int numeroPlantas;

  void fromJSON(String jsonString) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (!error) {
      idMaceta = doc["idMaceta"];
      humedadMediaMinimaRequerida = doc["humedadMediaMinimaRequerida"];
      humedadMediaMaximaRequerida = doc["humedadMediaMaximaRequerida"];
      numeroPlantas = doc["numeroPlantas"];
    } else {
      Serial.println("Failed to parse JSON");
    }
  }
};

Humedad humedad;
Sensor sensorHumedad;
Sensor sensorTemperatura;
Sensor sensorLuz;
Sensor sensorDistancia;
long distancia;
double temperatura;
float luz;
float porcentajeHumedad;
float porcentaje;

void setup() {
  Serial.begin(115200);
  if (on_Wifi) {
    inicializarConexionWifi();
  }
  inicializarBombaAgua();
  inicializarUltrasonidos();
  inicializarLCD();
  inicializarSensorHumedad();
  obtenerDatosMaceta();
}

void obtenerDatosMaceta() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(macetaURL);

    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Received humedad Data: " + response);
      humedad.fromJSON(response);
    } else {
      Serial.printf("Error in getting humedad Data. HTTP Response code: %d\n", httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}

void limpiarPantalla() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Huerto: ");
  lcd.setCursor(0, 1);
}

void mostrarSensorEnLCD(Sensor sensor) {
  limpiarPantalla();
  if ("Sensor de Distancia" == sensor.nombreSensor) {
    porcentaje = 100 * (1 - (sensor.cantidadMedida / profundidadMaceta));
    lcd.print("Lleno");
    lcd.print(":");
    lcd.print(porcentaje);
    lcd.print(" ");
    lcd.print("%");
  } else {
    lcd.print(sensor.magnitudAMedir);
    lcd.print(":");
    lcd.print(sensor.cantidadMedida);
    lcd.print(" ");
    lcd.print(sensor.unidades);
  }

  Serial.println(sensor.toJSON());
  delay(tiempo);
}

void enviarSensorData(String jsonData) {
  if (on_Wifi) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverURL);
      http.addHeader("Content-Type", "application/json");

      int httpResponseCode = http.PUT(jsonData);

      if (httpResponseCode > 0) {
        Serial.printf("PUT se mando correctamente %d\n", httpResponseCode);
      } else {
        Serial.printf("Fallo HTTP Response code: %d\n", httpResponseCode);
      }
      http.end();
    } else {
      Serial.println("WiFi desconectado");
    }
  }
}

void enviarSensorDataSensorDepositoAgua(String jsonData) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURLDepositoAgua);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.PUT(jsonData);

    if (httpResponseCode > 0) {
      Serial.printf("PUT Request sent successfully! HTTP Response code: %d\n", httpResponseCode);
    } else {
      Serial.printf("Error in sending PUT Request. HTTP Response code: %d\n", httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}

void echarAgua(float porcentajeAguaAguaRequeridad) {
  Serial.println("Porcentaje");
  float distancia = getSonar();
  porcentaje = 100 * (1 - (distancia / profundidadMaceta));
  Serial.println(porcentaje);

  if (10.0 < porcentaje) {
    float porcentajeAgua = conseguirhumedad();
    Serial.println("Humedad");
    Serial.println(porcentajeAgua);

    if (porcentajeAgua < porcentajeAguaAguaRequeridad || porcentajeAgua < 10.0) {
      Serial.println("activar");
      activarBombaAgua();
      echarAgua(porcentajeAguaAguaRequeridad);
    }
  }
}

void inicializarBombaAgua() {
  pinMode(PIN_BOMBAAGUA_IN, OUTPUT);
}

void activarBombaAgua() {
  digitalWrite(PIN_BOMBAAGUA_IN, HIGH);
  delay(4000);
  digitalWrite(PIN_BOMBAAGUA_IN, LOW);
}

void loop() {
  timer.start();
  recogerDatos();
  enviarDatos();
  while (timer.read() < 60000) {
    Serial.println(timer.read());
    mostrarDatos();
    recogerDatos();
    porcentaje = 100 * (1 - (sensorDistancia.cantidadMedida / profundidadMaceta));
    if (10.0 < porcentaje) {
      Serial.println("sensorDistancia.cantidadMedida");
      echarAgua(humedad.humedadMediaMinimaRequerida);
    }
  }
}

void inicializarLCD() {
  Wire.begin(SDA, SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Huerto: ");
}

void inicializarSensorHumedad() {
  pinMode(PIN_TRANSISTOR_HUMEDAD, OUTPUT);
  pinMode(PIN_ANALOG_HUMEDAD_IN, INPUT);
}

void inicializarUltrasonidos() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void inicializarConexionWifi() {
  delay(2000);
  Serial.println("Setup start");
  WiFi.begin(ssid_Router, password_Router);
  Serial.println(String("Connecting to ") + ssid_Router);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected, IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Setup End");
}

long getSonar() {
  float timeOut = MAX_DISTANCE * 60;
  const float soundVelocity = 340.0;

  unsigned long pingTime;
  float distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pingTime = pulseIn(echoPin, HIGH, timeOut);

  if (pingTime == 0) {
    return -1;
  }

  distance = (float)pingTime * soundVelocity / 2 / 10000.0;

  return distance;
}

double conseguirTemperatura() {
  const float V_ref = 3.3;
  int adcValue = analogRead(PIN_ANALOG_IN);
  double voltage = (float)adcValue / 4095.0 * V_ref;
  double Rt = 10 * voltage / (3.3 - voltage);
  double tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950.0);
  double tempC = tempK - 273.15;
  return tempC;
}

float conseguirhumedad() {
  digitalWrite(PIN_TRANSISTOR_HUMEDAD, HIGH);
  delay(300);
  int analogValue = analogRead(PIN_ANALOG_HUMEDAD_IN);
  delay(300);
  digitalWrite(PIN_TRANSISTOR_HUMEDAD, LOW);
  float porcentajeHumedad = map(analogValue, 4095, 800, 0, 100);
  if (porcentajeHumedad < 0) {
    porcentajeHumedad = 0;
  }
  if (porcentajeHumedad > 100) {
    porcentajeHumedad = 100;
  }
  return porcentajeHumedad;
}

void recogerDatos() {
  temperatura = conseguirTemperatura();
  luz = analogRead(PIN_ANALOGLUZ_IN) / 4095.0 * 100;
  distancia = getSonar();
  porcentajeHumedad = conseguirhumedad();
}

void enviarDatos() {
  sensorHumedad = {1, "Sensor de Humedad", "Humedad", porcentajeHumedad, "%"};
  sensorTemperatura = {2, "Sensor de Temperatura", "Temperatura", temperatura, "C"};
  sensorLuz = {3, "Sensor de Luz", "Luminosidad", luz, "%"};
  sensorDistancia = {4, "Sensor de Distancia", "Distancia", distancia, "cm"};

  enviarSensorData(sensorHumedad.toJSON());
  enviarSensorData(sensorTemperatura.toJSON());
  enviarSensorData(sensorLuz.toJSON());
  enviarSensorDataSensorDepositoAgua(sensorDistancia.toJSON());
}

void mostrarDatos() {
  mostrarSensorEnLCD(sensorTemperatura);
  mostrarSensorEnLCD(sensorHumedad);
  mostrarSensorEnLCD(sensorLuz);
  mostrarSensorEnLCD(sensorDistancia);
}
