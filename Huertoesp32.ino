#include <WiFi.h>
#include <HTTPClient.h>  // Include the HTTPClient library for HTTP requests
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <ArduinoJson.h>  // Include the ArduinoJson library

#include "Timer.h"


#define PIN_ANALOGLUZ_IN 36
#define PIN_ANALOG_IN 35
#define PIN_ANALOG_HUMEDAD_IN 34
#define PIN_TRANSISTOR_HUMEDAD 18
#define PIN_BOMBAAGUA_IN 21

#define SDA 13  // Define SDA pins
#define SCL 14  // Define SCL pins

// Definición de pines para el sensor ultrasónico
#define trigPin 32        // define trigPin
#define echoPin 33        // define echoPin
#define MAX_DISTANCE 700  // Distancia máxima del sensor en cm

LiquidCrystal_I2C lcd(0x3F, 16, 2);
const bool on_Wifi = true;
const char *ssid_Router = "MOVISTAR_37F0";                                               // Enter the router name
const char *password_Router = "Fqp4Gi5VF7KLKLAyjLcY";                                    // Enter the router password
const char *serverURL = "http://192.168.1.41:8080/auth/sensor";                          // Server URL for PUT requests
const char *serverURLDepositoAgua = "http://192.168.1.41:8080/auth/sensorDepositoAgua";  // Server URL for PUT requests
const char *macetaURL = "http://192.168.1.41:8080/auth/maceta/huerto/humedad/1";         // Server URL for Maceta data
const int profundidadMaceta = 17;
int tiempo = 4000;
Timer timer;


struct Sensor {
  int id;
  String nombreSensor;
  String magnitudAMedir;
  float cantidadMedida;
  String unidades;

  // Convert struct Sensor to JSON format
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
// Structure to store Maceta data
struct Humedad {
  int idMaceta;
  float humedadMediaMinimaRequerida;
  float humedadMediaMaximaRequerida;
  int numeroPlantas;

  // Function to convert JSON to struct
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

void setup() {
  Serial.begin(115200);
  if (on_Wifi) {
    inicializarConexionWifi();
  }
  inicializarBombaAgua();  // Inicializa el pin de la bomba de agua
  inicializarUltrasonidos();
  inicializarLCD();
  inicializarSensorHumedad();
  obtenerDatosMaceta();
}
void obtenerDatosMaceta() {
  if (WiFi.status() == WL_CONNECTED) {  // Check WiFi connection status
    HTTPClient http;
    http.begin(macetaURL);  // Specify the destination for the GET request

    int httpResponseCode = http.GET();  // Send GET request

    if (httpResponseCode > 0) {
      String response = http.getString();  // Get the response
      Serial.println("Received humedad Data: " + response);
      humedad.fromJSON(response);  // Parse the JSON response to populate the struct
    } else {
      Serial.printf("Error in getting humedad Data. HTTP Response code: %d\n", httpResponseCode);
    }
    http.end();  // Free resources
  } else {
    Serial.println("WiFi Disconnected");
  }
}



void limpiarPantalla() {
  lcd.clear();
  lcd.setCursor(0, 0);    // Move the cursor to row 0, column 0
  lcd.print("Huerto: ");  // The print content is displayed on the LCD
  lcd.setCursor(0, 1);
}

void principal() {
  sensorHumedad = { 1, "Sensor de Humedad", "Humedad", conseguirhumedad(), "%" };
  sensorTemperatura = { 2, "Sensor de Temperatura", "Temperatura", conseguirTemperatura(), "C" };
  sensorLuz = { 3, "Sensor de Luz", "Luz", conseguirLuz(), "Lux" };
  sensorDistancia = { 4, "Sensor de Distancia", "Distancia", getSonar(), "cm" };

  mostrarSensorEnLCD(sensorHumedad);
  enviarSensorData(sensorHumedad.toJSON());

  mostrarSensorEnLCD(sensorTemperatura);
  enviarSensorData(sensorTemperatura.toJSON());

  mostrarSensorEnLCD(sensorLuz);
  enviarSensorData(sensorLuz.toJSON());

  mostrarSensorEnLCD(sensorDistancia);
  enviarSensorDataSensorDepositoAgua(sensorDistancia.toJSON());
  Serial.println(sensorDistancia.cantidadMedida);

  Serial.println(sensorDistancia.cantidadMedida);
  Serial.println(profundidadMaceta);


  if (sensorDistancia.cantidadMedida < profundidadMaceta) {
    Serial.println("sensorDistancia.cantidadMedida");

    echarAgua(humedad.humedadMediaMinimaRequerida);
  }
}

void mostrarSensorEnLCD(Sensor sensor) {
  limpiarPantalla();
  if ("Sensor de Distancia" == sensor.nombreSensor) {
    float resultado = 100 * (1 - (sensor.cantidadMedida / profundidadMaceta));
    lcd.print("Lleno");
    lcd.print(":");
    lcd.print(resultado);
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
    if (WiFi.status() == WL_CONNECTED) {  // Check WiFi connection status
      HTTPClient http;
      http.begin(serverURL);                               // Specify the destination for the PUT request
      http.addHeader("Content-Type", "application/json");  // Specify content-type header

      int httpResponseCode = http.PUT(jsonData);  // Send PUT request

      if (httpResponseCode > 0) {
        Serial.printf("PUT Request sent successfully! HTTP Response code: %d\n", httpResponseCode);
      } else {
        Serial.printf("Error in sending PUT Request. HTTP Response code: %d\n", httpResponseCode);
      }
      http.end();  // Free resources
    } else {
      Serial.println("WiFi Disconnected");
    }
  }
}
void enviarSensorDataSensorDepositoAgua(String jsonData) {
  if (WiFi.status() == WL_CONNECTED) {  // Check WiFi connection status
    HTTPClient http;
    http.begin(serverURLDepositoAgua);                   // Specify the destination for the PUT request
    http.addHeader("Content-Type", "application/json");  // Specify content-type header

    int httpResponseCode = http.PUT(jsonData);  // Send PUT request

    if (httpResponseCode > 0) {
      Serial.printf("PUT Request sent successfully! HTTP Response code: %d\n", httpResponseCode);
    } else {
      Serial.printf("Error in sending PUT Request. HTTP Response code: %d\n", httpResponseCode);
    }
    http.end();  // Free resources
  } else {
    Serial.println("WiFi Disconnected");
  }
}
void echarAgua(float porcentajeAguaAguaRequeridad) {
  Serial.println("Distancia");
  float distancia = getSonar();
  Serial.println(distancia);

  if (distancia < profundidadMaceta) {
    float porcentajeAgua = conseguirhumedad();
    Serial.println("Humedad");
    Serial.println(porcentajeAgua);

    if (porcentajeAgua < porcentajeAguaAguaRequeridad) {
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
  digitalWrite(PIN_BOMBAAGUA_IN, HIGH);  // Apagar el relé con HIGH
  delay(4000);
  digitalWrite(PIN_BOMBAAGUA_IN, LOW);  // Activar el relé con LOW
}

void loop() {
  timer.start();
  recogerDatos();
  enviarDatos();
  while (timer.read() < 60000) {
    Serial.println(timer.read());
    mostrarDatos();
    recogerDatos();
    if (sensorDistancia.cantidadMedida < profundidadMaceta) {
      Serial.println("sensorDistancia.cantidadMedida");
      echarAgua(humedad.humedadMediaMinimaRequerida);
    }
  }


  //principal();
  //echarAgua(60);
  //conseguirLuz();
  //delay(tiempo);
}

void inicializarLCD() {
  Wire.begin(SDA, SCL);   // attach the IIC pin
  lcd.init();             // LCD driver initialization
  lcd.backlight();        // Turn on the backlight
  lcd.setCursor(0, 0);    // Move the cursor to row 0, column 0
  lcd.print("Huerto: ");  // The print content is displayed on the LCD
}

void inicializarSensorHumedad() {
  pinMode(PIN_TRANSISTOR_HUMEDAD, OUTPUT);
  pinMode(PIN_ANALOG_HUMEDAD_IN, INPUT);
}

void inicializarUltrasonidos() {
  // Configuración de pines
  pinMode(trigPin, OUTPUT);  // Configura trigPin como salida
  pinMode(echoPin, INPUT);   // Configura echoPin como entrada
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
  float timeOut = MAX_DISTANCE * 60;  // Calcula el tiempo de espera
  const float soundVelocity = 340.0;  // Velocidad del sonido en m/s

  unsigned long pingTime;
  float distance;

  // Envío del pulso de disparo
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Espera y mide el tiempo hasta recibir el eco
  pingTime = pulseIn(echoPin, HIGH, timeOut);

  // Verificar si el tiempo de espera expiró
  if (pingTime == 0) {
    return -1;  // Retorna -1 si hay un error
  }

  // Calcular la distancia según el tiempo medido
  distance = (float)pingTime * soundVelocity / 2 / 10000.0;

  return distance;  // Retorna la distancia en cm
}

double conseguirTemperatura() {
  const float V_ref = 3.3;
  int adcValue = analogRead(PIN_ANALOG_IN);                        // read ADC pin
  double voltage = (float)adcValue / 4095.0 * V_ref;               // calculate voltage
  double Rt = 10 * voltage / (3.3 - voltage);                      // calculate resistance value of thermistor
  double tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950.0);  // calculate temperature (Kelvin)
  double tempC = tempK - 273.15;                                   // calculate temperature (Celsius)
  return tempC;
}


float conseguirhumedad() {
  digitalWrite(PIN_TRANSISTOR_HUMEDAD, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRANSISTOR_HUMEDAD, HIGH);
  const float V_ref = 5;
  int adcValue = analogRead(PIN_ANALOG_HUMEDAD_IN);  // Leer pin ADC
  float voltage = (float)adcValue / 4095.0 * V_ref;  // Calcular voltaje
  float porcentaje = (1 - (voltage / V_ref)) * 100;
  digitalWrite(PIN_TRANSISTOR_HUMEDAD, LOW);
  return porcentaje;
}

float conseguirLuz() {
  const float V_ref = 3.3;
  const float A = 423233;
  const float B = 0.84;
  const float R_fix = 10000.0;  // Resistencia fija de 10kΩ en serie con el LDR

  int adcValue = analogRead(PIN_ANALOGLUZ_IN);       // Leer pin ADC
  float voltage = (float)adcValue / 4095.0 * V_ref;  // Calcular voltaje
  float R_ldr = R_fix / ((V_ref / voltage) - 1);     // Calcular resistencia del LDR
  float L = pow((R_ldr / A), -1 / B);                // Calcular luxes
  Serial.printf("Resistencia: %.3f Voltaje: %.3f Adc: %3.f lux: %3.f \n", R_ldr, voltage, adcValue, L);

  return L;
}

void recogerDatos() {
  sensorHumedad = { 1, "Sensor de Humedad", "Humedad", conseguirhumedad(), "%" };
  sensorTemperatura = { 2, "Sensor de Temperatura", "Temperatura", conseguirTemperatura(), "C" };
  sensorLuz = { 3, "Sensor de Luz", "Luz", conseguirLuz(), "Lux" };
  sensorDistancia = { 4, "Sensor de Distancia", "Distancia", getSonar(), "cm" };
}
void enviarDatos() {
  enviarSensorData(sensorHumedad.toJSON());
  enviarSensorData(sensorTemperatura.toJSON());
  enviarSensorData(sensorLuz.toJSON());
  enviarSensorDataSensorDepositoAgua(sensorDistancia.toJSON());
}
void mostrarDatos() {
  mostrarSensorEnLCD(sensorHumedad);
  mostrarSensorEnLCD(sensorTemperatura);
  mostrarSensorEnLCD(sensorLuz);
  mostrarSensorEnLCD(sensorDistancia);
}
