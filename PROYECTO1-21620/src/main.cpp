//**************************************************************************************************
//BE3029 - Proyecto 1 - Digital 2
//María Alejandra Rodríguez Roldán 
//No. 21620 
//**************************************************************************************************

//**************************************************************************************************
//Librerías 
//**************************************************************************************************
#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "display7.h"
#include "config.h" //Archivo de librería de Adafruit y el WIFI

//**************************************************************************************************
// Conexión con ADAFRUIT IO
//**************************************************************************************************
AdafruitIO_Feed *tempCanal = io.feed("Sensor");
AdafruitIO_Feed *LedCanal = io.feed("canalLed");
AdafruitIO_Feed *ServoCanal = io.feed("RELOJ");

//**************************************************************************************************
// Definición de etiquetas
//**************************************************************************************************
#define LM35_Sensor1 35 //Sensor de temperatura 
#define pwmChannel 0 // 16 canales 0-15
#define ledRChannel 1
#define ledGChannel 2
#define ledBChannel 3
#define freqPWM 5000 //Frencuencia en Hz para el uso de las leds
#define freqPWMS 50   // Frecuencia en Hz (se ajusta a 50Hz para controlar el servomotor)
#define resolution 8 //Resolución de 1 a 16 bits para la led RGB
#define resolutionS 10 // 1-16 bits de resolución del servo
#define pinLedR 5 
#define pinLedB 18
#define pinLedG 19
#define pinPWM 15 //GPIO para tener la salida del PWM
#define pinPWMS 2    // GPIO 2 para tener la salida del PWM del servo 
#define toma_TEMP 25 //Botón para la toma de temperatura

#define DA 27 //Pin A intercontado con multiplexeo para los 3 displays
#define DB 13 //Pin B intercontado con multiplexeo para los 3 displays
#define DC 12 //Pin C intercontado con multiplexeo para los 3 displays
#define DD 22 //Pin D intercontado con multiplexeo para los 3 displays
#define DE 4 //Pin E intercontado con multiplexeo para los 3 displays
#define DF 26 //Pin F intercontado con multiplexeo para los 3 displays
#define DG 21 //Pin G intercontado con multiplexeo para los 3 displays
#define pPunto 14
//El pin del punto no lo coloco, debido a que siempre estará encendido en el mismo display, por lo que lo conecté directamente a voltaje
#define display1 23
#define display2 32
#define display3 33
#define display4 15

//**************************************************************************************************
// Prototipos de funciones
//**************************************************************************************************
uint32_t readADC_Cal(int ADC_Raw); //Función para leer el sensor de temperatura con ADC de ESP32
void configurarPWM(void); //Función para configurar el PWM de las leds, el del servo está configurado en el setup
void temperatura_led(void); //Función para indicar la led que tiene que encender según la temperatura
void displaysvalor(void);
void handleMessage(AdafruitIO_Data *data);

//**************************************************************************************************
// Variables Globales
//**************************************************************************************************
int LM35_Raw_Sensor1 = 0; //En donde inicia el sensor
float LM35_TempC_Sensor1 = 0.0; //Temperatura en grados centígrados
float Voltage = 0.0; 
int botonpresionado; //Almacena el estado del botón
int botonanterior = 1; 
bool bandera = false; 
bool estadoR = false; 
bool estadoB = false; 
bool estadoG = false; 

//Variables para identificar la temperatura, según la tabla de proyecto 
const float TEMP_LOW = 24.0;
const float TEMP_MEDIUM = 25.0; 
const float TEMP_HIGH = 26.0; 
int SERVO_LOW = 45; 
int SERVO_MEDIUM = 90; 
int SERVO_HIGH = 135; 

//**************************************************************************************************
// Configuración
//**************************************************************************************************
void setup() {
  pinMode(toma_TEMP, INPUT_PULLUP);
  configurarPWM();
  
  configdisplay7(DA, DB, DC, DD, DE, DF, DG,pPunto);

  pinMode(display1, OUTPUT);
  pinMode(display2, OUTPUT);
  pinMode(display3, OUTPUT);
  pinMode(display4, OUTPUT);
  //Configuración de los transistores del multiplexeo
  digitalWrite(display1, LOW);
  digitalWrite(display2, LOW);
  digitalWrite(display3, LOW); 
  digitalWrite(display4, LOW); 
  
  
  //Configuración del PWM para el servo motor
  // Paso 1: Configurar el módulo PWM
  ledcSetup(pwmChannel, freqPWMS, resolutionS);
  // Paso 2: seleccionar en qué GPIO tendremos nuestra señal PWM
  ledcAttachPin(pinPWMS, pwmChannel);
  //Paso 3: Establecer la posición del servo motor inicialmente
  ledcWrite(0, map(180,0,180,0, 1023));

  Serial.begin(115200);

  while(! Serial);
  Serial.print("Conectando con Adafruit IO \n");
  io.connect();
  LedCanal->onMessage(handleMessage);

  while(io.status()< AIO_CONNECTED){
    Serial.print(".");
    delay(500);
  }

  LedCanal->get();

  Serial.println(); 
  Serial.println(io.statusText());
  estadoR = false;
  estadoB = false;
  estadoG = false; 
}

//**************************************************************************************************
// Loop Principal
//**************************************************************************************************
void loop() {
  displaysvalor();
  botonpresionado = digitalRead(toma_TEMP);
  
  if (botonpresionado == LOW && botonanterior == HIGH) {
    bandera = true; 
    if (bandera) {
      temperatura_led();
      delay(1000);
      io.run();
      Serial.print("sending ->");
      Serial.println(LM35_TempC_Sensor1);
      tempCanal-> save(LM35_TempC_Sensor1);
      bandera = false; 
    }
  }

  
  if(estadoR == true) {
    ledcWrite(ledRChannel, 255);
  } 

  if(estadoB == true) {
    ledcWrite(ledBChannel, 255);
  } 

  if(estadoG == true) {
    ledcWrite(ledGChannel, 255);
  } 

  
}

//****************************************************************
// Función para leer el sensor de temperatura
//****************************************************************
uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

//****************************************************************
// Función para configurar módulo PWM de la led RGB
//****************************************************************
void configurarPWM(void) {
  // Paso 1: Configurar el módulo PWM
  ledcSetup(pwmChannel, freqPWM, resolution);
  ledcSetup(ledRChannel, freqPWM, resolution);
  ledcSetup(ledGChannel, freqPWM, resolution);
  ledcSetup(ledBChannel, freqPWM, resolution);

  // Paso 2: seleccionar en qué GPIO tendremos nuestra señal PWM
  ledcAttachPin(pinPWM, pwmChannel);
  ledcAttachPin(pinLedR, ledRChannel);
  ledcAttachPin(pinLedG, ledGChannel);
  ledcAttachPin(pinLedB, ledBChannel);
}

//************************************************************************
// Función para actualizar el semáforo de leds y el movimiento del servo
//************************************************************************
void temperatura_led(void){
  // Read LM35_Sensor1 ADC Pin
  LM35_Raw_Sensor1 = analogRead(LM35_Sensor1);  
  // Calibrate ADC & Get Voltage (in mV)
  Voltage = readADC_Cal(LM35_Raw_Sensor1);
  // TempC = función con respecto al voltaje
  LM35_TempC_Sensor1 = ((Voltage/4095)*3.25) / 0.01; //De ser necesario se multiplica por un factor para que lea correctamente la temperatura
  
  // Imprimir las lecturas
  Serial.print("Lectura de la temperatura = ");
  Serial.print(LM35_TempC_Sensor1);
  Serial.print(" °C \n");
  
  //Para el semáforo de temperatura, al estar utilizando un led RGB, los colores indican lo siguiente: 
  //En la guía del proyecto, VERDE es menor a 37.0 °C, AMARILLO es mayor a 37.0 °C y menor a 37.5 °C y ROJO es mayor a 37.5 °C.
  //Con el LED RGB VERDE es menor a 37.0 °C, AZUL es mayor a 37.0 °C y menor a 37.5 °C y ROJO es mayor a 37.5 °C.
  if (LM35_TempC_Sensor1 < TEMP_LOW) {
    ledcWrite(ledRChannel, 0);
    ledcWrite(ledGChannel, 255);
    ledcWrite(ledBChannel, 0);
    ledcWrite(pwmChannel, map(SERVO_LOW, 0, 180, 30, 115));
    Serial.print("LED Verde encendido \n"); 
    Serial.print("sending ->");
    Serial.println(SERVO_LOW);
    ServoCanal-> save(SERVO_LOW); 
  } else if (LM35_TempC_Sensor1 >= TEMP_LOW && LM35_TempC_Sensor1 < TEMP_MEDIUM) {
    ledcWrite(ledRChannel, 0);
    ledcWrite(ledGChannel, 0);
    ledcWrite(ledBChannel, 255);
    ledcWrite(pwmChannel, map(SERVO_MEDIUM, 0, 180, 30, 115));
    Serial.print("LED Azul encendido \n");
    Serial.print("sending ->");
    Serial.println(SERVO_MEDIUM);
    ServoCanal-> save(SERVO_MEDIUM); 
  } else if (LM35_TempC_Sensor1 >= TEMP_MEDIUM && LM35_TempC_Sensor1 <= TEMP_HIGH) {
    ledcWrite(ledRChannel, 255);
    ledcWrite(ledGChannel, 0);
    ledcWrite(ledBChannel, 0);
    ledcWrite(pwmChannel, map(SERVO_HIGH, 0, 180, 30, 115));
    Serial.print("LED Rojo encendido \n");
    Serial.print("sending ->");
    Serial.println(SERVO_HIGH);
    ServoCanal-> save(SERVO_HIGH); 
  }
}

//************************************************************************
//Función para LED en Adafruit
//************************************************************************
void handleMessage(AdafruitIO_Data *data) {
  io.run();
  Serial.print("recieved <-");
  Serial.println(data->value());
  if(*data->value() == '1'){
    estadoR = true; 
  } else{
    estadoR = false;
  }

  if(*data->value() == '2'){
    estadoB = true; 
  } else{
    estadoB = false;
  }

  if(*data->value() == '4'){
    estadoG = true; 
  } else{
    estadoG = false;
  }
}

//************************************************************************
//Función para mostrar valor en los displays
//************************************************************************
void displaysvalor(void){
  int temperatura = LM35_TempC_Sensor1 * 100;
  int unidad = (temperatura/1) %10; 
  int decena = (temperatura/10) %10; 
  int decimal = (temperatura/100) % 10; 
  int centena = (temperatura/1000) %10;
  
  digitalWrite(display1, HIGH);
  digitalWrite(display2, LOW); 
  digitalWrite(display3, LOW);
  digitalWrite(display4, LOW);
  valor(centena);
  verpunto(0);
  delay(5);

  digitalWrite(display1, LOW);
  digitalWrite(display2, HIGH); 
  digitalWrite(display3, LOW);
  digitalWrite(display4, LOW);
  valor(decimal);
  verpunto(1);
  delay(5);

  digitalWrite(display1, LOW);
  digitalWrite(display2, LOW); 
  digitalWrite(display3, HIGH);
  digitalWrite(display4, LOW);
  valor(decena);
  verpunto(0);
  delay(5);

  digitalWrite(display1, LOW);
  digitalWrite(display2, LOW); 
  digitalWrite(display3, LOW);
  digitalWrite(display4, HIGH);
  valor(unidad);
  verpunto(0);
  delay(5);
}