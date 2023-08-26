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
//Crear librería para control de los displays
#include "esp_adc_cal.h"
#include "config.h" //Archivo de librería de Adafruit y el WIFI

//**************************************************************************************************
// Conexión con ADAFRUIT IO
//**************************************************************************************************
//AdafruitIO_Feed *tempCanal = io.feed("Sensor");

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

//Pinout de los displays
#define pinDisplayA 27 //Pin A intercontado con multiplexeo para los 3 displays
#define pinDisplayB 13 //Pin B intercontado con multiplexeo para los 3 displays
#define pinDisplayC 12 //Pin C intercontado con multiplexeo para los 3 displays
#define pinDisplayD 22 //Pin D intercontado con multiplexeo para los 3 displays
#define pinDisplayE 34 //Pin E intercontado con multiplexeo para los 3 displays
#define pinDisplayF 26 //Pin F intercontado con multiplexeo para los 3 displays
#define pinDisplayG 21 //Pin G intercontado con multiplexeo para los 3 displays
//El pin del punto no lo coloco, debido a que siempre estará encendido en el mismo display, por lo que lo conecté directamente a voltaje

//Pinout del COM de los displays, conectados a los transistores
#define display1 23
#define display2 32
#define display3 33

//**************************************************************************************************
// Prototipos de funciones
//**************************************************************************************************
uint32_t readADC_Cal(int ADC_Raw); //Función para leer el sensor de temperatura con ADC de ESP32
void configurarPWM(void); //Función para configurar el PWM de las leds, el del servo está configurado en el setup
void temperatura_led(void); //Función para indicar la led que tiene que encender según la temperatura


//**************************************************************************************************
// Variables Globales
//**************************************************************************************************
int LM35_Raw_Sensor1 = 0; //En donde inicia el sensor
float LM35_TempC_Sensor1 = 0.0; //Temperatura en grados centígrados
float Voltage = 0.0; 
int botonpresionado; //Almacena el estado del botón
int botonanterior = 1; 
bool bandera = false; 
//Variables para colocar los valores de temperatura en el display
int decena; 
int unidad; 
int decimal; 

//Variables para identificar la temperatura, según la tabla de proyecto 
const float TEMP_LOW = 25.0;
const float TEMP_MEDIUM = 30.0; 
const float TEMP_HIGH = 35.0; 
int SERVO_LOW = 45; 
int SERVO_MEDIUM = 90; 
int SERVO_HIGH = 135; 

//**************************************************************************************************
// Configuración ADAFRUIT
//**************************************************************************************************
//Colocar el canal que se está utilizando 



//**************************************************************************************************
// Configuración
//**************************************************************************************************
void setup() {
  pinMode(toma_TEMP, INPUT_PULLUP);
  configurarPWM();

  //Configuración del PWM para el servo motor
  // Paso 1: Configurar el módulo PWM
  ledcSetup(pwmChannel, freqPWMS, resolutionS);
  // Paso 2: seleccionar en qué GPIO tendremos nuestra señal PWM
  ledcAttachPin(pinPWMS, pwmChannel);
  //Paso 3: Establecer la posición del servo motor inicialmente
  ledcWrite(0, map(180,0,180,0, 1023));

  Serial.begin(115200);

  //while(! Serial);
  //Serial.print("Conectando con Adafruit IO \n");
  //io.connect();
}

//**************************************************************************************************
// Loop Principal
//**************************************************************************************************
void loop() {
  //io.run();

  botonpresionado = digitalRead(toma_TEMP);
  delay(100);

  if (botonpresionado == LOW && botonanterior == HIGH) {
    bandera = true; 
  }

  if (bandera) {
    temperatura_led();
    delay(100);
    bandera = false; 
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
  LM35_TempC_Sensor1 = ((Voltage/4095)*3.25) / 0.01;
  
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
  } else if (LM35_TempC_Sensor1 >= TEMP_LOW && LM35_TempC_Sensor1 < TEMP_MEDIUM) {
    ledcWrite(ledRChannel, 0);
    ledcWrite(ledGChannel, 0);
    ledcWrite(ledBChannel, 255);
    ledcWrite(pwmChannel, map(SERVO_MEDIUM, 0, 180, 30, 115));
    Serial.print("LED Azul encendido \n");
  } else if (LM35_TempC_Sensor1 >= TEMP_MEDIUM && LM35_TempC_Sensor1 <= TEMP_HIGH) {
    ledcWrite(ledRChannel, 255);
    ledcWrite(ledGChannel, 0);
    ledcWrite(ledBChannel, 0);
    ledcWrite(pwmChannel, map(SERVO_HIGH, 0, 180, 30, 115));
    Serial.print("LED Rojo encendido \n");
  }
}