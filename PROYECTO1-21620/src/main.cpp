//**************************************************************************************************
//Proyecto 1 - Digital 2
//María Alejandra Rodríguez Roldán 
//No. 21620 
//**************************************************************************************************

//**************************************************************************************************
//Librerías 
//**************************************************************************************************
#include <Arduino.h>
#include "driver/ledc.h"
//Crear librería paa control de los displays
#include "esp_adc_cal.h"
//falta la librería de Adafruit y el WIFI


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
int inicial = 90; //Ángulo inicial del servo 
int brillo = 255; //Brillo del led RGB
//Variables para colocar los valores de temperatura en el display
int decena; 
int unidad; 
int decimal; 

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
}

//**************************************************************************************************
// Loop Principal
//**************************************************************************************************
void loop() {
  // Read LM35_Sensor1 ADC Pin
  LM35_Raw_Sensor1 = analogRead(LM35_Sensor1);  
  // Calibrate ADC & Get Voltage (in mV)
  Voltage = readADC_Cal(LM35_Raw_Sensor1);
  // TempC = Voltage(mV) / 10
  LM35_TempC_Sensor1 = ((Voltage/4095)*3.25) / 0.01;

  // Print The Readings
  Serial.print("Temperatura = ");
  Serial.print(LM35_TempC_Sensor1);
  Serial.print(" °C \n");
  
  delay(100);

  ledcWrite(ledBChannel, 0);
  ledcWrite(ledGChannel, brillo);
  delay(1000);
  ledcWrite(ledGChannel, 0);
  ledcWrite(ledRChannel, brillo);
  delay(1000);
  ledcWrite(ledRChannel, 0);
  ledcWrite(ledBChannel, brillo);
  delay(1000);
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
