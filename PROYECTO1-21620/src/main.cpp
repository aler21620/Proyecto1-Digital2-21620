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
#include "esp_adc_cal.h"
//falta la librería de Adafruit y el WIFI


//**************************************************************************************************
// Definición de etiquetas
//**************************************************************************************************
#define LM35_Sensor1 35 //Sensor de temperatura 
#define pwmChannel 0 // 16 canales 0-15
#define freqPWMS 50   // Frecuencia en Hz (se ajusta a 50Hz para controlar el servomotor)
#define resolutionS 10 // 1-16 bits de resolución del servo
#define pinLedG 5 
#define pinLedY 18
#define pinLedR 19
#define pinPWMS 2    // GPIO 2 para tener la salida del PWM del servo 

//**************************************************************************************************
// Prototipos de funciones
//**************************************************************************************************
uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

//**************************************************************************************************
// Variables Globales
//**************************************************************************************************
int LM35_Raw_Sensor1 = 0;
float LM35_TempC_Sensor1 = 0.0;
float LM35_TempF_Sensor1 = 0.0;
float Voltage = 0.0; 


//**************************************************************************************************
// Configuración
//**************************************************************************************************
void setup() {
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
  LM35_TempC_Sensor1 = Voltage / 10;
  LM35_TempF_Sensor1 = (LM35_TempC_Sensor1 * 1.8) + 32;
 
  // Print The Readings
  Serial.print("Temperature = ");
  Serial.print(LM35_TempC_Sensor1);
  Serial.print(" °C , ");
  Serial.print("Temperature = ");
  Serial.print(LM35_TempF_Sensor1);
  Serial.println(" °F");
  
  delay(100);
}


//**************************************************************************************************
// Función para configurar módulo PWM
//**************************************************************************************************



