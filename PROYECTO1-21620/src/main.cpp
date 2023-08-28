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

//**************************************************************************************************
// Prototipos de funciones
//**************************************************************************************************
uint32_t readADC_Cal(int ADC_Raw); //Función para leer el sensor de temperatura con ADC de ESP32
void configurarPWM(void); //Función para configurar el PWM de las leds, el del servo está configurado en el setup
void temperatura_led(void); //Función para indicar la led que tiene que encender según la temperatura
void displays(void);  //Función para inicializar los displays y mostrar el valor 
void valor_temperatura(void); //Función para arreglar el valor de temperatura y poder mostrarlo en los displays

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

int pinesDisplay[7] = {27, 13, 12, 22, 4, 26, 21}; //No coloco el pin del punto debido a que siempre estará encendido
int pinesTransistores[3] = {23, 32, 33};
bool displaysencendidos = false; 
int valorestemperatura[4];

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

  for (int i = 0; i < 7; i++) {
    pinMode(pinesDisplay[i], OUTPUT);
  }

  for (int i = 0; i < 3; i++) {
    pinMode(pinesTransistores[i], OUTPUT);
  }

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

  if (!displaysencendidos && botonpresionado == LOW) {
    displaysencendidos = true;
  }

  if (botonpresionado == LOW && botonanterior == HIGH) {
    bandera = true; 
    if(displaysencendidos){
      if (bandera) {
        temperatura_led();
        valor_temperatura();
        displays();
        delay(100);
        bandera = false; 
      }
    }
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

//************************************************************************
// Función para mostrar el valor de temperatura en los displays
//************************************************************************
void displays(){
  //Esta variable, va a permitir tener todos los valores de los segmentos para mostrar en el display
  //Por ser un display de cátodo, el 1 significa que se va a encender y el 0 que se va a apagar
  int numeros[10][7] = {
    {1, 1, 1, 1, 1, 1, 0}, // Número 0
    {0, 1, 1, 0, 0, 0, 0}, // Número 1
    {1, 1, 0, 1, 1, 0, 1}, // Número 2
    {1, 1, 1, 1, 0, 0, 1}, // Número 3
    {0, 1, 1, 0, 0, 1, 1}, // Número 4
    {1, 0, 1, 1, 0, 1, 1}, // Número 5
    {1, 0, 1, 1, 1, 1, 1}, // Número 6
    {1, 1, 1, 0, 0, 0, 0}, // Número 7
    {1, 1, 1, 1, 1, 1, 1}, // Número 8
    {1, 1, 1, 1, 0, 1, 1}  // Número 9
  };
  
  int estadopinestransistores[3][3] = {
    {1, 0, 0}, // Esta sección habilita el display 1, en este caso el transistor 
    {0, 1, 0}, // Esta sección habilita el display 2, en este caso el transistor 
    {0, 0, 1}  // Esta sección habilita el display 3, en este caso el transistor 
  };

  for (int n = 0; n < 3; n++) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(pinesTransistores[i], estadopinestransistores[n][i]);
    }

    for (int j = 0; j < 7; j++) {
      digitalWrite(pinesDisplay[j], numeros[valorestemperatura[n]][j]);
    }

    delay(5); //Se coloca este delay para que no se perciba el cambio en los displays por el ojo humano
  }
}

//************************************************************************
//Función para arreglar el valor de la temperatura para los displays
//************************************************************************
void valor_temperatura(){
  int temperatura = LM35_TempC_Sensor1 * 100; 

  valorestemperatura[3] = ((temperatura) / 1) % 10;
  valorestemperatura[2] = ((temperatura) / 10) % 10;
  valorestemperatura[1] = ((temperatura) / 100) % 10;
  valorestemperatura[0] = ((temperatura) / 1000) % 10;
}
