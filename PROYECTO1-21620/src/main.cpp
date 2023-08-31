//**************************************************************************************************
//BE3029 - Proyecto 1 - Digital 2 
//Sensor de temperatura
//María Alejandra Rodríguez Roldán 
//No. 21620 
//**************************************************************************************************

//**************************************************************************************************
//Librerías 
//**************************************************************************************************
#include <Arduino.h> //Para trabajar con framework arduino
#include "driver/ledc.h" //Para poder controlar PWM
#include "esp_adc_cal.h" //Librería para utilización de ADC de ESP32
#include "display7.h" //Librería hecha para controlar los displays de 7 segmentos 
#include "config.h" //Archivo de librería de Adafruit y el WIFI, aquí está el usuario y contraseña de Adafruit así como la de internet

//**************************************************************************************************
// Conexión con ADAFRUIT IO
//**************************************************************************************************
//Conexión de los canales con Adafruit
AdafruitIO_Feed *tempCanal = io.feed("Sensor"); //Conexión con canal que recibe los datos de temperatura
AdafruitIO_Feed *LedCanal = io.feed("canalLed"); //Conexión con canal que va a mandar datos del led
AdafruitIO_Feed *ServoCanal = io.feed("RELOJ"); //Conexión que recibe las posiciones del servo 

//**************************************************************************************************
// Definición de etiquetas
//**************************************************************************************************
#define LM35_Sensor1 35 //Sensor de temperatura 
#define pwmChannel 0 // 16 canales 0-15
#define ledRChannel 1 //Canal de PWM para led rojo 
#define ledGChannel 2 //Canal de PWM para led verde
#define ledBChannel 3 //Canal de PWM para led azul 
#define freqPWM 5000 //Frencuencia en Hz para el uso de las leds
#define freqPWMS 50   // Frecuencia en Hz (se ajusta a 50Hz para controlar el servomotor)
#define resolution 8 //Resolución de 1 a 16 bits para la led RGB
#define resolutionS 10 // 1-16 bits de resolución del servo
#define pinLedR 5 //Conexión de pin al ESP 32 para el led rojo 
#define pinLedB 18 //Conexión de pin al ESP 32 para el led azul
#define pinLedG 19 //Conexión de pin al ESP 32 para el led verde
#define pinPWM 15 //GPIO para tener la salida del PWM
#define pinPWMS 2    // GPIO 2 para tener la salida del PWM del servo 
#define toma_TEMP 25 //Botón para la toma de temperatura

#define DA 27 //Pin A intercontado con multiplexeo para los displays
#define DB 13 //Pin B intercontado con multiplexeo para los displays
#define DC 12 //Pin C intercontado con multiplexeo para los displays
#define DD 22 //Pin D intercontado con multiplexeo para los displays
#define DE 4 //Pin E intercontado con multiplexeo para los displays
#define DF 26 //Pin F intercontado con multiplexeo para los displays
#define DG 21 //Pin G intercontado con multiplexeo para los displays
#define pPunto 14 //Pin punto interconectado con multiplexeo para los displays 
//El pin del punto no lo coloco, debido a que siempre estará encendido en el mismo display, por lo que lo conecté directamente a voltaje
#define display1 23 //Conexión del display 1, para controlarlo individualmente 
#define display2 32 //Conexión del display 2, para controlarlo individualmente 
#define display3 33 //Conexión del display 3, para controlarlo individualmente 
#define display4 15 //Conexión del display 4, para controlarlo individualmente 

//**************************************************************************************************
// Prototipos de funciones
//**************************************************************************************************
uint32_t readADC_Cal(int ADC_Raw); //Función para leer el sensor de temperatura con ADC de ESP32
void configurarPWM(void); //Función para configurar el PWM de las leds, el del servo está configurado en el setup
void temperatura_led(void); //Función para indicar la led que tiene que encender según la temperatura
void displaysvalor(void); //Función para mostrar el valor de temperatura en los displays
void handleMessage(AdafruitIO_Data *data); //Función para recibir el mensaje que manda Adafruit

//**************************************************************************************************
// Variables Globales
//**************************************************************************************************
int LM35_Raw_Sensor1 = 0; //En donde inicia el sensor
float LM35_TempC_Sensor1 = 0.0; //Temperatura en grados centígrados
float Voltage = 0.0; //Variable para utilizar en la función y obtener temperatura
int botonpresionado; //Almacena el estado del botón
int botonanterior = 1; //Variable bandera para el botón y revisar el estado anterior
bool bandera = false; //Variable bandera para el botón 
bool estadoR = false; //Estado de la led roja en Adafruit
bool estadoB = false; //Estado de la led azul en Adafruit
bool estadoG = false; //Estado de la led verde en Adafruit

//Variables para identificar la temperatura, según la tabla de proyecto 
const float TEMP_LOW = 24.0; //Valor mínimo de temperatura para considerarlo en estado bajo 
const float TEMP_MEDIUM = 25.0; //Valor medio de temperatura para considerarlo en estado medio
const float TEMP_HIGH = 26.0; //Valor medio de temperatura para considerarlo en estado alto 
int SERVO_LOW = 45; //Posición específica del servo para el estado bajo 
int SERVO_MEDIUM = 90; //Posición específica del servo para el estado medio
int SERVO_HIGH = 135; //Posición específica del servo para el estao alto 

//**************************************************************************************************
// Configuración
//**************************************************************************************************
void setup() {
  //Se configura el botón con resistencia interna pull - up 
  pinMode(toma_TEMP, INPUT_PULLUP);
  //Se llama la función que configura las leds para controlarlas con PWM
  configurarPWM();
  //Se llama la función que configura los displays de 7 segmentos  
  configdisplay7(DA, DB, DC, DD, DE, DF, DG,pPunto);

  //Se configuran los displays como salidas 
  pinMode(display1, OUTPUT);
  pinMode(display2, OUTPUT);
  pinMode(display3, OUTPUT);
  pinMode(display4, OUTPUT);

  //Configuración de los transistores del multiplexeo, para saber en que estado inician, por ser cátodo se coloca LOW
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

  //Se inicia la comunicación 
  Serial.begin(115200);

  //Se coloca la configuración para conectar con Adafruit 
  while(! Serial);
  Serial.print("Conectando con Adafruit IO \n");
  io.connect();

  //Se establece la conexión con el canal del led para recibir los datos de Adafruit
  LedCanal->onMessage(handleMessage);

  //Se configura para que aparezcan puntitos (........), mientras conecta con Adafruit
  while(io.status()< AIO_CONNECTED){
    Serial.print(".");
    delay(500);
  }

  //Para recibir los datos del canal de los leds
  LedCanal->get();

  //Imprime el estado en que está con Adafruit
  Serial.println(); 
  Serial.println(io.statusText());
  //Estados para el encendido y apagado de los leds con Adafruit
  estadoR = false;
  estadoB = false;
  estadoG = false; 
}

//**************************************************************************************************
// Loop Principal
//**************************************************************************************************
void loop() {
  //Se llama la función que va a poner el valor de temperatura en los displays 
  displaysvalor();

  //Leer el estado del botón 
  botonpresionado = digitalRead(toma_TEMP);
  
  //Se establece el condicional de la bandera del botón para evitar rebote
  if (botonpresionado == LOW && botonanterior == HIGH) {
    bandera = true; 
    //Si se cumple, se llama a la función para leer la temperatura
    if (bandera) {
      //Función que trabaja con el ADC para obtener un valor del sensor 
      temperatura_led();
      delay(1000);
      //Se pone a funcionar Adafruit solamente cuando se presiona el botón 
      io.run();
      //Manda los valores de temperatura a Adafruit 
      Serial.print("sending ->");
      Serial.println(LM35_TempC_Sensor1);
      tempCanal-> save(LM35_TempC_Sensor1);
      //Desactiva la bandera 
      bandera = false; 
    }
  }

  //Configuración de los estados de los leds para saber que hacer con los valores que envía Adafruit 
  //Led Rojo 
  if(estadoR == true) {
    ledcWrite(ledRChannel, 255);
  } 
  //Led Azul 
  if(estadoB == true) {
    ledcWrite(ledBChannel, 255);
  } 
  //Led Verde
  if(estadoG == true) {
    ledcWrite(ledGChannel, 255);
  } 
}

//****************************************************************
// Función para leer el sensor de temperatura
//****************************************************************
//Esta función utiliza el ADC, para obtener el dato del sensor, se utiliza ADC_RAW como variable par a
uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

//****************************************************************
// Función para configurar módulo PWM de la led RGB
//****************************************************************
//Esta es la función que configura el led RGB para que sea controlado por PWM y así poder variar su brillo 
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
//Esta función utiliza la función readADC_Cal para convertir el dato en crudo a un valor de temperatura 
void temperatura_led(void){
  // Leer el pin LM35_Sensor1 ADC 
  LM35_Raw_Sensor1 = analogRead(LM35_Sensor1);  
  //Calibrar ADC y tomar el voltaje en mV 
  Voltage = readADC_Cal(LM35_Raw_Sensor1);
  // TempC = función con respecto al voltaje
  LM35_TempC_Sensor1 = ((Voltage/4095)*3.25) / 0.01; //De ser necesario se multiplica por un factor para que lea correctamente la temperatura
  
  // Imprimir las lecturas, para saber si el sensor funciona 
  Serial.print("Lectura de la temperatura = ");
  Serial.print(LM35_TempC_Sensor1);
  Serial.print(" °C \n");
  
  //Para el semáforo de temperatura, al estar utilizando un led RGB, los colores indican lo siguiente: 
  //En la guía del proyecto, VERDE es menor a 37.0 °C, AMARILLO es mayor a 37.0 °C y menor a 37.5 °C y ROJO es mayor a 37.5 °C.
  //Con el LED RGB VERDE es menor a 37.0 °C, AZUL es mayor a 37.0 °C y menor a 37.5 °C y ROJO es mayor a 37.5 °C.
  //Para probar su funcionamiento, los valores, mínimo, máximo y medio pueden variar debido a que la temperatura ambiental 
  //generalmente es de 23 a 26 grados. 

  //Primer condicional, este nos dice que si se cumple que la temperatura tomada es menor a temp_low, se enciende la led verde y el servo está en 45 
  if (LM35_TempC_Sensor1 < TEMP_LOW) {
    ledcWrite(ledRChannel, 0);
    ledcWrite(ledGChannel, 255);
    ledcWrite(ledBChannel, 0);
    ledcWrite(pwmChannel, map(SERVO_LOW, 0, 180, 30, 115));
    Serial.print("LED Verde encendido \n"); 
    //Se configura para enviar el valor de la posición del servo a Adafruit
    Serial.print("sending ->");
    Serial.println(SERVO_LOW);
    ServoCanal-> save(SERVO_LOW); 
  //Segundo condicional, este nos dice que si se cumple que la temperatura tomada es menor a temp_medium, pero mayor a temp_low, se enciende la led azul y el servo está en 90
  } else if (LM35_TempC_Sensor1 >= TEMP_LOW && LM35_TempC_Sensor1 < TEMP_MEDIUM) {
    ledcWrite(ledRChannel, 0);
    ledcWrite(ledGChannel, 0);
    ledcWrite(ledBChannel, 255);
    ledcWrite(pwmChannel, map(SERVO_MEDIUM, 0, 180, 30, 115));
    //Se configura para enviar el valor de la posición del servo a Adafruit 
    Serial.print("LED Azul encendido \n");
    Serial.print("sending ->");
    Serial.println(SERVO_MEDIUM);
    ServoCanal-> save(SERVO_MEDIUM); 
  //Tercer condicional, este nos dice que si se cumple que la temperatura tomada es menor a temp_high, se enciende la led roja y el servo está en 135 
  } else if (LM35_TempC_Sensor1 >= TEMP_MEDIUM && LM35_TempC_Sensor1 <= TEMP_HIGH) {
    ledcWrite(ledRChannel, 255);
    ledcWrite(ledGChannel, 0);
    ledcWrite(ledBChannel, 0);
    ledcWrite(pwmChannel, map(SERVO_HIGH, 0, 180, 30, 115));
    //Se configura para enviar el valor de la posición del servo a Adafruit 
    Serial.print("LED Rojo encendido \n");
    Serial.print("sending ->");
    Serial.println(SERVO_HIGH);
    ServoCanal-> save(SERVO_HIGH); 
  }
}

//************************************************************************
//Función para LED en Adafruit
//************************************************************************
//Esta función ayuda a manejar los datos que adafruit envía al microcontrolador 
void handleMessage(AdafruitIO_Data *data) {
  //Se corre Adafruit
  io.run();
  //Recibe el dato 
  Serial.print("recieved <-");
  Serial.println(data->value());
  //Se crean condicionales para saber que hacer con el dato recibido 
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
//Función para obtener el valor que se desplegará en cada display
void displaysvalor(void){
  //La temperatura se multiplica por 100 para no tener decimales 
  int temperatura = LM35_TempC_Sensor1 * 100;
  //Se obtiene cada número por separado
  int unidad = (temperatura/1) %10; 
  int decena = (temperatura/10) %10; 
  int decimal = (temperatura/100) % 10; 
  int centena = (temperatura/1000) %10;
  
  //Se enciende el display correspondiente y con la función valor se indica que segmentos encender
  digitalWrite(display1, HIGH);
  digitalWrite(display2, LOW); 
  digitalWrite(display3, LOW);
  digitalWrite(display4, LOW);
  valor(centena);
  verpunto(0); //Para no prender el punto en este display 
  delay(5);

  digitalWrite(display1, LOW);
  digitalWrite(display2, HIGH); 
  digitalWrite(display3, LOW);
  digitalWrite(display4, LOW);
  valor(decimal);
  verpunto(1); //Se prende el punto en este display 
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