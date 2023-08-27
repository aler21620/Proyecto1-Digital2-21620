//**************************************************************************************************
//Creación de librería de 7 segmentos
//**************************************************************************************************
#ifndef __DISPLAY7_H__
#define __DISPLAY7_H__

//**************************************************************************************************
//Librerías 
//**************************************************************************************************
#include <Arduino.h>

//**************************************************************************************************
//Variables globales
//**************************************************************************************************
extern uint8_t pinDisplayA, pinDisplayB, pinDisplayC, pinDisplayD, pinDisplayE, pinDisplayF, pinDisplayG;  


//**************************************************************************************************
//Prototipos de funciones
//**************************************************************************************************
//Función para configurar el display de 7 segmentos
void configdisplay7(uint8_t pinDisplayA, uint8_t pinDisplayB, uint8_t pinDisplayC, uint8_t pinDisplayD, uint8_t pinDisplayE, uint8_t pinDisplayF, uint8_t pinDisplayG);

//Función para desplegar el valor al 7 segmentos
void valor(uint8_t valor);



#endif // __DISPLAY7_H__