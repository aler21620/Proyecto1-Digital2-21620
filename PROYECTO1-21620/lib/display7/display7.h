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
extern uint8_t pinA, pinB, pinC, pinD, pinE, pinF, pinG, pindP;  

//**************************************************************************************************
//Prototipos de funciones
//**************************************************************************************************
//Función para configurar el display de 7 segmentos
void configdisplay7(uint8_t pA, uint8_t pB, uint8_t pC, uint8_t pD, uint8_t pE, uint8_t pF, uint8_t pG, uint8_t pdP);

//Función para desplegar el valor al 7 segmentos
void valor(uint8_t valor);

//Función para desplegar el punto
void verpunto(boolean punto);

#endif // __DISPLAY7_H__

