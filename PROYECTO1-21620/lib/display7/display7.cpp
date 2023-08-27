//**************************************************************************************************
//Librerías 
//**************************************************************************************************
#include "display7.h"

uint8_t pinDisplayA, pinDisplayB, pinDisplayC, pinDisplayD, pinDisplayE, pinDisplayF, pinDisplayG;  

//**************************************************************************************************
//Funciones
//**************************************************************************************************
//Función para configurar el display de 7 segmentos
void configdisplay7(uint8_t pinDisplayA, uint8_t pinDisplayB, uint8_t pinDisplayC, uint8_t pinDisplayD, uint8_t pinDisplayE, uint8_t pinDisplayF, uint8_t pinDisplayG){
    //Dar valor a la variable global
    pinDisplayA = pinDisplayA;
    pinDisplayB = pinDisplayB;
    pinDisplayC = pinDisplayC;
    pinDisplayD = pinDisplayD;
    pinDisplayE = pinDisplayE;
    pinDisplayF = pinDisplayF;
    pinDisplayG = pinDisplayG;

    //Configuración de todos los pines como salidas
    pinMode(pinDisplayA, OUTPUT);
    pinMode(pinDisplayB, OUTPUT);
    pinMode(pinDisplayC, OUTPUT);
    pinMode(pinDisplayD, OUTPUT);
    pinMode(pinDisplayE, OUTPUT);
    pinMode(pinDisplayF, OUTPUT);
    pinMode(pinDisplayG, OUTPUT);

    //Configuración para apagar todos los pines
    digitalWrite(pinDisplayA, LOW);
    digitalWrite(pinDisplayB, LOW);
    digitalWrite(pinDisplayC, LOW);
    digitalWrite(pinDisplayD, LOW);
    digitalWrite(pinDisplayE, LOW);
    digitalWrite(pinDisplayF, LOW);
    digitalWrite(pinDisplayG, LOW);
}

//Función para desplegar el valor al 7 segmentos
void valor(uint8_t valor){
    switch (valor) {
    case 0:
        digitalWrite(pinDisplayA, HIGH);
        digitalWrite(pinDisplayB, HIGH);
        digitalWrite(pinDisplayC, HIGH);
        digitalWrite(pinDisplayD, HIGH);
        digitalWrite(pinDisplayE, HIGH);
        digitalWrite(pinDisplayF, HIGH);
        digitalWrite(pinDisplayG, LOW);
        break;
    case 1:
        digitalWrite(pinDisplayA, LOW);
        digitalWrite(pinDisplayB, HIGH);
        digitalWrite(pinDisplayC, HIGH);
        digitalWrite(pinDisplayD, LOW);
        digitalWrite(pinDisplayE, LOW);
        digitalWrite(pinDisplayF, LOW);
        digitalWrite(pinDisplayG, LOW);
        break;
    case 2:
        digitalWrite(pinDisplayA, HIGH);
        digitalWrite(pinDisplayB, HIGH);
        digitalWrite(pinDisplayC, LOW);
        digitalWrite(pinDisplayD, HIGH);
        digitalWrite(pinDisplayE, HIGH);
        digitalWrite(pinDisplayF, LOW);
        digitalWrite(pinDisplayG, HIGH);
        break;
    case 3:
        digitalWrite(pinDisplayA, HIGH);
        digitalWrite(pinDisplayB, HIGH);
        digitalWrite(pinDisplayC, HIGH);
        digitalWrite(pinDisplayD, HIGH);
        digitalWrite(pinDisplayE, LOW);
        digitalWrite(pinDisplayF, LOW);
        digitalWrite(pinDisplayG, HIGH);
        break;
    case 4:
        digitalWrite(pinDisplayA, LOW);
        digitalWrite(pinDisplayB, HIGH);
        digitalWrite(pinDisplayC, HIGH);
        digitalWrite(pinDisplayD, LOW);
        digitalWrite(pinDisplayE, LOW);
        digitalWrite(pinDisplayF, HIGH);
        digitalWrite(pinDisplayG, HIGH);
        break;
    case 5:
        digitalWrite(pinDisplayA, HIGH);
        digitalWrite(pinDisplayB, LOW);
        digitalWrite(pinDisplayC, HIGH);
        digitalWrite(pinDisplayD, HIGH);
        digitalWrite(pinDisplayE, LOW);
        digitalWrite(pinDisplayF, HIGH);
        digitalWrite(pinDisplayG, HIGH);
        break;
    case 6:
        digitalWrite(pinDisplayA, HIGH);
        digitalWrite(pinDisplayB, LOW);
        digitalWrite(pinDisplayC, HIGH);
        digitalWrite(pinDisplayD, HIGH);
        digitalWrite(pinDisplayE, HIGH);
        digitalWrite(pinDisplayF, HIGH);
        digitalWrite(pinDisplayG, HIGH);
        break;
    case 7:
        digitalWrite(pinDisplayA, HIGH);
        digitalWrite(pinDisplayB, HIGH);
        digitalWrite(pinDisplayC, HIGH);
        digitalWrite(pinDisplayD, LOW);
        digitalWrite(pinDisplayE, LOW);
        digitalWrite(pinDisplayF, LOW);
        digitalWrite(pinDisplayG, HIGH);
        break;
    case 8:
        digitalWrite(pinDisplayA, HIGH);
        digitalWrite(pinDisplayB, HIGH);
        digitalWrite(pinDisplayC, HIGH);
        digitalWrite(pinDisplayD, HIGH);
        digitalWrite(pinDisplayE, HIGH);
        digitalWrite(pinDisplayF, HIGH);
        digitalWrite(pinDisplayG, HIGH);
        break;
    case 9:
        digitalWrite(pinDisplayA, HIGH);
        digitalWrite(pinDisplayB, HIGH);
        digitalWrite(pinDisplayC, HIGH);
        digitalWrite(pinDisplayD, HIGH);
        digitalWrite(pinDisplayE, LOW);
        digitalWrite(pinDisplayF, HIGH);
        digitalWrite(pinDisplayG, HIGH);
        break;
    
    default:
    Serial.print("Ese valor no se puede mostrar \n");
        break;
    }
}