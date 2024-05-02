#include <Adafruit_MCP23X17.h>
#include <DFRobot_I2C_Multiplexer.h>
#include <Wire.h>

//----------------------------------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------MCP23017 registers (everything except direction defaults to 0)---------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
#define IODIRA 0x00  // IO direction  (0 = output, 1 = input (Default))
#define IODIRB 0x01
#define IOPOLA 0x02  // IO polarity   (0 = normal, 1 = inverse)
#define IOPOLB 0x03
#define GPINTENA 0x04  // Interrupt on change (0 = disable, 1 = enable)
#define GPINTENB 0x05
#define DEFVALA 0x06  // Default comparison for interrupt on change (interrupts on opposite)
#define DEFVALB 0x07
#define INTCONA 0x08  // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
#define INTCONB 0x09
#define IOCON 0x0A  // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp
#define IOCON 0x0B  // same as 0x0A
#define GPPUA 0x0C  // Pull-up resistor (0 = disabled, 1 = enabled)
#define GPPUB 0x0D
#define INTFA 0x0E  // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define INTFB 0x0F
#define INTCAPA 0x10  // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define INTCAPB 0x11
#define GPIOA 0x12  // Port value. Write to change, read to obtain value
#define GPIOB 0x13
#define OLLATA 0x14  // Output latch. Write to latch output.
#define OLLATB 0x15

// Définition des broches
const int entreePin = 10; // Broche de l'entrée digitale
const int sortiePin = 6; // Broche de sortie pour l'impulsion
const int sortiePin2 = 4; // Broche de sortie pour l'impulsion
const int motorPin3 = 9; //Broche de sortie pour le Pont en H
const int motorPin4 = 8; //Broche de sortie pour le Pont en H
const int periodePinOut = 5; // Broche de sortie du signal periodique

//Définition du pin d'interruption de la carte de capteurs (8) à définir :
const int pinBoard = 0;

const int pin1 = 0;
const int pin2 = 0;
const int pin3 = 0;
const int pin4 = 0;
const int pin5 = 0;
const int pin6 = 0;
const int pin7 = 0;
const int pin8 = 0;


//Rajout potentiel d'un potentiomètre -> Adresse à définir
const int potPin = A0; 

Adafruit_MCP23X17 mcp;
#define MCP23017_ADDRESS_1 0x21

// Variables pour le moteur
int speed = 255;

// Variables pour la temporisation de l'impulsion et de la temporisation avant impulsion
unsigned long tempsDebutTempo = 0; // Stocke le temps de début de la temporisation avant impulsion
unsigned long tempsDebutTempo2 = 0; // Stocke le temps de début de la temporisation avant impulsion
unsigned long dureeTempo = 2500; // Durée de la temporisation avant impulsion en millisecondes
unsigned long dureeTempo2 = 40; // Durée de la temporisation avant impulsion en millisecondes
unsigned long dureeImpulsion = 1500; // Durée de l'impulsion en millisecondes
unsigned long dureeImpulsion2 = 1500; // Durée de l'impulsion en millisecondes
const unsigned long waitTime = 1000; // Durée minimale requise entre 2 détections
unsigned long time = 0; // Temps au moment d'une detection

bool inter = false;

//Fonction permettant d'activer l'attraction
void active_impulsion_attraction(){
  digitalWrite(sortiePin, HIGH); // Active l'impulsion
  analogWrite(motorPin3, speed);
}
//Fonction permettant de désactiver l'attraction
void desactive_impulsion_attraction(){
  digitalWrite(sortiePin, LOW); // Désactive l'impulsion
  analogWrite(motorPin3, 0);
}

//Fonction permettant d'activer la répulsion
void active_impulsion_repulsion(){
  digitalWrite(sortiePin2, HIGH); // Active l'impulsion
  analogWrite(motorPin4, speed);
}
//Fonction permettant de désactiver la répulsion
void desactive_impulsion_repulsion(){
  digitalWrite(sortiePin2, LOW); // Désactive l'impulsion
  analogWrite(motorPin4, 0);
}

//Lorsqu'une interruption est détectée, on change cette variable afin de permettre de rentrer dans le code et lancer les
//timers seulement si il s'est écoulé plus de 1 seconde depuis la dernière détection afin de détecter le pendule en entrée
void detected(){
  inter = true;        //Variable permettant de rentrer dans la loop
  if(millis()-time>waitTime){
    tempsDebutTempo = millis();
    tempsDebutTempo2 = millis();
  }
  time = millis();      //Timer entre 2 détections
}

//Fonction permettant de recalculer le temps de l'impulsion en fonction du potentiomètre
void calculImpuls(){
  float coef = analogRead(potPin);
  dureeImpulsion = (1500/1023) * coef;
  dureeImpulsion2 = dureeImpulsion;
  dureeTempo = 4000 - dureeImpulsion;
  Serial.print(dureeImpulsion);
  Serial.print(dureeImpulsion2);
  Serial.print(dureeTempo);
}

void expanderWriteBoth(const byte port, const byte reg, const byte data) {
  Wire.beginTransmission(port);
  Wire.write(reg);
  Wire.write(data);  // port A
  Wire.write(data);  // port B
  Wire.endTransmission();
} 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void expanderWriteSingle(const byte port, const byte reg, const byte data) {
  Wire.beginTransmission(port);
  Wire.write(reg);
  Wire.write(data);  // port
  Wire.endTransmission();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int expanderRead(const byte port, const byte reg) {
  Wire.beginTransmission(port);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(port, 1);
  return Wire.read();
} 

void setup() {
  // Configuration des broches
  pinMode(sortiePin, OUTPUT);
  pinMode(sortiePin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  
  mcp.begin_I2C(MCP23017_ADDRESS_1); 
  expanderWriteBoth(MCP23017_ADDRESS_1, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_1, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_1, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_1, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_1, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_1, INTCAPA);
  expanderRead(MCP23017_ADDRESS_1, INTCAPB);  

  //Configuration du potentiomètre 
  pinMode(potPin, INPUT);

  //Configuration des capteurs en mode interruption
  attachInterrupt(digitalPinToInterrupt(pinBoard), detected, FALLING);

  attachInterrupt(digitalPinToInterrupt(pin1), detected, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin2), detected, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin3), detected, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin4), detected, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin5), detected, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin6), detected, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin7), detected, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin8), detected, FALLING);
  

  //Initialisation des tempos avec les potentiomètres
  calculImpuls();

  time = millis();
}

void loop() {
  //On entre dans le code qu'en cas de détection sur un capteur
  if(inter){
    //Réactualisation des variables de temporisation
    calculImpuls();
    // Si la temporisation avant impulsion est terminée
    if (millis() - tempsDebutTempo >= dureeTempo) {
      // Démarrer l'impulsion
      if (millis() - tempsDebutTempo < dureeTempo + dureeImpulsion) {
        active_impulsion_attraction();
      } else {
        desactive_impulsion_attraction();
      }
    }
    if (millis() - tempsDebutTempo2 >= dureeTempo2) {
      // Démarrer l'impulsion
      if (millis() - tempsDebutTempo2 < dureeTempo2 + dureeImpulsion2) {
        active_impulsion_repulsion();
      } else {
        desactive_impulsion_repulsion();
        inter = false;
      }
    }
  }
}
