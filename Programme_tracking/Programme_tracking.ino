//----------------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------Déclaration des Libraries-----------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
#include <DFRobot_I2C_Multiplexer.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include "rgb_lcd.h"
#include <Adafruit_NeoPixel.h>

//----------------------------------------------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------Variables globales---------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
#define NUM_MCP23017_CHIPS 2                                                            //Nombre de MCP23017
#define NUM_HALL_SENSORS_PER_CHIP 16                                                    //Nombre d'entrées disponible du MCP23017
#define MCP23017_ADDRESS_0 0x20                                                         // Adresses des MCP23017
#define MCP23017_ADDRESS_1 0x21                                                         // Adresses des MCP23017
#define MCP23017_ADDRESS_2 0x22                                                         // Adresses des MCP23017
#define MCP23017_ADDRESS_3 0x23                                                         // Adresses des MCP23017
#define MCP23017_ADDRESS_4 0x24                                                         // Adresses des MCP23017
#define MCP23017_ADDRESS_5 0x25                                                         // Adresses des MCP23017
#define MCP23017_ADDRESS_6 0x26                                                         // Adresses des MCP23017
#define MCP23017_ADDRESS_7 0x27                                                         // Adresses des MCP23017
#define INT1PIN 49                                                                      // Interrupt on this Arduino Uno pin.
#define INT2PIN 51                                                                      // Interrupt on this Arduino Uno pin.
#define controlArduinoInt attachInterrupt(digitalPinToInterrupt(INTPIN), isr, FALLING)  // serve interrupt when falling, ie when button is released
#define port1 0x20                                                                      // MCP23017 is on I2C port 0x20, A0 = 0, A1 = 0, A2 = 0
#define ONBOARD_LED 13                                                                  // Sortie de visualisation par LED branché sur la broche 13
#define PIN 6                                                                          // Broche de sortie pour piloter le Din du ruban LEDs
#define PIXEL 360                                                                    // Nombre de LEDs du ruban LEDs
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

#define NUM_LEDS 24
#define R 255
#define G 150
#define B 0
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------Déclaration des Variables--------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
byte value;
byte value2;
const int colorR = 0;
const int colorG = 200;
const int colorB = 127;
bool pinState = false;
bool pinState1 = false;
int SnakeSize = 5;  //Taille de serpent voulue
int waitTime = 0;   //Variable de timer

const int NB_Board = 8;    //Nombre de cartes par bus du multiplexeur
int Actual_Board = 0;      //Numéro de la carte actuel
const int NB_Sensor = 12;  //Nombre de capteurs par carte $
int Actual_Sensor = 0;     //Numéro du capteur actuel
const int NB_Mult = 4;     //Nombre de bus du multiplexeur
int Actual_Mult = 0;       //Numéro du bus actuel

const int B2 = 4275;           // B value of the thermistor
const int R0 = 100000;         // R0 = 100k
const int pinTempSensor = A0;  //Connexion du capteur de température

const int maxTemp = 80;
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------Création des Objets-----------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
DFRobot_I2C_Multiplexer I2CMultiplexer(&Wire, 0x70);                            //Création de l'objet Multiplexeur I2C à l'adresse 0x70
Adafruit_MCP23X17 mcp[NUM_MCP23017_CHIPS];                                      //Création de l'objet MCP23017 est du nombres de MCP23017 utilisés
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL, PIN, NEO_GRB + NEO_KHZ800);  //Création de l'objet Ruban LEDs
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------SOUS-PROGRAMMES-------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
void expanderWriteBoth(const byte port, const byte reg, const byte data) {
  Wire.beginTransmission(port);
  Wire.write(reg);
  Wire.write(data);  // port A
  Wire.write(data);  // port B
  Wire.endTransmission();
}  // end of expanderWrite
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
}  // end of expanderRead
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  //  Serial.print(bus);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Fonction donnant la position que l'on souhaite modifier pour la LED, en fonction de déplacement souhaité
int position(int n, int d) {
  if (n + d > NUM_LEDS - 1) {
    return n + d - NUM_LEDS;
  } else if (n + d < 0) {
    return NUM_LEDS - 1 + n + d + 1;
  } else {
    return n + d;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Création de la fonction snake permettant l'affichage des LEDs sous la forme d'un serpent
void snake(int pos, int size) {
  strip.clear();
  float fade_increment = 0.4;
  int adjustedR = R;
  int adjustedG = G;
  int adjustedB = B;
  strip.setPixelColor(position(pos, 0), strip.Color(R, G, B));
  for (int i = 1; i < size; i++) {
    adjustedR = fade_increment * adjustedR;
    adjustedG = fade_increment * adjustedG;
    adjustedB = fade_increment * adjustedB;
    strip.setPixelColor(position(pos, -i), strip.Color(adjustedR, adjustedG, adjustedB));
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Création de la fonction permettant de trouver la position du capteur :
void sensorPosition(int d) {
  int new_Sensor = Actual_Sensor + d;
  if (0 <= new_Sensor < 12) {  //Cas où le déplacement n'implique pas de changements de cartes
    Actual_Sensor = new_Sensor;
  } else if (new_Sensor < 0) {        //Cas où le déplacement implique un changement de carte
    if (0 <= Actual_Board - 1 < 8) {  //Cas où le changement de carte n'implique pas de changement de bus
      Actual_Board = Actual_Board - 1;
      Actual_Sensor = 12 + new_Sensor;
    } else if (Actual_Board - 1 < 0) {  //Cas où le changement de carte implique un changement de bus
      if (Actual_Mult - 1 == -1) {      //Cas où on revient au dernier bus
        Actual_Mult = 3;
        Actual_Board = 5;
        Actual_Sensor = 12 + new_Sensor;
      } else {  //Cas où on change de bus sans revenir au dernier
        Actual_Mult = Actual_Mult - 1;
        Actual_Board = NB_Board - 1;
        Actual_Sensor = 12 + new_Sensor;
      }
    }
  } else if (new_Sensor >= 12) {   //Cas où le déplacement implique un changement de carte dans le sens +
    if (Actual_Mult < 3) {         //Cas où le bus actuel est entre 0 et 2
      if (Actual_Board + 1 < 8) {  //Cas où il n'y a pas dépassement de bus
        Actual_Board++;
        Actual_Sensor = new_Sensor - 12;
      } else {  //Cas où il y a dépassement de bus
        Actual_Board = 0;
        Actual_Mult++;
        Actual_Sensor = new_Sensor - 12;
      }
    } else if (Actual_Mult == 3) {  //Cas où le bus actuel est 3
      if (Actual_Board + 1 < 6) {   //Cas où il n'y a pas dépassement de bus
        Actual_Board++;
        Actual_Sensor = new_Sensor - 12;
      } else if (Actual_Board + 1 == 6) {  //Cas où il y a dépassement de bus
        Actual_Board = 0;
        Actual_Mult = 0;
        Actual_Sensor = new_Sensor - 12;
      }
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Fonction permettant d'associer une position de LED à la position actuelle du capteur :
int sensorLED() {
  int res = 0;
  if(Actual_Mult==0){
    res = 3*12*8 + 12*Actual_Board + Actual_Sensor;
  } else if(Actual_Mult==1){
    res = 2*12*8 + 12*Actual_Board + Actual_Sensor;
  } else if(Actual_Mult==2){
    res = 1*12*8 + 12*Actual_Board + Actual_Sensor;
  } else if(Actual_Mult==3){
    res = Actual_Board*12 + Actual_Sensor;
  }
  return res;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fonction permettant de tester la détection d'un capteur et la modification du capteur à chercher
int sensorDetection(int sens, int board, int mult) {
  TCA9548A(mult);
  int pinState = mcp[board].digitalRead(sens);
  if (pinState == 0) {
    Actual_Sensor = sens;
    Actual_Board = board;
    Actual_Mult = mult;
  }
  return pinState;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fonction permettant de lancer un compteur
void wait() {
  waitTime = millis();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fonction qui permet de rechercher le capteur actuel en cas d'erreur
void sensorSearch() {
  int ready = 1;
  int pinState;
  while (ready) {
    pinState = sensorDetection(Actual_Sensor, Actual_Board, Actual_Mult);
    if (pinState == 0) {
      ready = 0;
    }
    sensorPosition(1);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fonction permettant d'identifier le PIN d'interruption
int pinSearch(){
  int res = 0;
  for(int i = 8; i<=11; i++){
    if(digitalRead(i)==0){res = i;}
  }
  return res;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fonction permettant d'associer la couleur à un chiffre
int colorInt(String color){
  int res = 0;
  if(color == "green"){res = 1;}
  if(color == "red"){res = 2;}
  if(color == "blue"){res = 3;}
  if(color == "yellow"){res = 4;}
  return res;
}
//**********************************************************************************************************************************************************//
//********************************************************************* SETUP ******************************************************************************//
//**********************************************************************************************************************************************************//
void setup() {
  Wire.begin();            //Initialisation de la communication I2C
  Serial.begin(9600);      //Initialisation de la communication Série
  I2CMultiplexer.begin();  //Intitialisation du multiplexeur I2C TCA9548

  byte tabAdress[8] = {MCP23017_ADDRESS_0, MCP23017_ADDRESS_1, MCP23017_ADDRESS_2, MCP23017_ADDRESS_3, MCP23017_ADDRESS_4, MCP23017_ADDRESS_5, MCP23017_ADDRESS_6, MCP23017_ADDRESS_7};
  
  for(int j =0; j<4; j++){
    for(int i=0; i<8;i++){
      TCA9548A(j);
      if((1<=j<4)||((j==0)&&(i<6))){
        expanderWriteBoth(tabAdress[i], IOCON, 0b01000000); 
        expanderWriteSingle(tabAdress[i], GPPUA, 0b11111111);  
        expanderWriteSingle(tabAdress[i], GPINTENA, 0xFF);    
        expanderWriteSingle(tabAdress[i], GPPUB, 0b11111111);  
        expanderWriteSingle(tabAdress[i], GPINTENB, 0xFF);
        expanderRead(tabAdress[i], INTCAPA);
        expanderRead(tabAdress[i], INTCAPB); 
      }
    }
  }

  TCA9548A(6);
  Adafruit_MCP23X17 mcpTest1;
  mcpTest1.begin_I2C(MCP23017_ADDRESS_7);
  expanderWriteBoth(MCP23017_ADDRESS_7, IOCON, 0b00000000); 
  expanderWriteSingle(MCP23017_ADDRESS_7, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_7, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_7, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_7, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_7, INTCAPA);
  expanderRead(MCP23017_ADDRESS_7, INTCAPB); 
  TCA9548A(7);
  Adafruit_MCP23X17 mcpTest2;
  mcpTest2.begin_I2C(MCP23017_ADDRESS_7);
  expanderWriteBoth(MCP23017_ADDRESS_7, IOCON, 0b00000000); 
  expanderWriteSingle(MCP23017_ADDRESS_7, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_7, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_7, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_7, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_7, INTCAPA);
  expanderRead(MCP23017_ADDRESS_7, INTCAPB);

  strip.begin();                                  //Initialisation du ruban LEDs
  strip.setBrightness(40);
  strip.show();  // Initialisation de toutes les LEDs du ruban LEDs en position "off"

  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
}
//**********************************************************************************************************************************************************//
//******************************************************************** LOOP ********************************************************************************//
//**********************************************************************************************************************************************************//
void loop() {
  /*
  int testG = 9999;     //Associé au vert
  int testR = 9999;     //Associé au rouge
  int testY = 9999;    //Associé au jaune port 0 
  int testB = 9999;    //Associé au bleu
  */

  byte tabAdress[8] = {MCP23017_ADDRESS_0, MCP23017_ADDRESS_1, MCP23017_ADDRESS_2, MCP23017_ADDRESS_3, MCP23017_ADDRESS_4, MCP23017_ADDRESS_5, MCP23017_ADDRESS_6, MCP23017_ADDRESS_7};


  int green[8];
  int red[8];
  int blue[8];
  int yellow[6];
  
  int value = 0;²
  bool test = true;
  int index = -1;
  String color = "";
  switch(pinSearch()){                // Lecture du pin d'interruption permettant de capter un changement d'état dans l'une des cartes
    case 8 :
      color = "green";
      TCA9548A(7);
      value = expanderRead(MCP23017_ADDRESS_7, INTFA);      // Lecture du flag
      expanderRead(MCP23017_ADDRESS_7, INTCAPA);            // Reset de l'interrupt
      break;
    case 9 : 
      color = "red";
      TCA9548A(7);
      value = expanderRead(MCP23017_ADDRESS_7, INTFB);
      expanderRead(MCP23017_ADDRESS_7, INTCAPB); 
      break;
    case 11 : 
      color = "blue";
      TCA9548A(6);
      value = expanderRead(MCP23017_ADDRESS_7, INTFB);
      expanderRead(MCP23017_ADDRESS_7, INTCAPB); 
      break;
    case 10 : 
      color = "yellow";
      TCA9548A(6);
      value = expanderRead(MCP23017_ADDRESS_7, INTFA);
      expanderRead(MCP23017_ADDRESS_7, INTCAPA); 
      break;
    case 0 :
      break;
  }
  Serial.println("");
  Serial.println(color);
  for(int i =0; i<8; i++){
    Serial.print(value & (1 << i) ? 1 : 0);
  }
  //Récupération des informations dans les tableaux de couleurs et de l'index de la carte
  switch(colorInt(color)){
    case 1 : 
      for (byte i = 0; i < 8; i++) {
        green[i] = value & (1 << i) ? 1 : 0;                  //Remplissage du tableau avec les interruptions des cartes
        if(green[i]==1){index = i;}                           //Récupération de l'index de la carte
      }
      Actual_Mult = 2;                                      //Adapter les numéros en fonction du cablage
      Actual_Board = index;
      break;
    case 2 : 
      for (byte i = 0; i < 8; i++) {
        red[i] = value & (1 << i) ? 1 : 0;
        if(red[i]==1){index = i;}
      }
      Actual_Mult = 3;
      Actual_Board = index;
      break;
    case 3 : 
      for (byte i = 0; i < 8; i++) {
        blue[i] = value & (1 << i) ? 1 : 0;
        if(blue[i]==1){index = i;}
      }
      Actual_Mult = 1;
      Actual_Board = index;
      break;
    case 4 : 
      for (byte i = 0; i < 6; i++) {
        yellow[i] = value & (1 << i) ? 1 : 0;
        if(yellow[i]==1){index = i;}
      }
      Actual_Mult = 0;
      Actual_Board = index;
      break;
    case 0 : 
      break;
  }
  Serial.println();
  Serial.println(Actual_Board);
  TCA9548A(Actual_Mult); 
  int value1 = expanderRead(tabAdress[Actual_Board], INTFA);
  expanderRead(tabAdress[Actual_Board], INTCAPA);
  int value2 = expanderRead(tabAdress[Actual_Board], INTFB);
  expanderRead(tabAdress[Actual_Board], INTCAPB);

  Serial.println();
  Serial.print("Partie A : ");
  for(int i =0; i<6; i++){
    Serial.print(value1 & (1 << i) ? 1 : 0);
  }
  Serial.print("       Partie B : ");
  for(int i =0; i<6; i++){
    Serial.print(value2 & (1 << i) ? 1 : 0);
  }
  
  Serial.println();
  int tabSensor[12];
  for(int i = 0; i<12; i++){              //Concaténation des données dans un seul tableau
    if(i<6){
      tabSensor[i] = value2 & (1 << (6-i)) ? 1 : 0;
    } else {
      tabSensor[i] = value1 & (1 << (12-i)) ? 1 : 0;
    }
    Serial.print(tabSensor[i]);
  }

  for(int i = 0; i<12; i++){              //Récupération du numéro de capteur
    if(tabSensor[i]==1){
      Actual_Sensor = i;
    }
  }
  
  snake(sensorLED(), 5);
  delay(5000);
  /*
  int a = analogRead(pinTempSensor);
  float R = 1023.0/a-1.0;
  R = R0*R;
  float temperature = 1.0/(log(R/R0)/B2+1/298.15)-273.15;
  if(temperature<maxTemp){
  }*/
}
//********************************************************************************************************************************************************//
//*************************************************************** FIN DU PROGRAMME ***********************************************************************//
//********************************************************************************************************************************************************//
