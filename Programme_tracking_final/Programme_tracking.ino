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
#define NUMPIXELS 360                                                                    // Nombre de LEDs du ruban LEDs
#define interruptPinRouge 2
#define interruptPinVert 3
#define interruptPinBleu 18
#define interruptPinJaune 19
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

#define NUM_LEDS 360
#define R 255
#define G 150
#define B 0
byte tabAdress[8] = {MCP23017_ADDRESS_0, MCP23017_ADDRESS_1, MCP23017_ADDRESS_2, MCP23017_ADDRESS_3, MCP23017_ADDRESS_4, MCP23017_ADDRESS_5, MCP23017_ADDRESS_6, MCP23017_ADDRESS_7};
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
int period = 2800;     //Période du pendule

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
byte Historique[360];
bool first_boot = true;
String color = "";
int green[8];
int red[8];
int blue[8];
int yellow[6];
bool InterruptFound = false;
int InterruptColor = 99;
int index = -1;
int iteration = 0;
int add_led = 0;
bool decompte = false;
const int motorPin1 = 9; //Out 1 relié à In3 sur le pont en H
const int motorPin2 = 8; //Out 2 relié à In4 sur le pont en H
const int bobinePin = 13;
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------Création des Objets-----------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
DFRobot_I2C_Multiplexer I2CMultiplexer(&Wire, 0x70);                            //Création de l'objet Multiplexeur I2C à l'adresse 0x70
Adafruit_MCP23X17 mcp[NUM_MCP23017_CHIPS];                                      //Création de l'objet MCP23017 est du nombres de MCP23017 utilisés
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);  //Création de l'objet Ruban LEDs
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
//  Fonction permettant d'associer une position de LED à la position actuelle du capteur :
int sensorLED() {
  return Actual_Mult * 8 + Actual_Board * 12 + Actual_Sensor;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fonction permettant de lancer un compteur
void wait() {
  waitTime = millis();
  decompte = true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fonction permettant d'associer la couleur à un chiffre
int colorInt(String color){
  int res = 0;
  if(color == "green"){res = 3;}
  if(color == "red"){res = 2;}
  if(color == "blue"){res = 18;}
  if(color == "yellow"){res = 19;}
  return res;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fonction d'allumage des LEDs
int AddLED(int couleur,int capteur,int carte)
{
  int adresse_calculee = 0;
  int Coeff_couleur = 0;
  if(couleur == 2)
  {
    couleur = 0;
  }
  else if (couleur == 3)
  {
    couleur = 1;
  }
  else if(couleur == 18)
  {
    couleur = 2;
  }
  else if(couleur == 19)
  {
    couleur = 3;
  }
  adresse_calculee = (couleur * 8*12) + ((carte)*12) + 12-capteur;
  //Serial.println("");
  Serial.print("couleur:");
  Serial.println(couleur);
  Serial.print("capteur:");
  Serial.println(capteur);
  Serial.print("carte:");
  Serial.println(carte);
  Serial.print("adresse_calculee:");
  //Serial.println(adresse_calculee);
  /*Serial.print("multiplexeur:");
  //Serial.println(multiplexeur);*/

  
  
  return adresse_calculee-1;
}
// Fonction des interruption des pins quarts couleurs
void InterRouge()
{
  InterruptFound = true;
  InterruptColor = 2;
}

void InterVert()
{
  InterruptFound = true;
  InterruptColor = 3;
}


void InterBleu()
{
  InterruptFound = true;
  InterruptColor = 18;
}


void InterJaune()
{
  InterruptFound = true;
  InterruptColor = 19;
}
//Fonction permettant d'allumer la bobine dans le sens + 
void bob_plus(){
  digitalWrite(motorPin1, 180);
  digitalWrite(motorPin2, 0);
  Serial.println("Bobine dans le sens +");
}

//Fonction permettant d'éteindre la bobine
void bobOff(){
  digitalWrite(motorPin1, 0);
  digitalWrite(motorPin2, 0);
  Serial.println("Bobine à l'arrêt");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Création de la fonction snake permettant l'affichage des LEDs sous la forme d'un serpent
void snake(int color, int size) {
  float fade_increment = 0.7;
  int adjustedR = R;
  int adjustedG = G;
  int adjustedB = B;
  int posLed = AddLED(color, Actual_Sensor, Actual_Board);
  strip.setPixelColor(posLed, strip.Color(R, G, B));
  strip.setPixelColor(position(posLed, 180), strip.Color(R, G, B));
  for (int i = 1; i < size; i++) {
    adjustedR = fade_increment * adjustedR;
    adjustedG = fade_increment * adjustedG;
    adjustedB = fade_increment * adjustedB;
    strip.setPixelColor(position(posLed, -i), strip.Color(adjustedR, adjustedG, adjustedB));
    strip.setPixelColor(position(posLed, 180-i), strip.Color(adjustedR, adjustedG, adjustedB));
  }
  strip.show();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Fonction donnant la position que l'on souhaite modifier pour la LED, en fonction de déplacement souhaité
int position(int n, int d) {
  if (n + d > NUMPIXELS - 1) {
    return n + d - NUMPIXELS;
  } else if (n + d < 0) {
    return NUMPIXELS + n + d;
  } else {
    return n + d;
  }
}
//**********************************************************************************************************************************************************//
//********************************************************************* SETUP ******************************************************************************//
//**********************************************************************************************************************************************************//
void setup() {
  delay(100);
  Wire.begin();            //Initialisation de la communication I2C
  Serial.begin(9600);      //Initialisation de la communication Série
  I2CMultiplexer.begin();  //Intitialisation du multiplexeur I2C TCA9548
 
  memset(Historique,0,359);
  TCA9548A(1);
  mcp[0].begin_I2C(MCP23017_ADDRESS_0);  //Initialisation du MCP23017 à l'adresse correspondante
  expanderWriteBoth(MCP23017_ADDRESS_0, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_0, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_0, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_0, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_0, GPINTENB, 0xFF);
  expanderRead(MCP23017_ADDRESS_0, INTCAPA);
  expanderRead(MCP23017_ADDRESS_0, INTCAPB); 
  delay(10);   
  mcp[1].begin_I2C(MCP23017_ADDRESS_1); 
  expanderWriteBoth(MCP23017_ADDRESS_1, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_1, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_1, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_1, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_1, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_1, INTCAPA);
  expanderRead(MCP23017_ADDRESS_1, INTCAPB);  
  delay(10);  
  mcp[2].begin_I2C(MCP23017_ADDRESS_2); 
  expanderWriteBoth(MCP23017_ADDRESS_2, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_2, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_2, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_2, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_2, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_2, INTCAPA);
  expanderRead(MCP23017_ADDRESS_2, INTCAPB);  
  delay(10);  
  mcp[3].begin_I2C(MCP23017_ADDRESS_3);
  expanderWriteBoth(MCP23017_ADDRESS_3, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_3, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_3, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_3, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_3, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_3, INTCAPA);
  expanderRead(MCP23017_ADDRESS_3, INTCAPB); 
  delay(10); 
  mcp[4].begin_I2C(MCP23017_ADDRESS_4);
  expanderWriteBoth(MCP23017_ADDRESS_4, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_4, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_4, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_4, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_4, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_4, INTCAPA);
  expanderRead(MCP23017_ADDRESS_4, INTCAPB); 
  delay(10); 
  mcp[5].begin_I2C(MCP23017_ADDRESS_5);
  expanderWriteBoth(MCP23017_ADDRESS_5, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_5, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_5, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_5, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_5, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_5, INTCAPA);
  expanderRead(MCP23017_ADDRESS_5, INTCAPB); 
  delay(10); 
  mcp[6].begin_I2C(MCP23017_ADDRESS_6); 
  expanderWriteBoth(MCP23017_ADDRESS_6, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_6, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_6, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_6, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_6, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_6, INTCAPA);
  expanderRead(MCP23017_ADDRESS_6, INTCAPB); 
  delay(10); 
  mcp[7].begin_I2C(MCP23017_ADDRESS_7); 
  expanderWriteBoth(MCP23017_ADDRESS_7, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_7, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_7, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_7, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_7, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_7, INTCAPA);
  expanderRead(MCP23017_ADDRESS_7, INTCAPB); 
  delay(10); 
  TCA9548A(2);
  mcp[0].begin_I2C(MCP23017_ADDRESS_0);  //Initialisation du MCP23017 à l'adresse correspondante
  expanderWriteBoth(MCP23017_ADDRESS_0, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_0, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_0, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_0, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_0, GPINTENB, 0xFF);
  expanderRead(MCP23017_ADDRESS_0, INTCAPA);
  expanderRead(MCP23017_ADDRESS_0, INTCAPB);  
  delay(10);   
  mcp[1].begin_I2C(MCP23017_ADDRESS_1); 
  expanderWriteBoth(MCP23017_ADDRESS_1, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_1, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_1, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_1, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_1, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_1, INTCAPA);
  expanderRead(MCP23017_ADDRESS_1, INTCAPB); 
  delay(10);   
  mcp[2].begin_I2C(MCP23017_ADDRESS_2); 
  expanderWriteBoth(MCP23017_ADDRESS_2, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_2, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_2, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_2, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_2, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_2, INTCAPA);
  expanderRead(MCP23017_ADDRESS_2, INTCAPB); 
  delay(10);   
  mcp[3].begin_I2C(MCP23017_ADDRESS_3);
  expanderWriteBoth(MCP23017_ADDRESS_3, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_3, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_3, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_3, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_3, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_3, INTCAPA);
  expanderRead(MCP23017_ADDRESS_3, INTCAPB); 
  delay(10); 
  mcp[4].begin_I2C(MCP23017_ADDRESS_4);
  expanderWriteBoth(MCP23017_ADDRESS_4, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_4, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_4, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_4, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_4, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_4, INTCAPA);
  expanderRead(MCP23017_ADDRESS_4, INTCAPB); 
  delay(10); 
  mcp[5].begin_I2C(MCP23017_ADDRESS_5);
  expanderWriteBoth(MCP23017_ADDRESS_5, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_5, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_5, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_5, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_5, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_5, INTCAPA);
  expanderRead(MCP23017_ADDRESS_5, INTCAPB); 
  delay(10); 
  mcp[6].begin_I2C(MCP23017_ADDRESS_6); 
  expanderWriteBoth(MCP23017_ADDRESS_6, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_6, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_6, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_6, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_6, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_6, INTCAPA);
  expanderRead(MCP23017_ADDRESS_6, INTCAPB); 
  delay(10); 
  mcp[7].begin_I2C(MCP23017_ADDRESS_7); 
  expanderWriteBoth(MCP23017_ADDRESS_7, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_7, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_7, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_7, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_7, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_7, INTCAPA);
  expanderRead(MCP23017_ADDRESS_7, INTCAPB); 
  delay(10); 
  TCA9548A(3);
  //mcp[0].begin_I2C(MCP23017_ADDRESS_0);  //Initialisation du MCP23017 à l'adresse correspondante
  mcp[0].begin_I2C(MCP23017_ADDRESS_0);  //Initialisation du MCP23017 à l'adresse correspondante
  expanderWriteBoth(MCP23017_ADDRESS_0, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_0, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_0, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_0, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_0, GPINTENB, 0xFF);
  expanderRead(MCP23017_ADDRESS_0, INTCAPA);
  expanderRead(MCP23017_ADDRESS_0, INTCAPB);    
  delay(10); 
  mcp[1].begin_I2C(MCP23017_ADDRESS_1); 
  expanderWriteBoth(MCP23017_ADDRESS_1, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_1, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_1, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_1, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_1, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_1, INTCAPA);
  expanderRead(MCP23017_ADDRESS_1, INTCAPB);   
  delay(10); 
  mcp[2].begin_I2C(MCP23017_ADDRESS_2); 
  expanderWriteBoth(MCP23017_ADDRESS_2, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_2, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_2, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_2, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_2, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_2, INTCAPA);
  expanderRead(MCP23017_ADDRESS_2, INTCAPB); 
  delay(10);   
  mcp[3].begin_I2C(MCP23017_ADDRESS_3);
  expanderWriteBoth(MCP23017_ADDRESS_3, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_3, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_3, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_3, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_3, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_3, INTCAPA);
  expanderRead(MCP23017_ADDRESS_3, INTCAPB);
  delay(10);  
  mcp[4].begin_I2C(MCP23017_ADDRESS_4);
  expanderWriteBoth(MCP23017_ADDRESS_4, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_4, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_4, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_4, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_4, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_4, INTCAPA);
  expanderRead(MCP23017_ADDRESS_4, INTCAPB); 
  delay(10); 
  mcp[5].begin_I2C(MCP23017_ADDRESS_5);
  expanderWriteBoth(MCP23017_ADDRESS_5, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_5, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_5, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_5, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_5, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_5, INTCAPA);
  expanderRead(MCP23017_ADDRESS_5, INTCAPB); 
  delay(10); 
  mcp[6].begin_I2C(MCP23017_ADDRESS_6); 
  expanderWriteBoth(MCP23017_ADDRESS_6, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_6, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_6, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_6, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_6, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_6, INTCAPA);
  expanderRead(MCP23017_ADDRESS_6, INTCAPB); 
  delay(10); 
  mcp[7].begin_I2C(MCP23017_ADDRESS_7); 
  expanderWriteBoth(MCP23017_ADDRESS_7, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_7, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_7, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_7, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_7, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_7, INTCAPA);
  expanderRead(MCP23017_ADDRESS_7, INTCAPB); 
  delay(10); 
  TCA9548A(0);
  mcp[0].begin_I2C(MCP23017_ADDRESS_0);  //Initialisation du MCP23017 à l'adresse correspondante
  expanderWriteBoth(MCP23017_ADDRESS_0, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_0, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_0, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_0, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_0, GPINTENB, 0xFF);
  expanderRead(MCP23017_ADDRESS_0, INTCAPA);
  expanderRead(MCP23017_ADDRESS_0, INTCAPB);  
  delay(10);   
  mcp[1].begin_I2C(MCP23017_ADDRESS_1); 
  expanderWriteBoth(MCP23017_ADDRESS_1, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_1, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_1, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_1, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_1, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_1, INTCAPA);
  expanderRead(MCP23017_ADDRESS_1, INTCAPB);   
  delay(10); 
  mcp[2].begin_I2C(MCP23017_ADDRESS_2); 
  expanderWriteBoth(MCP23017_ADDRESS_2, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_2, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_2, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_2, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_2, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_2, INTCAPA);
  expanderRead(MCP23017_ADDRESS_2, INTCAPB);  
  delay(10);  
  mcp[3].begin_I2C(MCP23017_ADDRESS_3);
  expanderWriteBoth(MCP23017_ADDRESS_3, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_3, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_3, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_3, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_3, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_3, INTCAPA);
  expanderRead(MCP23017_ADDRESS_3, INTCAPB); 
  delay(10); 
  mcp[4].begin_I2C(MCP23017_ADDRESS_4);
  expanderWriteBoth(MCP23017_ADDRESS_4, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_4, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_4, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_4, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_4, GPINTENB, 0xFF); 
  expanderRead(MCP23017_ADDRESS_4, INTCAPA);
  expanderRead(MCP23017_ADDRESS_4, INTCAPB); 
  delay(10); 
  mcp[5].begin_I2C(MCP23017_ADDRESS_5);
  expanderWriteBoth(MCP23017_ADDRESS_5, IOCON, 0b01000000); 
  expanderWriteSingle(MCP23017_ADDRESS_5, GPPUA, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_5, GPINTENA, 0xFF);    
  expanderWriteSingle(MCP23017_ADDRESS_5, GPPUB, 0b11111111);  
  expanderWriteSingle(MCP23017_ADDRESS_5, GPINTENB, 0xFF);  
  expanderRead(MCP23017_ADDRESS_5, INTCAPA);
  expanderRead(MCP23017_ADDRESS_5, INTCAPB); 


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
  //chenille de demarrage
  for(int x = 0;x<360;x++)
  {
      strip.clear();
      delay(10);
      strip.setPixelColor(x, 0,  250,  0); //85*3
      delay(10);
      strip.show();
  }
  strip.clear();
  delay(500);
  strip.clear();
  strip.show();
  delay(250);
  pinMode(interruptPinRouge, INPUT);
  pinMode(interruptPinVert, INPUT);
  pinMode(interruptPinBleu, INPUT);
  pinMode(interruptPinJaune, INPUT);

  attachInterrupt(digitalPinToInterrupt(interruptPinRouge), InterRouge, FALLING );
  attachInterrupt(digitalPinToInterrupt(interruptPinVert), InterVert, FALLING );
  attachInterrupt(digitalPinToInterrupt(interruptPinBleu), InterBleu, FALLING );
  attachInterrupt(digitalPinToInterrupt(interruptPinJaune), InterJaune, FALLING );
  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(bobinePin, OUTPUT);
  digitalWrite(bobinePin, HIGH);
}
//**********************************************************************************************************************************************************//
//******************************************************************** LOOP ********************************************************************************//
//**********************************************************************************************************************************************************//
void loop() 
{
  if(first_boot == true)
  {
    first_boot = false;
    for(int k;k<8;k++)
    {
      TCA9548A(0);
      expanderRead(tabAdress[k], INTCAPA);
      expanderRead(tabAdress[k], INTCAPB);
      TCA9548A(1);
      expanderRead(tabAdress[k], INTCAPA);
      expanderRead(tabAdress[k], INTCAPB);
      TCA9548A(2);
      expanderRead(tabAdress[k], INTCAPA);
      expanderRead(tabAdress[k], INTCAPB);
      TCA9548A(3);
      expanderRead(tabAdress[k], INTCAPA);
      expanderRead(tabAdress[k], INTCAPB);
    }
  }

  if(InterruptFound && ( InterruptColor!= 99))
  {
    digitalWrite(bobinePin, LOW);
    switch(InterruptColor)
    {                // Lecture du pin d'interruption permettant de capter un changement d'état dans l'une des cartes
      case 3 :
        color = "green";
        TCA9548A(7);
        value = expanderRead(MCP23017_ADDRESS_7, INTFA);      // Lecture du flag
        delay(50);
        break;
      case 2 : 
        color = "red";
        TCA9548A(7);
        value = expanderRead(MCP23017_ADDRESS_7, INTFB);
        delay(50);
        break;
      case 18 : 
        color = "blue";
        TCA9548A(6);
        value = expanderRead(MCP23017_ADDRESS_7, INTFB);
        delay(50);
        break;
      case 19 : 
        color = "yellow";
        TCA9548A(6);
        value = expanderRead(MCP23017_ADDRESS_7, INTFA);
        delay(50);
        break;
      case 0 :
        break;
    }
    //Serial.println("");
    //Serial.println(color);
    //Serial.println(value);
    /*for(int i =0; i<8; i++){
      Serial.print(value & (1 << i) ? 1 : 0);
    }*/
    //Récupération des informations dans les tableaux de couleurs et de l'index de la carte
    switch(colorInt(color)){
      case 3 : 
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
      case 18 : 
        for (byte i = 0; i < 8; i++) {
          blue[i] = value & (1 << i) ? 1 : 0;
          if(blue[i]==1){index = i;}
        }
        Actual_Mult = 1;
        Actual_Board = index;
        break;
      case 19 : 
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
    
    /*Serial.println("Actual_Board:");
    Serial.println(Actual_Board); //
    Serial.println("Actual_Mult:");
    Serial.println(Actual_Mult); //*/
    TCA9548A(Actual_Mult); 
    int value1 = expanderRead(tabAdress[Actual_Board], INTFA);
    expanderRead(tabAdress[Actual_Board], INTCAPA);
    delay(10);
    int value2 = expanderRead(tabAdress[Actual_Board], INTFB);
    expanderRead(tabAdress[Actual_Board], INTCAPB);
    delay(10);
    
    /*expanderRead(tabAdress[Actual_Board], INTCAPA);
    delay(50);
    expanderRead(tabAdress[Actual_Board], INTCAPB);
    delay(50);*/
  
  
    /*Serial.println();
    Serial.print("Partie A : ");
    for(int i =0; i<6; i++){
      Serial.print(value1 & (1 << i) ? 1 : 0);
    }
    Serial.print("       Partie B : ");
    for(int i =0; i<6; i++){
      Serial.print(value2 & (1 << i) ? 1 : 0);
    }*/
  
    for(int i = 0; i<12; i++)
    {
      if(i<6)
      {
        if(value1 & (1 << i) ? 1 : 0)
        {
          Actual_Sensor = i;
        }
        
      } 
      else if(i>=6)
      {
        if(value2 & (1 << (i-6)) ? 1 : 0)
        {
          Actual_Sensor = i;
        }
      }
    }
    
    if(colorInt(color) != 0)
    {
      /*TCA9548A(6); 
      expanderRead(MCP23017_ADDRESS_7, INTCAPA);
      expanderRead(MCP23017_ADDRESS_7, INTCAPB); 
      TCA9548A(7); 
      expanderRead(MCP23017_ADDRESS_7, INTCAPA);
      expanderRead(MCP23017_ADDRESS_7, INTCAPB); */
      strip.clear();
      delay(100);
      
      /*for(iteration=0;iteration<360;iteration++)
      {
        if(Historique[iteration] >0)
        {
           //Serial.println(iteration,Historique[iteration]);
          Historique[iteration] = Historique[iteration] - 1;
          strip.setPixelColor(iteration, Historique[iteration],  Historique[iteration],  Historique[iteration]); //85*3
        }
      }
      add_led = AddLED(colorInt(color),Actual_Sensor, Actual_Board);
      strip.setPixelColor(add_led, 255, 255, 255); //85*3
       Historique[add_led] = 255;
      strip.show();*/
      snake(colorInt(color), 20);
          for(int k;k<8;k++)
      {
        TCA9548A(0);
        expanderRead(tabAdress[k], INTCAPA);
        expanderRead(tabAdress[k], INTCAPB);
        TCA9548A(1);
        expanderRead(tabAdress[k], INTCAPA);
        expanderRead(tabAdress[k], INTCAPB);
        TCA9548A(2);
        expanderRead(tabAdress[k], INTCAPA);
        expanderRead(tabAdress[k], INTCAPB);
        TCA9548A(3);
        expanderRead(tabAdress[k], INTCAPA);
        expanderRead(tabAdress[k], INTCAPB);
        TCA9548A(6);
        expanderRead(tabAdress[k], INTCAPA);
        expanderRead(tabAdress[k], INTCAPB);
        TCA9548A(7);
        expanderRead(tabAdress[k], INTCAPA);
        expanderRead(tabAdress[k], INTCAPB);
      }
      
    }
    //    Serial.println(InterruptFound);
    //    Serial.println(InterruptColor);
    //    Serial.println(add_led);
    InterruptFound = false;
    InterruptColor = 99;
  }
}
//********************************************************************************************************************************************************//
//*************************************************************** FIN DU PROGRAMME ***********************************************************************//
//********************************************************************************************************************************************************//
