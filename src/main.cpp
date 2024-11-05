#include <Arduino.h>
#include <Wire.h>
#include <CustomLCD.h> //https://github.com/Workshopshed/LCD/tree/main
#include <INA236.h>
#include "TinyGPS++.h"

/*
https://github.com/RobTillaart/INA236
https://github.com/teddokano/GPIO_NXP_Arduino
https://github.com/teddokano/I2C_device_Arduino/tree/main

//https://www.mouser.fr/datasheet/2/468/RPMA_4_5_OF-2634336.pdf
*/

int incomingByte = 0; // for incoming serial data

// https://www.nxp.com/docs/en/data-sheet/PCAL6416A.pdf
#define PCAL6416_ADDR 0x20  // Adresse I2C du PCAL6416

#define PCAL6416_CONFIG 0x07     // Registre de configuration des broches PCAL6416
#define PCAL6416_OUTPUT 0x03     // Registre des données de sortie PCAL6416

// https://www.digikey.fr/htmldatasheets/production/2750891/0/0/1/pcal6408a.html#pf8
#define PCAL6408_ADDR 0x21  // Adresse I2C du PCAL6408

#define PCAL6408_CONFIG 0x03     // Registre de configuration des broches PCAL6408
#define PCAL6408_OUTPUT 0x01     // Registre des données de sortie PCAL6408
#define PCAL6408_INPUT 0x00     // Registre des données d'entree PCAL6408

// https://newhavendisplay.com/content/specs/NHD-C0220BiZ-FSW-FBW-3V3M.pdf
#define NHD_COG 0x3C  // Adresse I2C de l'écran 0x3C

#define CTS1ADD 0x40
#define CTS2ADD 0x41
#define CTPVADD 0x42
#define CTBATTADD 0x43  

//#define SET_DDRAM_CMD 0x80 // Set DDRAM address command
//#define DISP_CMD 0x0  // Command for the display

#define NUM_STATES_IHM 7 // Nombre d'etats possibles de l'IHM

#define PHOTOPIN 36  // Broche pour la photodiode BPW 21
#define PHOTOARRAYSIZE 5 // Nombre de mesures de la moyenne glissante
#define PHOTOWAIT 250 // Temps d'attente entre chaque mesure (ms)

#define S1PIN 27 // Broche Sortie 1
#define S2PIN 26 // Broche Sortie 2
#define PVPIN 32 // Broche Sortie 2

#define S1CH 0 // Canal PWM S1
#define S2CH 1 // Canal PWM S2


float avlux;
int currentLuxIndex = 0;


typedef enum {
  S1, S2, BATT, PV, LUX, GPSS, TIME
} ScreenState;


/**
Rafraichir le LCD en fonction de l'etat demande

@param screenState L'etat a ecrire

@return None
*/
void Update_Display(ScreenState screenState);

/**
Ecrit la valeur au registre donné a l'addresse I2C donnée

@param i2c_address addresse I2C
@param register_adress addresse du registre au sein du peripherique I2C
@param value valeur a écrire

@return None
*/
void I2CWrite(int8_t i2c_address, int8_t register_address, int8_t value);

/**
Lancer et recevoir une requete I2C

@param address addresse I2C
@param register_adress addresse du registre au sein du peripherique I2C

@return contenu du registre
*/
char I2CRead(int8_t address, int8_t register_adress);

/**
Obtenir l'etat d'un bouton de l'IHM

@param bp_num numero du bouton a lire 0,1,2

@return None
*/
int ReadBP(int bp_num);

/**
Taches Lux-metre, effectue une moyenne glissante sur les PHOTOARRAYSIZE dernieres mesures 
Mesures efffectuees toutes les PHOTOWAIT
*/
void LuxMeter(void *parameters);

/**
Taches de gestion de l'IHM
*/
void IHM(void *parameters);

/**
Reception et execution des commandes UART
*/
void reception(char ch);

/**
Initialisation capteurs Courant/Tension
*/
void InitCapteursCT();


void I2CScan();


ST7036 LCD(2, 20, NHD_COG);

HardwareSerial GPS_Serial(2);  // Utilise UART2

//GPS GPSINST(GPS_Serial);
TinyGPSPlus Gps;

INA236 CTS1(CTS1ADD, &Wire);
INA236 CTS2(CTS2ADD, &Wire);
INA236 CTPV(CTPVADD, &Wire);
INA236 CTBATT(CTBATTADD, &Wire);


void setup() {

  Serial.begin(115200);



  pinMode(S1PIN, OUTPUT);
  pinMode(S2PIN, OUTPUT);

  pinMode(PVPIN, OUTPUT);


  //ledcSetup(S1CH, 100, 10); // PWM
  //ledcSetup(S2CH, 100, 10); // PWM

  //ledcAttachPin(S1PIN, S1CH); // PWM
  //ledcAttachPin(S2PIN, S2CH); // PWM


  //InitExtender(PCAL6408_ADDR, PCAL6408_CONFIG, 0x0E, PCAL6408_OUTPUT);

  pinMode(0, OUTPUT);
  digitalWrite(0,1);

  //InitExtenders();
  //InitExtender(PCAL6408_ADDR, PCAL6408_CONFIG, 0x0E, PCAL6408_OUTPUT);
}

void loop() {
  delay(500);
}

void LuxMeter(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  float luxValues[PHOTOARRAYSIZE] = { 0 };

  while(1)
  {
    float sensorValue = analogRead(PHOTOPIN);
    float ve = (sensorValue / 4095.0) * 3.3;  // tension aux broche de l'ESP
    float vc = (ve / 3.3) * 5;  // valeur avant le pont diviseur
    float courant = vc / (3300000);           // Courant de sortie du capteur
    float lx = courant / 9.0E-9;               // Conversion du courant en lux

    luxValues[currentLuxIndex] = lx;  // Stocker la mesure de lux dans le tableau

    currentLuxIndex++;
    currentLuxIndex %= PHOTOARRAYSIZE;


    float sum = 0;
    /*for (int i = 3; i < 7; i++) {  // Prendre les indices 3 à 6 (les 4 valeurs du milieu)
        sum += luxValues[i];
    }*/

    for (int i = 0; i < PHOTOARRAYSIZE; i++) {  // Prendre les indices 3 à 6 (les 4 valeurs du milieu)
      sum += luxValues[i];
    }

    avlux = sum / PHOTOARRAYSIZE;  // Calculer la moyenne des lux

    vTaskDelayUntil(&xLastWakeTime, PHOTOWAIT)
  }
}

void IHM(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  ScreenState screenState = S1; // Current Interface State
  bool screenChange;
  
  while(1)
  {
    if (!screenChange)
    {
      if (ReadBP(1))
      {
        screenState = (ScreenState)0;
        screenChange = true;
      }
      else 
      {
        if (ReadBP(2))
        {
          screenState = (ScreenState)(((int)screenState + NUM_STATES_IHM - 1) % (NUM_STATES_IHM));
          screenChange = true;
        }

        if (ReadBP(3))
        {
          screenState = (ScreenState)(((int)screenState + 1) % (NUM_STATES_IHM));
          screenChange = true;
        }
      }
    }
    else if (!ReadBP(1) && !ReadBP(2) && !ReadBP(3))
      screenChange = false;

    Update_Display(screenState);

    vTaskDelayUntil(&xLastWakeTime, 50)
  }
}

void Update_Display(ScreenState screenState)
{
  switch(screenState)
  {
    case S1:
    {
      char line1[20] = {};
      sprintf(line1, "S1 V = %2.2f V", CTS1.getBusVoltage());
      LCD.setCursor(0,0);
      LCD.lcd_print((uint8_t *)line1, 20);

      char line2[20] = {};
      sprintf(line2, "   I = %2.2f A", CTS1.getCurrent());
      LCD.setCursor(1,0);
      LCD.lcd_print((uint8_t *)line2, 20);
      break;
    }
    case S2:
    {
      char line1[20] = {};
      sprintf(line1, "S2 V = %2.2f V", CTS2.getBusVoltage());
      LCD.setCursor(0,0);
      LCD.lcd_print((uint8_t *)line1, 20);

      char line2[20] = {};
      sprintf(line2, "   I = %2.2f A", CTS2.getCurrent());
      LCD.setCursor(1,0);
      LCD.lcd_print((uint8_t *)line2, 20);
      break;
    }
    case BATT:
    {
      char line1[20] = {};
      sprintf(line1, "BATT V = %2.2f V", CTBATT.getBusVoltage());
      LCD.setCursor(0,0);
      LCD.lcd_print((uint8_t *)line1, 20);

      char line2[20] = {};
      sprintf(line2, "     I = %2.2f A", CTBATT.getCurrent());
      LCD.setCursor(1,0);
      LCD.lcd_print((uint8_t *)line2, 20);
      break;
    }
    case PV:
    {
      char line1[20] = {};
      sprintf(line1, "PV V = %2.2f V", CTPV.getBusVoltage());
      LCD.setCursor(0,0);
      LCD.lcd_print((uint8_t *)line1, 20);

      char line2[20] = {};
      sprintf(line2, "   I = %2.2f A", CTPV.getCurrent());
      LCD.setCursor(1,0);
      LCD.lcd_print((uint8_t *)line2, 20);
      break;
    } 
    case LUX:
    {
      char line1[20] = {};
      sprintf(line1, "LUX = %2.2f LX", avlux);
      LCD.setCursor(0,0);
      LCD.lcd_print((uint8_t *)line1, 20);

      char line2[20] = {};
      //sprintf(line2, "   I = %2.2f", 10);
      LCD.setCursor(1,0);
      LCD.lcd_print((uint8_t *)line2, 20);
      break;
    } 
    case GPSS:
    {
      while (GPS_Serial.available()) {
        Gps.encode(GPS_Serial.read());
      }

      char line1[20] = {};

      if (Gps.location.isValid())
        sprintf(line1, "GPS LAT = %2.2f", Gps.location.lat());
      else
        sprintf(line1, "GPS LAT Invalid");

      LCD.setCursor(0,0);
      LCD.lcd_print((uint8_t *)line1, 20);

      char line2[20] = {};
      
      if (Gps.location.isValid())
        sprintf(line2, "    LON = %2.2f", Gps.location.lng());
      else
        sprintf(line2, "    LON Invalid");

      LCD.setCursor(1,0);
      LCD.lcd_print((uint8_t *)line2, 20);
      break;
    } 
    case TIME:
    {
      
      while (GPS_Serial.available()) {
        Gps.encode(GPS_Serial.read());
      }
  
      char line1[20] = {};

      if (Gps.location.isValid())
        sprintf(line1, "Time = %2d:%2d:%2d", Gps.time.hour(), Gps.time.minute(), Gps.time.second());
      else
        sprintf(line1, "Time Invalid");

      LCD.setCursor(0,0);
      LCD.lcd_print((uint8_t *)line1, 20);

      char line2[20] = {};

      if (Gps.location.isValid())
        sprintf(line2, "Date = %2d/%2d/%4d", Gps.date.day(), Gps.date.month(), Gps.date.year());
      else
        sprintf(line2, "Date Invalid");

      LCD.setCursor(1,0);
      LCD.lcd_print((uint8_t *)line2, 20);
      break;
    } 
  }
}

void I2CWrite(int8_t i2c_address, int8_t register_address, int8_t value)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(register_address);
  Wire.write(value);
  Wire.endTransmission();
}

char I2CRead(int8_t address, int8_t register_adress)
{
  Wire.beginTransmission(address); // Start communication with I2C device
  Wire.write(register_adress); // Send the register address you want to read from (if needed)
  Wire.endTransmission(false); // End transmission but keep the connection active*/
  
  char data; // Read the first byte
  
  Wire.requestFrom(address, 1); // Request 2 bytes from the device
  
  while (Wire.available()) { // Check if data is available
    data = Wire.read(); // Read the first byte
    //Serial.print("data: ");
    //Serial.println(data, HEX); // Print data in hexadecimal
  }

  return data;
}

int ReadBP(int bp_num)
{
  return (I2CRead(PCAL6408_ADDR, PCAL6408_INPUT) >> bp_num) & 0x01;
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}

void reception(char ch)
{
  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10)) // Si retour à la ligne
  { 
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "S1")
    {
      //ledcWrite(S1CH, (int)((1 << 10) * valeur.toFloat() / 100.0));
      digitalWrite(S1PIN, valeur.toInt());
    }

    if (commande == "S2")
    {
      //ledcWrite(S2CH, (int)((1 << 10) * valeur.toFloat() / 100.0));
      digitalWrite(S2PIN, valeur.toInt());
    }

    if (commande == "PV")
    {
      //ledcWrite(S2CH, (int)((1 << 10) * valeur.toFloat() / 100.0));
      digitalWrite(PVPIN, valeur.toInt());
    }

    if (commande == "INIT")
    {
      //test = valeur.toFloat(); // récupérer la valeur
      // Action!

      Wire.begin();

      I2CWrite(PCAL6416_ADDR, PCAL6416_CONFIG, 0xFD);  
      I2CWrite(PCAL6416_ADDR, PCAL6416_OUTPUT, 0x02);

      I2CWrite(PCAL6408_ADDR, PCAL6408_CONFIG, 0xFF); // Configurer les broches comme entrees

      InitCapteursCT();

      LCD.init();
      LCD.setBacklight(255);

      GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);  // Rx sur GPIO 16, Tx sur GPIO 17

      xTaskCreate(LuxMeter, "LuxMeter", 8192, NULL, 10, NULL);

      xTaskCreate(IHM, "Display", 8192, NULL, 10, NULL);
    }

    chaine = "";
  }
  else
  {
    chaine += ch; // ajouter le caractère à la chaine
  }
}

void InitCapteursCT()
{
  // Initialiser les capteurs INA236
  if (!CTS1.begin()) {
    Serial.println("Impossible de trouver INA236 Sortie 1 à l'adresse 0x40");
    while (1);
  }
  if (!CTS2.begin()) {
    Serial.println("Impossible de trouver INA236 Sortie 2 à l'adresse 0x41");
    while (1);
  }
  if (!CTPV.begin()) {
    Serial.println("Impossible de trouver INA236 PhotoVoltaique à l'adresse 0x42");
    while (1);
  }
  if (!CTBATT.begin()) {
    Serial.println("Impossible de trouver INA236 Batterie à l'adresse 0x43");
    while (1);
  }

  CTS1.setMaxCurrentShunt(16, 0.005);
  CTS2.setMaxCurrentShunt(16, 0.005);
  CTPV.setMaxCurrentShunt(16, 0.005);
  CTBATT.setMaxCurrentShunt(16, 0.005);
}
