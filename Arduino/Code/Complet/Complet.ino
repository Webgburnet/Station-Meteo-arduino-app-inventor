#include <VirtualWire.h>;

// LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

//ANEMOMETRE
#include <PinChangeInt.h>                                     // bibliotheque
#define pinILS 5                                              // bibliotheque
#define pi 3.1415                                             // bibliotheque
#define RayonDesBras 0.07                                     // bibliotheque
byte comptageILSold = 0;                                      // nombre compris entre 0 et 255
const unsigned long dureeAntiRebond = 1;
unsigned long time = 0;                                       // nomnre non négatif de 0 à 4 294 967 295 (2 ^ 32 - 1)(32 bits)
unsigned long timeold;                                        // nomnre non négatif de 0 à 4 294 967 295 (2 ^ 32 - 1)(32 bits)
int nombreTourSec =0;                                         // nombre entier
int nombreTourMin =0;                                         // nombre entier
float vitesseVent(0);                                         // nombre à virgule
float FEtalonage(1);                                          // nombre à virgule
volatile unsigned int comptageILS = 0;                        // une variable utilisée dans une interruption doit être déclarée "volatile"
void interruptILS()                                           // comptage de l'ILS
{
  static unsigned long dateDernierChangement = 0;
  unsigned long date = millis();
 
  if ((date - dateDernierChangement) > dureeAntiRebond) {
    comptageILS++;
    dateDernierChangement = date;
  }
}

//HUMIDITE ET TEMPERATURE
#include <DHT.h>                                              // bibliotheque
#define DHTPIN A2                                             // bibliotheque
#define DHTTYPE DHT11                                         // bibliotheque
DHT dht(DHTPIN, DHTTYPE);

//GIROUETTE
float valeurPot;
int val_D_V = valeurPot;

//LUMINOSITE
#include <math.h>   
#include <LED_Bar.h>                                          // inclure la bibliotheque
LED_Bar mesLeds_pin54(5,4);                                   // pin de la barre LED

//PLUVIOMETRE
volatile int Impulsion=0;
float Precipitation=0;
int val_pluie = Precipitation;
int interval = 1000;      //temps (86400000 ms pour 1 journée, 10000 ms pour 10 sec)
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

//PRESSION ATMOSPHERIQUE
#include <HP20x_dev.h>
#include "Arduino.h"
#include "Wire.h" 
#include <KalmanFilter.h>
unsigned char ret = 0;
KalmanFilter t_filter;                                        //temperature
KalmanFilter p_filter;                                        //pressure 
KalmanFilter a_filter;                                        //altitude 

//QUALITE DE L'AIR
#include <Air_Quality_Sensor.h>
#include "Air_Quality_Sensor.h"
AirQualitySensor sensor(A3);







void setup()
{
//PARTIE CAMILLE
Serial.begin(9600);
vw_setup(2000);

// LCD
lcd.init();                            // initialisation de l'afficheur
  
//ANEMOMETRE
Serial.begin(9600);
pinMode(pinILS, INPUT);
PCintPort::attachInterrupt(pinILS, interruptILS, FALLING); 
timeold = millis();
comptageILSold=0;

//HUMIDITE ET TEMPERATURE
dht.begin();

//PLUVIOMETRE
Serial.begin(9600);
attachInterrupt(0, gestionINT0, RISING);
previousMillis = millis();
pinMode (5, INPUT);
 
//PRESSION ATMOSPHERIQUE
delay(150);
HP20x.begin();
delay(100);

//QUALITE DE L'AIR
Serial.begin(9600);
 
}








void loop()
{    

// LCD
lcd.backlight();

//ANEMOMETRE
int val_V_V = vitesseVent;
  if ((millis() - timeold)> 1000) {                                   // durée d'1 seconde 
    timeold = millis();
    nombreTourSec = (comptageILS - comptageILSold);                   //comptage du nombre de tours par seconde
    nombreTourMin = nombreTourSec*60;
    vitesseVent = pi*RayonDesBras*nombreTourMin*FEtalonage*3.6/30;    // formule pour le calcul de la vitesse du vent
    comptageILSold = comptageILS;                                     // réinitialisation du comptage    
  Serial.print("Vitesse du vent = ");                               // affichage des valeurs
  Serial.print(vitesseVent);
  Serial.print(" km/h\r\n");
      lcd.setCursor(0, 0);                   // Sur la colonne 0 et sur la ligne 0
      lcd.print(vitesseVent);  
      lcd.print("km/h");           
    }

//HUMIDITE ET TEMPERATURE
 int h = dht.readHumidity();
 int tem = dht.readTemperature();
 int val_temp = tem;
 int val_hum = h;
 Serial.print("Humidity: ");
 Serial.print(h);
 Serial.print(" %\t");
 Serial.print("Temperature: ");
 Serial.print(tem);
 Serial.println(" *C");
   lcd.setCursor(11,0);
   lcd.print(h);
   lcd.print("%");
   lcd.setCursor(16,0);
   lcd.print(tem);  
   lcd.print("C");
 

 //GIROUETTE
 valeurPot = analogRead(A1);
 valeurPot = valeurPot*5/1023;0;
 lcd.setCursor(0,2); 
 delay(100);
 Serial.print ( "direction du vent=");
 Serial.println  (valeurPot);
 if(valeurPot > 3.0 && valeurPot <3.4){
 //Serial.println("SUD");}
   lcd.print("SUD");}
 else if(valeurPot >1.9 && valeurPot <2.3 ){
 //Serial.println("NORD");}
   lcd.print("NORD");}
 else if(valeurPot > 3.4 && valeurPot <4.7){
 //Serial.println("OUEST");}
   lcd.print("OUEST");}
 else if(valeurPot > 0.4 && valeurPot <1.50){
 //Serial.println("EST");}
   lcd.print("EST");}

 //LUMINOSITE
 int value = analogRead(A2);
 float resistance = (float)(1023 - value) * 10 / value;
 float Lux1 = 630*pow(resistance,-0.95);
 int val_lum = Lux1;
 Serial.print("Capteur : ");
 Serial.print(value);
 Serial.print("  Lux ");
 Serial.println(Lux1);
   lcd.setCursor(9,1);
   lcd.print("  Lux");
   lcd.print(Lux1);
 delay (1000);

 //PLUVIOMETRE
 currentMillis = millis();
if(previousMillis + interval < currentMillis )  
{
  previousMillis = currentMillis;                      //va déduire le temps écoulé au temps défini (24h) si inférieur à 86400000 ça attend avant envoie, si supérieur à 86400000 il envoie)
  Precipitation=0.307*Impulsion;
  lcd.setCursor (11,2);
  lcd.print("mm:");
  lcd.print(Precipitation);
  //Serial.print("Precipitation (mm)= ");
  //Serial.println(Precipitation); 
  Impulsion=0;       
}
delay(1000);                                           //pause entre les mesures

//PRESSION ATMOSPHERIQUE
 char display[40];
 long Temper = HP20x.ReadTemperature();
 float t = Temper/100.0;
 int val_press = t;   
 long Pressure = HP20x.ReadPressure();
 //Serial.print("Pressure:");
 t = Pressure/100.0;
 Serial.print(t);
 Serial.println("hPa.\n");
   lcd.setCursor(0,3);
   lcd.print(t);
   lcd.print("hPa");

 //QUALITE DE L'AIR
 int quality = sensor.slope();
 int val_qual = quality;
 lcd.setCursor(0,1); 
  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    Serial.println("Forte pollution élevé! Force active.");
      lcd.print("Forte pollution élevé! Force active.");
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION) {
    Serial.println("Forte pollution!");
      lcd.print("Forte pollution!");
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION) {
    Serial.println("Faible pollution!");
      lcd.print("Faible pollution!");
  }
  else if (quality == AirQualitySensor::FRESH_AIR) {
    Serial.println("Air frais.");
      lcd.print("Air frais");
  }
  delay(1000);

//PARTIE CAMILLE
int valeurs[8];
valeurs[0] = val_temp;
valeurs[1] = val_hum;
valeurs[2] = val_pluie;
valeurs[3] = val_press;
valeurs[4] = val_D_V;
valeurs[5] = val_V_V;
valeurs[6] = val_lum;
valeurs[7] = val_qual;
vw_send((byte *) &valeurs, sizeof(valeurs)); // On envoie le message
vw_wait_tx(); // On attend la fin de l'envoi
delay(1000);
}




void gestionINT0()
{
  
++Impulsion;           //comptage  
}


