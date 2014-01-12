/* 
 ArduiLog v.01 http://rc-lab.fr
 
 License: http://creativecommons.org/licenses/by-nc/4.0/
 
 Code source pour les utilisateurs de Megunlink - avec les commentaires en francais
 
 Tutorial{ Lien court: http://wp.me/p2z2tt-7u
 http://rc-lab.fr/arduilog-v-01-appareil-dacquisition-48-voies-analogiques
 
 code écrit par Shanmugathas Vigneswaran-Ismaël.
 */


/*
 Merci à Mayhew Labs : http://mayhewlabs.com/arduino-mux-shield
 pour le code concernant le multiplexer.
 
 Merci à Maurice Ribble
 http://www.glacialwanderer.com/hobbyrobotics
 pour le code du module temps réel DS1307.
 
 Merci à mon ami Edouard Chalouhb : http://www.edouard-chalhoub.com/
 pour m'avoir fourni un laboratoire, afin de tester l'appareil 
 dans une application concrète.
 */


#include <Wire.h>
#include <SerialLCD.h>
#include <SoftwareSerial.h>
#include <math.h>

// Utilise la libraire SoftwareSerial pour l'affichage de l'écran LCD
SerialLCD slcd(11,12);

// Définition de l'adresse du module temps réel
#define DS1307_I2C_ADDRESS 0x68

// Définition des connecteurs digitaux controlants le multiplexeur
#define CONTROL0 5    
#define CONTROL1 4
#define CONTROL2 3
#define CONTROL3 2

int analog_input[48];  // valeurs entrées analogiques (0 à 1023)
int an_disp_count = 0; // integer, compteur pour affichage des valeurs sur l'écran LCD
char* input_unit[48]; // unité de vos entrées analogiques pour MegunoLink et l'écran LCD
char* input_label[48]; // commentaires de vos entrées analogiques pour MegunoLink

// Uniquement les valeurs ci-dessous sont envoyées vers MegunoLink et l'écran LCD
float analog_mapped[48]; // Utilisez vos équations transformer les valeurs analogiques en valeurs physiques


// Sauvegarde la valeur de la date et du temps à partir module temps réel
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;


void setup()
{
  //Ouvreture du port série à 500000 bps
  Serial.begin(500000);

  //Ouverture de la série LCD
  slcd.begin();

  //Allumage du rétro-éclairage
  slcd.backlight();

  // Ouverture de la communication Wire pour le module RTC
  Wire.begin();

  // Configuration des connecteurs en mode sortie
  pinMode(CONTROL0, OUTPUT);
  pinMode(CONTROL1, OUTPUT);
  pinMode(CONTROL2, OUTPUT);
  pinMode(CONTROL3, OUTPUT);

  /* 
   Utiliser les valeurs ci-dessous vous voulez configurer le module temps réel.
   Une fois configuré, laisser cette partie du code en commentaire
   second = 40;
   minute = 54;
   hour = 14;
   dayOfWeek = 5;
   dayOfMonth = 20;
   month = 12;
   year = 13;
   setDateDs1307(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
   */

  /* 
   Configurer les unités (Pa,C,V,mA,A ....) des entrées analogiques, 
   pour l'affichage de MegunoLink et de l'écran LCD
   ATTENTION n'utilisez pas de caractères spéciaux (°, %, $ ù etc....)
   */
  input_unit[0]="";   //unité de analog1 
  input_unit[1]="";   //unité de analog2 
  input_unit[2]="";   //unité de analog3
  input_unit[3]="";   //unité de analog4 
  input_unit[4]="";   //unité de analog5 
  input_unit[5]="";   //unité de analog6 
  input_unit[6]="";   //unité de analog7 
  input_unit[7]="";   //unité de analog8 
  input_unit[8]="";   //unité de analog9 
  input_unit[9]="";   //unité de analog10 
  input_unit[10]="";   //unité de analog11    
  input_unit[11]="";   //unité de analog12 
  input_unit[12]="";   //unité de analog13 
  input_unit[13]="";   //unité de analog14 
  input_unit[14]="";   //unité de analog15 
  input_unit[15]="";   //unité de analog16 
  input_unit[16]="";   //unité de analog17 
  input_unit[17]="";   //unité de analog18 
  input_unit[18]="";   //unité de analog19 
  input_unit[19]="";   //unité de analog20 
  input_unit[20]="";   //unité de analog21 
  input_unit[21]="";   //unité de analog22 
  input_unit[22]="";   //unité de analog23 
  input_unit[23]="";   //unité de analog24 
  input_unit[24]="";   //unité de analog25 
  input_unit[25]="";   //unité de analog26 
  input_unit[26]="";   //unité de analog27 
  input_unit[27]="";   //unité de analog28 
  input_unit[28]="";   //unité de analog29 
  input_unit[29]="";   //unité de analog30 
  input_unit[30]="";   //unité de analog31 
  input_unit[31]="";   //unité de analog32 
  input_unit[32]="";   //unité de analog33 
  input_unit[33]="";   //unité de analog34 
  input_unit[34]="";   //unité de analog35 
  input_unit[35]="";   //unité de analog36 
  input_unit[36]="";   //unité de analog37 
  input_unit[37]="";   //unité de analog38 
  input_unit[38]="";   //unité de analog39 
  input_unit[39]="";   //unité de analog40 
  input_unit[40]="";   //unité de analog41 
  input_unit[41]="";   //unité de analog42 
  input_unit[42]="";   //unité de analog43 
  input_unit[43]="";   //unité de analog44 
  input_unit[44]="";   //unité de analog45 
  input_unit[45]="";   //unité de analog46 
  input_unit[46]="";   //unité de analog47 
  input_unit[47]="";   //unité de analog48 



  /*
   Configurer les commentaires pour l'affichage sur MegunoLink
   Exemple : T chambre 1, T chambre 2, P aspiration ...
   ATTENTION n'utilisez pas de caractères spéciaux (°, %, $ ù etc....)
   */
  input_label[0]="";   // commentaire de analog1 
  input_label[1]="";   // commentaire de analog2 
  input_label[2]="";   // commentaire de analog3 
  input_label[3]="";   // commentaire de analog4 
  input_label[4]="";   // commentaire de analog5 
  input_label[5]="";   // commentaire de analog6 
  input_label[6]="";   // commentaire de analog7 
  input_label[7]="";   // commentaire de analog8 
  input_label[8]="";   // commentaire de analog9 
  input_label[9]="";   // commentaire de analog10 
  input_label[10]="";   // commentaire de analog11    
  input_label[11]="";   // commentaire de analog12 
  input_label[12]="";   // commentaire de analog13 
  input_label[13]="";   // commentaire de analog14 
  input_label[14]="";   // commentaire de analog15 
  input_label[15]="";   // commentaire de analog16 
  input_label[16]="";   // commentaire de analog17 
  input_label[17]="";   // commentaire de analog18 
  input_label[18]="";   // commentaire de analog19 
  input_label[19]="";   // commentaire de analog20 
  input_label[20]="";   // commentaire de analog21 
  input_label[21]="";   // commentaire de analog22 
  input_label[22]="";   // commentaire de analog23 
  input_label[23]="";   // commentaire de analog24 
  input_label[24]="";   // commentaire de analog25 
  input_label[25]="";   // commentaire de analog26 
  input_label[26]="";   // commentaire de analog27 
  input_label[27]="";   // commentaire de analog28 
  input_label[28]="";   // commentaire de analog29 
  input_label[29]="";   // commentaire de analog30 
  input_label[30]="";   // commentaire de analog31 
  input_label[31]="";   // commentaire de analog32 
  input_label[32]="";   // commentaire de analog33 
  input_label[33]="";   // commentaire de analog34 
  input_label[34]="";   // commentaire de analog35 
  input_label[35]="";   // commentaire de  analog36 
  input_label[36]="";   // commentaire de  analog37 
  input_label[37]="";   // commentaire de  analog38 
  input_label[38]="";   // commentaire de  analog39 
  input_label[39]="";   // commentaire de  analog40 
  input_label[40]="";   // commentaire de  analog41 
  input_label[41]="";   // commentaire de  analog42 
  input_label[42]="";   // commentaire de  analog43 
  input_label[43]="";   // commentaire de  analog44 
  input_label[44]="";   // commentaire de  analog45 
  input_label[45]="";   // commentaire de  analog46 
  input_label[46]="";   // commentaire de  analog47 
  input_label[47]="";   // commentaire de  analog48 




}



void loop()
{
  get_time(); // obtenir le temps à partir du module temps réel 
  read_analogs(); // lecture des valeurs analogiques des trois multiplexeurs
  analogs_map();  // utiliser cette fonction pour transfer la valeur numérique lue en valeur physique
  megunolink_datalog(); // transmission vers Megunolink pour l'exporation de données
  megunolink_timeplot(); //  transmission vers MegunLink pour l'affichage de graphique en temps réel
  megunolink_table(); // transmission vers MegunLink pour l'affichage du tableau de valeur
  lcd_print(); // affichage sur l'écran LCD
  delay(1000); // pause du programme pendant 1 seconde. Ne pas descendre en dessous de 500ms.
}


void get_time(){
  // Obtenir le temps à partir du module temps réel, vous n'avez pas besoin de modifier cette fonction
  getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
}

void read_analogs(){
  // Lecture des valeurs analogiques des trois multiplexeurs, vous n'avez pas besoin de modifier cette fonction

  // Cette boucle est utilisé pour scruter et sauvegarder les 16 entrées analogiques du premier multiplexeur
  for (int i=0; i<16; i++)
  {
    //Les 4 commandes suivantes, configurent les connecteurs controleurs pour pour sélectionner l'entrée souhaitée
    //Regarder sur Arduino la référnce "Bitwise And" : http://www.arduino.cc/en/Reference/BitwiseAnd
    //Regarder sur Aruino  la référence "Bitshift": http://www.arduino.cc/en/Reference/Bitshift
    digitalWrite(CONTROL0, (i&15)>>3); 
    digitalWrite(CONTROL1, (i&7)>>2);  
    digitalWrite(CONTROL2, (i&3)>>1);  
    digitalWrite(CONTROL3, (i&1));     

    // Lecture et sauvegarde de l'entrée analogique dans la matrice
    analog_input[i] = analogRead(0);
  }

  // Cette boucle est utilisé pour scruter et sauvegarder les 16 entrées analogiques du second multiplexeur
  for (int i=16; i<32; i++)
  {
    digitalWrite(CONTROL0, (i&15)>>3); 
    digitalWrite(CONTROL1, (i&7)>>2);  
    digitalWrite(CONTROL2, (i&3)>>1);  
    digitalWrite(CONTROL3, (i&1));     

    // Lecture et sauvegarde de l'entrée analogique dans la matrice
    analog_input[i] = analogRead(1);
  }

  // Cette boucle est utilisé pour scruter et sauvegarder les 16 entrées analogiques du troisième multiplexeur
  for (int i=32; i<48; i++)
  {
    digitalWrite(CONTROL0, (i&15)>>3); 
    digitalWrite(CONTROL1, (i&7)>>2);  
    digitalWrite(CONTROL2, (i&3)>>1);  
    digitalWrite(CONTROL3, (i&1));    

    // Lecture et sauvegarde de l'entrée analogique dans la matrice 
    analog_input[i] = analogRead(2);
  }    
}

void analogs_map(){
  // Utilisez cette fonction pour convertir les valeurs numériques lues en valeur phsique

  // Conversion en tension, convertie toutes les voies de [0;1023] à [0;30]V
  for (int i=0; i<48; i++){
    analog_mapped[i]= float_map(analog_input[i], 0, 1023, 0, 30); 
  }

  /*    
   Utilisez ce code pour obtenir une valeur de résistance,
   si vous utiliser une thermistance ou un capteur photo-sensible etc...
   Ce code converti la tension en valeur de résistance.
   
   float resistor_value = ((-6*(analog_mapped[i] -5))/(analog_mapped[i] ))*1000;
   */


  /*    
   Utilisez ce code si vous utiliser un capteur de température TMP36,
   pour convertir la tension mesurée en T(°C)
   
   float temperature_c = ( analog_mapped[i]*1000 - 500) / 10 ;
   */



}


void megunolink_datalog(){
  // Envoie les "messages" vers MegunoLink,pour exporter les fichiers en format *.txt (pour etre converti au format *.csv)

  Serial.print("{MESSAGE|data|");
  Serial.print(hour, DEC);
  Serial.print(":");
  Serial.print(minute, DEC);
  Serial.print(":");
  Serial.print(second, DEC);
  Serial.print(";");

  for (int i=0; i<48; i++)
  {
    Serial.print(analog_mapped[i]);
    Serial.print(";");
  }

  Serial.println("}");
}

void megunolink_timeplot(){
  // Envoie les valeurs vers MegunoLink pour tracer les graphiques en temps réel

  for (int i=0; i<48; i++){
    Serial.print("{TIMEPLOT:");
    Serial.print("Analog"); // nom du canal
    Serial.print(i+1);        // numéro du canal
    Serial.print("|data|");
    Serial.print(input_unit[i]);
    Serial.print(":");
    Serial.print("k_n");    //  configuration du graphique
    Serial.print("|T|");
    Serial.print(analog_mapped[i],3);  // valeur du canal
    Serial.println("}");
  }
}


void megunolink_table(){  
  // Envoie les valeurs vers MegunoLink pour le tableau synthétisant les données

  for (int i=0; i<48; i++){
    Serial.print("{TABLE");
    Serial.print("|SET|");
    Serial.print("Analog");
    Serial.print(i+1);
    Serial.print("|");
    Serial.print(analog_mapped[i],3);
    Serial.print("|");
    Serial.print(input_unit[i]);
    Serial.print("  //  ");
    Serial.print(input_label[i]);
    Serial.println("}");
  }

}

void lcd_print(){
  // Affichage sur l'écran LCD

  slcd.clear();
  slcd.setCursor(0,0);
  slcd.print("Heure:");

  if (hour<0b1010) {
    slcd.setCursor(7,0);
    slcd.print("0");  
    slcd.setCursor(8,0);
    slcd.print(hour, DEC);
  }
  else{
    slcd.setCursor(7,0);
    slcd.print(hour, DEC);
  }

  slcd.setCursor(9,0);
  slcd.print(":");

  if (minute<0b1010) {
    slcd.setCursor(10,0);
    slcd.print("0");
    slcd.setCursor(11,0);
    slcd.print(minute, DEC);
  }
  else{
    slcd.setCursor(10,0);
    slcd.print(minute, DEC);
  }

  slcd.setCursor(12,0);
  slcd.print(":");

  if (second<0b1010) {
    slcd.setCursor(13,0);
    slcd.print("0");
    slcd.setCursor(14,0);
    slcd.print(second, DEC);
  }
  else{
    slcd.setCursor(13,0);
    slcd.print(second, DEC);
  }

  an_disp_count=an_disp_count+1;

  if (an_disp_count == 1){
    slcd.setCursor(0,1);
    slcd.print("A1:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 2){
    slcd.setCursor(0,1);
    slcd.print("A2:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 3){
    slcd.setCursor(0,1);
    slcd.print("A3:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 4){
    slcd.setCursor(0,1);
    slcd.print("A4:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 5){
    slcd.setCursor(0,1);
    slcd.print("A5:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 6){
    slcd.setCursor(0,1);
    slcd.print("A6:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 7){
    slcd.setCursor(0,1);
    slcd.print("A7:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 8){
    slcd.setCursor(0,1);
    slcd.print("A8:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 9){
    slcd.setCursor(0,1);
    slcd.print("A9:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 10){
    slcd.setCursor(0,1);
    slcd.print("A10:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 11){
    slcd.setCursor(0,1);
    slcd.print("A11:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 12){
    slcd.setCursor(0,1);
    slcd.print("A12:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 13){
    slcd.setCursor(0,1);
    slcd.print("A13:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 14){
    slcd.setCursor(0,1);
    slcd.print("A14:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 15){
    slcd.setCursor(0,1);
    slcd.print("A15:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 16){
    slcd.setCursor(0,1);
    slcd.print("A16:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 17){
    slcd.setCursor(0,1);
    slcd.print("A17:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 18){
    slcd.setCursor(0,1);
    slcd.print("A18:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 19){
    slcd.setCursor(0,1);
    slcd.print("A19:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 20){
    slcd.setCursor(0,1);
    slcd.print("A20:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 21){
    slcd.setCursor(0,1);
    slcd.print("A21:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 22){
    slcd.setCursor(0,1);
    slcd.print("A22:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 23){
    slcd.setCursor(0,1);
    slcd.print("A23:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 24){
    slcd.setCursor(0,1);
    slcd.print("A24:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 25){
    slcd.setCursor(0,1);
    slcd.print("A25:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 26){
    slcd.setCursor(0,1);
    slcd.print("A26:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 27){
    slcd.setCursor(0,1);
    slcd.print("A27:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 28){
    slcd.setCursor(0,1);
    slcd.print("A28:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 29){
    slcd.setCursor(0,1);
    slcd.print("A29:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 30){
    slcd.setCursor(0,1);
    slcd.print("A30:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 31){
    slcd.setCursor(0,1);
    slcd.print("A31:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 32){
    slcd.setCursor(0,1);
    slcd.print("A32:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }
  if (an_disp_count == 33){
    slcd.setCursor(0,1);
    slcd.print("A33:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 34){
    slcd.setCursor(0,1);
    slcd.print("A34:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 35){
    slcd.setCursor(0,1);
    slcd.print("A35:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 36){
    slcd.setCursor(0,1);
    slcd.print("A36:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 37){
    slcd.setCursor(0,1);
    slcd.print("A37:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 38){
    slcd.setCursor(0,1);
    slcd.print("A38:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 39){
    slcd.setCursor(0,1);
    slcd.print("A39:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 40){
    slcd.setCursor(0,1);
    slcd.print("A40:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 41){
    slcd.setCursor(0,1);
    slcd.print("A41:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 42){
    slcd.setCursor(0,1);
    slcd.print("A42:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 43){
    slcd.setCursor(0,1);
    slcd.print("A43:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 44){
    slcd.setCursor(0,1);
    slcd.print("A44:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }
  if (an_disp_count == 45){
    slcd.setCursor(0,1);
    slcd.print("A45:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 46){
    slcd.setCursor(0,1);
    slcd.print("A46:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 47){
    slcd.setCursor(0,1);
    slcd.print("A47:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 48){
    slcd.setCursor(0,1);
    slcd.print("A48:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    an_disp_count =0;
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }
}

//

byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Converti le code binaire en nombre décimale
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

void setDateDs1307(
byte second,        // 0-59
byte minute,        // 0-59
byte hour,          // 1-23
byte dayOfWeek,     // 1-7
byte dayOfMonth,    // 1-28/29/30/31
byte month,         // 1-12
byte year)  // 0-99

{
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0);
  Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
  // bit 6 (also need to change readDateDs1307)
  Wire.write(decToBcd(dayOfWeek));
  Wire.write(decToBcd(dayOfMonth));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
}

// Obtenir la date et l'heure à partir du ds1307
void getDateDs1307(
byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year)
{
  // Reset the register pointer
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);

  // A few of these need masks because certain bits are control bits
  *second     = bcdToDec(Wire.read() & 0x7f);
  *minute     = bcdToDec(Wire.read());
  *hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
  *dayOfWeek  = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month      = bcdToDec(Wire.read());
  *year       = bcdToDec(Wire.read());
}


void SLCDprintFloat(double number, uint8_t digits) 

/* 
 
 Serial LCD ne supporte pas l'affichage des flottantes,
 utilisez cette fonction si vous souhaitez tout de meme en afficher
 SLCDprintFloat(valeur_flottante,nombre de chiffre après la virgule)
 */

{ 
  // Handle negative numbers
  if (number < 0.0)
  {
    slcd.print('-');
    number = -number;
  }

  // Round correctly so that slcd.print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  float remainder = number - (float)int_part;
  slcd.print(int_part , DEC); // base DEC

    // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    slcd.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    float toPrint = float(remainder);
    slcd.print(toPrint , DEC); //base DEC
    remainder -= toPrint; 
  } 
}

float float_map(float x, float in_min, float in_max, float out_min, float out_max)
{
  /* 
   Use this function if you want map values, x = input reading value
   x1 -> x2 >>> y1 -> y2
   */
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}













