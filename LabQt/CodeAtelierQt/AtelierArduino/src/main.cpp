/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon     
 * date: 1 mai 2019
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <libExample.h> // Vos propres librairies
/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

/*---------------------------- variables globales ---------------------------*/


volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer lightTimer_;              // chronometre pour éteindre la lumière

int lightState_ = 0;


/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void sendMsg(); 
void readMsg();
void serialEvent();
void turnOnLight();
void turnOffLight();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle
  
  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre éteindre lumière
  lightTimer_.setCallback(turnOffLight);
  lightTimer_.disable();

  pinMode(LED_BUILTIN, OUTPUT);
}

/* Boucle principale (infinie)*/
void loop() {

  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }

  // mise a jour des chronometres
  timerSendMsg_.update();
  lightTimer_.update();
}

/*---------------------------Definition de fonctions ------------------------*/

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}


void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  
  // Elements du message
  doc["time"] = millis();
  doc["lightState"] = lightState_;

  // Serialisation
  serializeJson(doc, Serial);
  
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Analyse des éléments du message message
  // Si on doit allumer la lumière
  parse_msg = doc["turnOnLight"];
  if(!parse_msg.isNull()){
     int duration = doc["turnOnLight"].as<int>();
     turnOnLight();
     lightTimer_.setDelay(duration * 1000);
     lightTimer_.enable();
  }

  // Si on doit faire un echo du message recu
  parse_msg = doc["userMsg"];
  if(!parse_msg.isNull()){
      Serial.println(doc["userMsg"].as<String>());
  }
}

// Allumer la lumière
void turnOnLight(){
  digitalWrite(LED_BUILTIN, HIGH);
  lightState_ = 1;
}

// Éteindre la lumière
void turnOffLight(){
  digitalWrite(LED_BUILTIN, LOW);
  lightTimer_.disable();
  lightState_ = 0;
}