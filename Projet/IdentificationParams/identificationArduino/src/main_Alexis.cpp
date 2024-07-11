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

#define MAGPIN          32          // Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50*0.6          // Rapport de vitesse du moteur

#define MAXPIDOUTPUT    10*1.3      // Valeur maximale du PID
#define WHEELCIRCUM     2*pi*0.04   // Circonférence des roues

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_;                           // objet PID

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool shouldMagOn_ = false; // drapeau pour activer l'électro-aimant
volatile bool shouldMagOff_ = false;// drapeau pour désactiver l'électro-aimant
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 0;                // Amplitude de la tension au moteur [-1,1]


float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void startMag();
void endMag();
void sendMsg();
void readMsg();
void serialEvent();

// Fonctions pour le PID
double PIDmeasurement();
void PIDcommand(double cmd);
void PIDgoalReached();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle
  AX_.init();                       // initialisation de la carte ArduinoX
  imu_.init();                      // initialisation de la centrale inertielle
  pinMode(MAGPIN, OUTPUT);

  //vexEncoder_.init(2,3);            // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  //attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);

  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);

  // Initialisation du PID
  pid_.setGains(10, 0.01 , 1);
  // Attache des fonctions de retour
  pid_.setMeasurementFunc(PIDmeasurement);
  pid_.setCommandFunc(PIDcommand);
  pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(0.001);
  pid_.setPeriod(200);
}

/* Boucle principale (infinie)*/
void loop() {

  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }
  if(shouldMagOn_){
    startMag();
  }
  if(shouldMagOff_){
    endMag();
  }

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();

  // mise à jour du PID
  pid_.run();
}

/*---------------------------Definition de fonctions ------------------------*/

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
  AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(0,0);
  AX_.setMotorPWM(1,0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void startMag(){
  pinMode(MAGPIN, HIGH);
}

void endMag(){
  pinMode(MAGPIN, LOW);
}

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pid_.getGoal();
  doc["measurements"] = PIDmeasurement();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent();
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
  doc["accelX"] = imu_.getAccelX();
  doc["accelY"] = imu_.getAccelY();
  doc["accelZ"] = imu_.getAccelZ();
  doc["gyroX"] = imu_.getGyroX();
  doc["gyroY"] = imu_.getGyroY();
  doc["gyroZ"] = imu_.getGyroZ();
  doc["isGoal"] = pid_.isAtGoal();
  doc["actualTime"] = pid_.getActualDt();

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
  parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     pulsePWM_ = doc["pulsePWM"].as<float>();
  }

  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     pulseTime_ = doc["pulseTime"].as<float>();
  }

  parse_msg = doc["pulse"];
  if(!parse_msg.isNull()){
     shouldPulse_ = doc["pulse"];
  }
  parse_msg = doc["setGoal"];
  if(!parse_msg.isNull()){
    pid_.disable();
    pid_.setGains(doc["setGoal"][0], doc["setGoal"][1], doc["setGoal"][2]);
    pid_.setEpsilon(doc["setGoal"][3]);
    pid_.setGoal(doc["setGoal"][4]);
    pid_.enable();
  }

  parse_msg = doc["magOn"];
  if(!parse_msg.isNull()){
    shouldMagOn_ = doc["magOn"];
  }

  parse_msg = doc["magOff"];
  if(!parse_msg.isNull()){
    shouldMagOff_ = doc["magOff"];
  }
}

// Fonctions pour le PID
double PIDmeasurement(){
  // To do
  pulses = AX_.ENCODER_Read(0);
  nb_turns = (pulses / PASPARTOUR ) * RAPPORTVITESSE;

  distance_traveled = nb_turns * WHEELCIRCUM;

  return distance_traveled;
}
void PIDcommand(double cmd){
  // To do
  cmd = cmd / MAXPIDOUTPUT;
  if (Cmd > 1.0) {
    Cmd = 1.0;
  } else if (Cmd < -1.0) {
    Cmd = -1.0;
  }

  AX_.setMotorPWM(0,cmd);
}
void PIDgoalReached(){
  // To do
  AX_.setMotorPWM(0,0);
  AX_.ENCODER_Reset(0);
}