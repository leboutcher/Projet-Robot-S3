/*
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon
 * date: 1 mai 2019
*/
 
/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
//#include <libExample.h> // Vos propres librairies
/*------------------------------ Constantes ---------------------------------*/
 
#define BAUD            115200         // Frequence de transmission serielle
#define UPDATE_PERIODE  100            // Periode (ms) d'envoie d'etat general
 
#define MAGPIN          32             // Port numerique pour electroaimant
#define POTPIN          A5             // Port analogique pour le potentiometre
 
#define PASPARTOUR      64*50          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  0.6            // Rapport de vitesse du moteur
 
#define kp              15
#define MAXPIDOUTPUT    kp*1.3         // Valeur maximale du PID
#define WHEELCIRCUM     2*3.1416*0.08  // Circonférence des roues
 
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
volatile bool shouldStartSeq_ = false;    // drapeau pour commencer la séquence
volatile bool shouldStopSeq_ = false;// drapeau pour arrêterla séquence
 
SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse
 
uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 0;                // Amplitude de la tension au moteur [-1,1]
 
 
float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre
double total_distance_traveled;     // variable qui garde la distance totale parcourue
double energy;                      // variable qui garde la puissance totale consommée
float position;                    // variable qui garde la position courante du robot
float last_position;                // variable qui garde la dernière position
bool goalreached = false;           // variable pour indiquer si l'objectif est atteint
bool movementComplete = false;     // variable pour indiquer si le mouvement est complet
bool firstRun = true;              // variable pour indiquer si c'est la première fois que la séquence est appelée
bool enablePID = true;            // variable pour indiquer si le PID est activé
 
// Enumération pour les différentes séquences
enum deplacement
{
  START = 0,
  FORWARD_05 = 1,
  FORWARD_BAC = 2,
  BACKWARD_03 = 3,
  HOME = 4,
  STOP = 5,
} deplacement;
 
 
/*------------------------- Prototypes de fonctions -------------------------*/
 
void timerCallback();
void startPulse();
void endPulse();
void startMag();
void endMag();
void sendMsg();
void readMsg();
void serialEvent();
void sequence();
void FORWARD05f();
void FORWARD_BACf();
void HOMEf();
void STOPf();
void BACKWARD03f();
 
// Fonctions pour le PID
double PIDmeasurement();
void PIDcommand(double cmd);
void PIDgoalReached();
 
double computeAngle();
 
/*---------------------------- fonctions "Main" -----------------------------*/
 
void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle
  AX_.init();                       // initialisation de la carte ArduinoX
  imu_.init();                      // initialisation de la centrale inertielle
  pinMode(MAGPIN, OUTPUT);
 
  //vexEncoder_.init(2,3);            // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  //attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
 
  // Initialisation position
  position = 0;
  last_position = 0;
  energy = 0;
 
  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();
 
  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
 
  // Initialisation du PID
  pid_.setGains(kp, 1.118 , 0.05);
  // Attache des fonctions de retour
  pid_.setMeasurementFunc(PIDmeasurement);
  pid_.setCommandFunc(PIDcommand);
  pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(0.05);
  pid_.setPeriod(100);
  pid_.enable();
  deplacement = START;
}
 
/* Boucle principale (infinie)*/
void loop() {
  //Serial.print("deplacement: ");
  //Serial.println(deplacement);
  if (!enablePID) {
    pid_.enable();
    enablePID = true;
    Serial.println("PID enabled");
  }
 
  sequence();
 
  Serial.print("deplacement: ");
  Serial.println(deplacement);
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
  if(shouldStartSeq_){
    sequence();
  }
  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
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
  digitalWrite(MAGPIN, HIGH);
}
 
void endMag(){
  digitalWrite(MAGPIN, LOW);
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
  doc["position"] = position;
  total_distance_traveled += abs(position - last_position);
  doc["distance"] = total_distance_traveled;
  energy+= abs(AX_.getCurrent()) * abs(AX_.getVoltage());
  doc["energie"] = energy;
 
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
    shouldMagOff_ = false;
  }
 
  parse_msg = doc["magOff"];
  if(!parse_msg.isNull()){
    shouldMagOff_ = doc["magOff"];
    shouldMagOn_ = false;
  }
 
  parse_msg = doc["seqOn"];
    if(!parse_msg.isNull()){
    shouldStartSeq_ = true;
  }
  parse_msg = doc["seqOff"];
    if(!parse_msg.isNull()){
    shouldStopSeq_ = false;
  }
 
 
}
 
// Fonctions pour le PID
double PIDmeasurement(){
 
  last_position = position;
 unsigned long pulses = AX_.readEncoder(0);
 
  float nb_turns = (pulses/(PASPARTOUR) )*0.6;
 
  position = nb_turns * WHEELCIRCUM;
  Serial.print ("Position: ");
  Serial.println(position);
  return position;
}
 
void PIDcommand(double cmd){
  // To do
  cmd = cmd / MAXPIDOUTPUT;
  if (cmd > 1.0) {
    cmd = 1.0;
  } else if (cmd < -1.0) {
    cmd = -1.0;
  }
 
  AX_.setMotorPWM(0,cmd);
}
 
void PIDgoalReached(){
  AX_.setMotorPWM(0,0);
  goalreached = true;
  Serial.println("Goal reached");
  enablePID = false;
}
 
double computeAngle(){
  return (analogRead(POTPIN)-535)*4.55;
}
 
void sequence() {
    switch (deplacement) {
        case START:
            deplacement = FORWARD_05;
            break;
        case FORWARD_05:
            FORWARD05f();
            Serial.println("Forward 0.5");
            break;
        case FORWARD_BAC:
            FORWARD_BACf();
            Serial.println("Forward 1.2");
            break;
        case HOME:
            HOMEf();
            Serial.println("Home");
            break;
        case STOP:
            STOPf();
            Serial.println("Stop");
            break;
        case BACKWARD_03:
            BACKWARD03f();
            Serial.println("Backward 0.3");
            break;
    }
}
 
// State machine functions
void FORWARD05f() {
  pid_.setGoal(0.5);
  pid_.run();
 
  // Check if the goal has been reached
  if (goalreached) {
    if (computeAngle() > -12) {
      goalreached = false;
      Serial.println("Je suis dans le if 1");
      deplacement = BACKWARD_03; // Set movement to backward
    }
    else if (computeAngle() <= -12) {
      goalreached = false;
      Serial.println("Je suis dans le if 2");
      deplacement = FORWARD_BAC; // Set movement to forward
    }
  }
}
 
void FORWARD_BACf() {
  pid_.setGoal(1.2);
  pid_.run();
  if (goalreached) {
      if (computeAngle() <= 5)
      {
        goalreached = false;
        // Electro-aimant off
        deplacement = HOME;
      }
  }
}
void HOMEf() {
  pid_.setGoal(0);
  pid_.run();
if (goalreached) {
      if (computeAngle() <= 5)
      {
        goalreached = false;
        // Electro-aimant off
        deplacement = STOP;
      }
  }
}
 
void STOPf() {
  pid_.setGoal(0);
  pid_.run();
  if (goalreached) {
    goalreached = false;
    deplacement = FORWARD_05;
  }
}
 
void BACKWARD03f() {
  pid_.setGoal(0.3);
  pid_.run();
  if (goalreached) {
    goalreached = false;
    deplacement = FORWARD_05;
  }
}