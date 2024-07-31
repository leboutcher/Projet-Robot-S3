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
 
#define PASPARTOUR      64.0*50.0          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  0.6            // Rapport de vitesse du moteur
 
#define kp              100
#define MAXPIDOUTPUT    1    // Valeur maximale du PID
#define WHEELCIRCUM     2*3.1416*0.08  // Circonférence des roues
 
/*---------------------------- variables globales ---------------------------*/
int i = 1; 

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
double position;                    // variable qui garde la position courante du robot
double last_position;                // variable qui garde la dernière position
bool goalreached = false;           // variable pour indiquer si l'objectif est atteint
bool movementComplete = false;     // variable pour indiquer si le mouvement est complet
bool firstRun = true;              // variable pour indiquer si c'est la première fois que la séquence est appelée
bool enablePID = true;            // variable pour indiquer si le PID est activé
int value = 2;
int last_angle;
float value_stab = 2 ;
int erreur = 0;
 
// Enumération pour les différentes séquences
enum deplacement
{
  START = 0,
  FORWARD_05 = 1,
  FORWARD_PASS = 2,
  BACKWARD_03 = 3,
  HOME = 4,
  STOP = 5,
  STAB_FORWARD = 7,
  STAB_BACKWARD = 8,
  EMERGENCYSTOP = 9,
  FORWARD_BOX = 10,
  ENCODER = 11,
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
void FORWARD_PASSf();
void HOMEf();
void BACKWARD03f();
void ARRETf();
void STAB_FORWARDf();
void STAB_BACKWARDf();
void EMERGENCYSTOPf();
void FORWARD_BOXf();
void ENCODERf();

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
  pid_.setGains(kp, 1 , 1);
  // Attache des fonctions de retour
  pid_.setMeasurementFunc(PIDmeasurement);
  pid_.setCommandFunc(PIDcommand);
  pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(0.02);
  pid_.setPeriod(100);
  pid_.enable();
  deplacement = START;
}
 
/* Boucle principale (infinie)*/
void loop() {
 
  if (!enablePID) {
    pid_.enable();
    enablePID = true;
  }

  sequence();

 
  // if(shouldRead_){
  //   readMsg();
  // }
  // if(shouldSend_){
  //   sendMsg();
  // }
  // if(shouldPulse_){
  //   startPulse();
  // }
  // if(shouldMagOn_){
  //   startMag();
  // }
  // if(shouldMagOff_){
  //   endMag();
  // }
  // if(shouldStartSeq_){
  //   sequence();
  // }
  // // mise a jour des chronometres
  // timerSendMsg_.update();
  // timerPulse_.update();
  // Serial.print("distance: ");
  // Serial.println(position);
  
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
  digitalWrite(MAGPIN, HIGH);
}
 
void endMag(){
  digitalWrite(MAGPIN, LOW);
}
 
void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message
 
  doc["potVex"] = analogRead(POTPIN);
  doc["goal"] = pid_.getGoal();
  doc["measurements"] = PIDmeasurement();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent();
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
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
    //Serial.print("deserialize() failed: ");
    //Serial.println(error.c_str());
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
 
  parse_msg = doc["sequOn"];
    if(!parse_msg.isNull()){
    shouldStartSeq_ = true;
    pid_.enable();
    deplacement = START;
  }
  parse_msg = doc["sequOff"];
    if(!parse_msg.isNull()){
    shouldStopSeq_ = false;
    deplacement = EMERGENCYSTOP;
    sequence();
  }
 
 
}
 
// Fonctions pour le PID
double PIDmeasurement(){

  last_position = position;
  long pulses = AX_.readEncoder(0);
  double  nb_turns = (pulses/(PASPARTOUR) )*0.6;
  position = nb_turns * WHEELCIRCUM;
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
 
  AX_.setMotorPWM(0,cmd/value);
}
 
void PIDgoalReached(){
  goalreached = true;
  enablePID = false;
}
 
double computeAngle(){
  return (analogRead(POTPIN)-535)/4.55;
}
 
void sequence() {
    switch (deplacement) {
        case START:
            pid_.setGoal(0);
            digitalWrite(MAGPIN, HIGH);
            if (goalreached) 
            {
              goalreached = false;
              int previous_millis = millis();
              while (millis() - previous_millis < 3000) {
                AX_.setMotorPWM(0,0);
                digitalWrite(MAGPIN, HIGH);
                value = 2;
              }
              deplacement = FORWARD_05;
            }
            break;
        case FORWARD_05:
            FORWARD05f();
            //Serial.println("Forward 0.5");
            break;
        case FORWARD_PASS:
            FORWARD_PASSf();
            //Serial.println("Forward 1.2");
            break;
        case HOME:
            HOMEf();
            //Serial.println("Home");
            break;
        case EMERGENCYSTOP:
            EMERGENCYSTOPf();
            break;
        case BACKWARD_03:
            BACKWARD03f();
            //Serial.println("Backward 0.3");
            break;
        case STOP:
            ARRETf();
            //Serial.println("Stop");
            break;
        case STAB_FORWARD:
            STAB_FORWARDf();
            //Serial.println("Stop");
            break;
        case STAB_BACKWARD:
            STAB_BACKWARDf();
            //Serial.println("Stop");
            break;
        case FORWARD_BOX:
            FORWARD_BOXf();
            //Serial.println("Stop");
            break;
        case ENCODER:
            ENCODERf();
            break;
    }
}
 
// State machine functions
void FORWARD05f() {
  pid_.setGoal(0.19);
  // Check if the goal has been reached
  if (goalreached) {
      goalreached = false;
      value = 2;
      deplacement = BACKWARD_03;// Set movement to backward
   
  }
}

void BACKWARD03f() {
  pid_.setGoal(0.185);
  if (goalreached) { 
    if (computeAngle() < 5) { 
   
    goalreached = false;
    value = 2;
    deplacement = FORWARD_05;
    }
    if(computeAngle() >= 5){
      goalreached = false;
      value = 1;
      deplacement = FORWARD_BOX; // was FORWARD_PASS 
    }
  }
}

void FORWARD_PASSf() {

  // pid_.setGoal(0.65);
  // if (goalreached) {
  //       goalreached = false;
  //       value = 2.7;
  //       deplacement = FORWARD_BOX;
  // }
}

void FORWARD_BOXf(){

  pid_.setGoal(1);
  last_angle = computeAngle();

  if (goalreached){
    if(i>=1){
      goalreached = false;
      value = value_stab;
      long time = millis();
      while (millis() - time < 1000) 
      {
      AX_.setMotorPWM(0,0);
      }
      deplacement = STOP;
    }
    else if (computeAngle() > 0 && computeAngle() > last_angle)
    {
      goalreached = false;
      deplacement = STAB_FORWARD;
    } 
    else if (computeAngle() > 0 && computeAngle() < last_angle)
    {
      goalreached = false;
      deplacement = STAB_BACKWARD;
    }

    else if (computeAngle() < 0 && computeAngle() > last_angle)
    {
      goalreached = false;
      deplacement = STAB_BACKWARD;
    }
    else if(computeAngle() < 0 && computeAngle() < last_angle)
    {
      goalreached = false;
      deplacement = STAB_FORWARD;
    }
  }
}


void STAB_FORWARDf() {
  value = value_stab;
  pid_.setGoal(1.02 );
  last_angle = computeAngle();
  if (goalreached) {
    if (i>=1)
    {
      goalreached = false;
      value = 3;
      deplacement = FORWARD_BOX;
    }
    else if(computeAngle() > 0 && computeAngle() > last_angle)
    {
      goalreached = false;
      value = value_stab;
      i++;
      deplacement = STAB_FORWARD;
    } 
    else if (computeAngle() > 0 && computeAngle() < last_angle)
    {
      goalreached = false;
      value = value_stab;
      i++;
      deplacement = STAB_BACKWARD;
    }

    else if (computeAngle() < 0 && computeAngle() > last_angle)
    {
      goalreached = false;
      value = value_stab;
      i++;
      deplacement = STAB_BACKWARD;
    }
    else if (computeAngle() < 0 && computeAngle() < last_angle)
    {
      goalreached = false;
      value = value_stab;
      i++;
      deplacement = STAB_FORWARD;
    }
  }
}

void STAB_BACKWARDf() {
  pid_.setGoal(0.98);
  last_angle = computeAngle();
  if (goalreached) {
    if (i>=1)
    {
      goalreached = false;
      value = 3;
      deplacement = FORWARD_BOX;
    }
    else if(computeAngle() > 0 && computeAngle() > last_angle)
    {
      goalreached = false;
      value = value_stab;
      i++;
      deplacement = STAB_FORWARD;
    } 
    else if (computeAngle() > 0 && computeAngle() < last_angle)
    {
      goalreached = false;
      value = value_stab;
      i++;
      deplacement = STAB_BACKWARD;
    }

    else if (computeAngle() < 0 && computeAngle() > last_angle)
    {
      goalreached = false;
      value = value_stab;
      i++;
      deplacement = STAB_BACKWARD;
    }
    else if (computeAngle() < 0 && computeAngle() < last_angle)
    {
      goalreached = false;
      value = value_stab;
      i++;
      deplacement = STAB_FORWARD;
    }
  }
}

void ARRETf() {
    int previoustime = millis();
    while (millis() - previoustime < 1000) {
        AX_.setMotorPWM(0,0);
        digitalWrite(MAGPIN, LOW);
    }
    deplacement = HOME;
    
}

void HOMEf() {

  value = 3;
  pid_.setGoal(0);
  i=0;
  digitalWrite(MAGPIN, LOW);
if (goalreached) {
      if (computeAngle() <= 5)
      {
        goalreached = false;
        digitalWrite(MAGPIN, HIGH);
        value = 2;
        //AX_.resetEncoder(0);
        deplacement = ENCODER;
      }
  }
}

void ENCODERf(){
  pid_.setGoal(-0.08);
  if (goalreached){
    AX_.resetEncoder(0);
    // Serial.print("Encodeur values: ");
    // Serial.println(AX_.readEncoder(0));
    deplacement = START;
  }
}

void EMERGENCYSTOPf(){
  AX_.setMotorPWM(0,0);
  pid_.disable();
  enablePID = true;
  AX_.resetEncoder(0);
}