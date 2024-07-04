/*
Projet S1 2018
Class to interface some of the ArduinoX "on board" functionnalities
@author Jean-Samuel Lauzon, Jonathan Vincent, Marc Rousseau
@version 1.0 26/05/2018
*/

#include "ArduinoX.h"

void ArduinoX::init(){
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LOWBAT_PIN, INPUT);
  ina219.begin();
  for(uint8_t id = 0; id < 2; id++){
    motor_[id].init(MOTOR_PWM_PIN[id], MOTOR_DIR_PIN[id]);
    encoder_[id].init(COUNTER_SLAVE_PIN[id], COUNTER_FLAG_PIN[id]);
  }
}

void ArduinoX::buzzerOn(){
  digitalWrite(BUZZER_PIN, HIGH);
}

void ArduinoX::buzzerOn(uint32_t freq){
  tone(BUZZER_PIN, freq);
}

void ArduinoX::buzzerOn(uint32_t freq, uint64_t duration){
  tone(BUZZER_PIN, freq, duration);
}

void ArduinoX::buzzerOff(){
  digitalWrite(BUZZER_PIN, LOW);
}

float ArduinoX::getCurrent(){
  return ina219.getCurrent_mA()/1000.0;
}

float ArduinoX::getBusVoltage(){
  return ina219.getBusVoltage_V();
}

float ArduinoX::getShuntVoltage(){
  return ina219.getShuntVoltage_mV()/1000.0;
}

float ArduinoX::getVoltage(){
  return getBusVoltage() + getShuntVoltage();
}

bool ArduinoX::isLowBat(){
  return !digitalRead(LOWBAT_PIN);
}

void ArduinoX::setMotorPWM(uint8_t id, float PWM){
  if(id<0 || id>1){
    Serial.println("Invalid motor id!");
    return;
  }
  if(id==1){
    PWM *= -1; // left motor is inverted
  }
  motor_[id].setSpeed(PWM);
}

int32_t ArduinoX::readEncoder(uint8_t id){
  if(id<0 || id>1){
    Serial.println("Invalid encoder id!");
    return 0;
  }
  if(id == 0){
    return -encoder_[id].read();// Left motor is inverted
  }else{
    return encoder_[id].read();
  }
}

int32_t ArduinoX::readResetEncoder(uint8_t id){
  if(id<0 || id>1){
    Serial.println("Invalid encoder id!");
    return 0;
  }
  if(id){
    return -encoder_[id].readReset();// Left motor is inverted
  }else{
    return encoder_[id].readReset();
  }
}

void ArduinoX::resetEncoder(uint8_t id){
  if(id<0 || id>1){
    Serial.println("Invalid encoder id!");
    return;
  }
  encoder_[id].reset();// Reset counter
}

void ArduinoX::setDAC(uint8_t id, double voltage){
  dac_.ouput(id,voltage);
}

void ArduinoX::calibrateDAC(double offset){
  dac_.setMaxVoltage(offset);
}