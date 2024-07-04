/*
Projet S3 GRO
Class to control a PID
@author Jean-Samuel Lauzon
@version 1.0 23/04/2019
*/

#include "PID.h"

PID::PID(){
}


PID::PID(double kp, double ki, double kd){
    Kp_ = kp;
    Ki_ = ki;
    Kd_ = kd;
}

void PID::enable(){

    // if function pointers are initiated
    if(measurementFunc_!=nullptr && commandFunc_!=nullptr){
        enable_ = true;
        measureTime_ = millis() + dtMs_;
        atGoal_ = false;
        eIntegral_ = 0;
        ePrevious = 0;
    }
    
    
}

void PID::disable(){
    enable_ = false;
}


void PID::run(){
    // if enabled and time to run iteration
    if(millis() >= measureTime_ && enable_){

        //actualDt_ = millis() - measureTime_;
        measureTime_ = millis() + dtMs_;
        double error = goal_ - measurementFunc_();
        
        // if goal reached
        if(fabs(error)<epsilon_){

            atGoal_ = true;
            enable_ = false;
            if (atGoalFunc != nullptr){
                atGoalFunc();
            }
        }else{
            commandFunc_(computeCommand(error));
        }
       lastMeasureTime_ =  measureTime_;
    }

}

double PID::computeCommand(double error){
    double CMD;
    actualDt_ = millis() - lastMeasureTime_;
    eIntegral_ += error;
    
    // Integral saturation
    if(eIntegral_ > eIntegralLim_){
        eIntegral_ = eIntegralLim_;
    }
    if(eIntegral_ < -eIntegralLim_){
        eIntegral_ = -eIntegralLim_;
    }

    CMD = Kp_*error + Ki_*eIntegral_*dt_ + Kd_*(error-ePrevious)/dt_;

    ePrevious = error;
    return CMD;
}