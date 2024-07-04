/*
Projet S3 GRO
Class to control a PID
@author Jean-Samuel Lauzon
@version 1.0 23/04/2019
*/

#ifndef PID_H_
#define PID_H_

#include <Arduino.h>

class PID
{
  public:
    /** Default constructor
    */
    PID();
    
    /** Method to initialize attributes
     
    @param Kp
    Proportional constant

    @param Ki
    Integral constant

    @param Kd
    Derivative constant 
    */
    PID(double Kp, double Ki, double Kd);

    /** Method to enable the object
    */
    void enable();

    /** Method to disable the object
    */
    void disable();
    
    /** Method to check if the PID needs to be run
    @param goal
    Desired value to converge to
    */
    void run();
    
    // Set methods
    
    /** Method to set all gains constant
    @param Kp
    Proportional constant
    @param Ki
    Integral constant
    @param Kd
    Derivative constant 
    */
    void setGains(double kp, double ki, double kd){
      Kp_ = kp; Ki_ = ki; Kd_ = kd;
    };

    /** Method to set Kp
    @param Kp
    Proportional constant
    */
    void setKp(double kp){Kp_ = kp;};

    /** Method to set Ki
    @param Ki
    Integral constant
    */
    void setKi(double ki){Ki_ = ki;};

    /** Method to set Kd
    @param Kd
    Derivative constant
    */
    void setKd(double kd){Kd_ = kd;};

    /** Method to set period attribute
    @param period
    Time period between iterations in ms
    */
    void setPeriod(unsigned long period){
    dtMs_ = period; 
    dt_=period/1000.0;
    };

    /** Method to set goal attribute
    @param goal
    Desired value to converge to
    */
    void setGoal(double goal){goal_ = goal;};


    /** Method to set measurement function
    @param f
    Parameterless function that returns a double
    (how to get the measurement)
    */
    void setMeasurementFunc(double (*f)()){measurementFunc_ = f;};

    /** Method to set measurement function
    @param f
    no retrun function with double parameter (the command)
    (what to do with the new command)
    */
    void setCommandFunc(void (*f)(double)){commandFunc_ = f;};

     /** Method to set function to run when at goal (optional)
    @param f
    parameterless, no retrun function
    (what to do when goal reached)
    */
    void setAtGoalFunc(void (*f)()){atGoalFunc = f;};

    /** Method to set integral cumulative limit
    @param lim
    limit cumulative error
    */
    void setIntegralLim(double lim){eIntegralLim_ = lim;};

    /** Method to set epsilon
    @param eps
    exit condition
    */
    void setEpsilon(double eps){epsilon_ = eps;};

    // Get methods

    /** Method to know if goal is reached
    @return true if goal is reached within epsilon
    */
    bool isAtGoal(){return atGoal_;}

    /** Method to get the current goal registered
    @return current goal
    */
    double getGoal(){return goal_;}

    /** Method to get the actual delta time
    @return actualDt_
    */
    double getActualDt(){return actualDt_;}

  private:
    /** Method to compute the command
    @param error
    error relative to the desired goal value
    */
    double computeCommand(double error);


    double Kp_ = 0; // Proportional constant
    double Ki_ = 0; // Integral constant
    double Kd_ = 0; // Derivative constant

    // Function pointers
    double (*measurementFunc_)() = nullptr; // Measurement function
    void (*commandFunc_)(double) = nullptr; // Command function
    void (*atGoalFunc)() = nullptr; // Fonction called when goal is reached

    double goal_ = 0; // Desired state
    bool enable_ = false; // Enable flag
    bool atGoal_ = false; // Flag to know if at goal 
    
    double dt_; // Theoric time between 2 measurments
    unsigned long dtMs_; // Periode between commands
    unsigned long actualDt_; // Actual periode between last command
    unsigned long measureTime_ = 0; // Time for next iteration 
    unsigned long lastMeasureTime_ = 0; // Time of last iteration 



    double epsilon_ = 5;
    double eIntegralLim_ = 100;
    double eIntegral_ = 0; // Variable to store the sum of errors
    double ePrevious = 0; // Variable to store the last error
};
#endif //PID
