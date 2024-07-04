/*
Projet S3 GRO
Class to acces digital-to-analog converter 2629
@author Jean-Samuel Lauzon,
        Marc Rousseau
@version 2.0 06/05/2019
*/
#include "Arduino.h"
#include <Wire.h>

#ifndef DAC2629_h
#define DAC2629_h

/* -- DAC2629 address --*/
#define DAC2629_ADDR 0x72  // DAC2629 address 

class DAC2629
{
    public:
    DAC2629();

    /** Method to set output on DAC submodule
    
    @param submodule
    identification of submodule [0,3]
    
    @param Volt
    svoltage to apply [0.0, 5.0] V

    @return false if parameters out of range
    */
    bool ouput(unsigned int submodule, double volt);

    /** Method to set maximum voltage parameter
     this act as a calibration step
    
    @param offset
    */
    void setMaxVoltage(double offset){ajust_ = offset;};

  private:
    const uint16_t registerAdresses_[4]  = {0x30, 0x31, 0x32, 0x33};
    double ajust_ = 5;
};
#endif