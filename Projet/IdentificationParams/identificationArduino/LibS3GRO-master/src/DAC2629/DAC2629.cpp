/*
Projet S3 GRO
Class to acces digital-to-analog converter 2629
@author Jean-Samuel Lauzon,
        Marc Rousseau
@version 2.0 06/05/2019
*/
#include "DAC2629.h"

DAC2629::DAC2629() {
 }


// Envoi sortie analogique au DAC A ou B
bool DAC2629::ouput(unsigned int submodule, double volt) {
    if(submodule > 3 || submodule < 0){
        return false;
    }
    if(volt < 0 || volt > 5){
        return false;
    }
	unsigned int aout;
	unsigned int msb;
	unsigned int lsb;


	aout = (volt)*4095.0/ajust_;	// convertir volts en 12 bits 0-4095, 3.3v = 1241, 5v = 819
    msb = (aout & 0x0fff) >>4; // garde 12 bits et shift au msb
	lsb = (aout <<4); 	// mettre les 4 derniers bits à 0 au lsb
	

	Wire.beginTransmission(DAC2629_ADDR);           // start transmission to device
    Wire.write(registerAdresses_[submodule]);    // send register address
    Wire.write(msb);                                // send MSB value to write
	Wire.write(lsb);                                // send LSB value to write
	Wire.endTransmission();                         // end transmission 
    return true;
}