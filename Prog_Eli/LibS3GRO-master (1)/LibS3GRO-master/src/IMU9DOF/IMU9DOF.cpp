/*
Projet S3 GRO 2019
Class to communicate to Grove IMU 9DOF
@author Jean-Samuel Lauzon
@version 1.0 27/03/2019
*/
// From the example file by SeeedStudio
#include "IMU9DOF.h"

void IMU9DOF::init()
{
  imu_.initialize();
}


double IMU9DOF::getAccelX(){
    return (double) imu_.getAccelerationX() / 16384;
}

double IMU9DOF::getAccelY(){
    return (double) imu_.getAccelerationY() / 16384;
}

double IMU9DOF::getAccelZ(){
    return (double) imu_.getAccelerationZ() / 16384;
}

double IMU9DOF::getGyroX(){
    return (double) imu_.getRotationX() * 250 / 32768;
}

double IMU9DOF::getGyroY(){
    return (double) imu_.getRotationY() * 250 / 32768;
}

double IMU9DOF::getGyroZ(){
    return (double) imu_.getRotationZ() * 250 / 32768;
}

double IMU9DOF::getTemp(){
    return (double) imu_.getTemperature() /340 + 36.53;
}