/*
Projet S3 GRO 2019
General librairies for S3
@author Jean-Samuel Lauzon
@version 1.0 17/03/2019
*/
#ifndef LibS3GRO_H_
#define LibS3GRO_H_

// Includes
#include <ArduinoX/ArduinoX.h>
#include <VexQuadEncoder/VexQuadEncoder.h>
#include <SoftTimer/SoftTimer.h>
#include <MegaServo/MegaServo.h>
#include <IMU9DOF/IMU9DOF.h>
#include <PID/PID.h>

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
#define endl "\r\n"

#endif //LibS3GRO
