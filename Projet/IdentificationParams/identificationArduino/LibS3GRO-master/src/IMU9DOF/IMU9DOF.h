/*
Projet S3 GRO 2019
Class to communicate to Grove IMU 9DOF
@author Jean-Samuel Lauzon
@version 1.0 27/03/2019
*/
#ifndef IMU9DOF_h
#define IMU9DOF_h

#include "MPU6050.h"
// From the example file by SeeedStudio

class IMU9DOF
{
  public:
    /** Methode to initialize communication between arduino and IMU9DOF
    */
    void init();

    /** Method to poll the availability of the sensor

    @return true if available
    */
    bool isConnected(){return imu_.testConnection();};

    /** Method to get acceleration data in X

    @return double value in g [-2, 2]
    */
    double getAccelX();

    /** Method to get acceleration data in Y

    @return double value in g [-2, 2]
    */
    double getAccelY();

    /** Method to get acceleration data in Z

    @return double value in g [-2, 2]
    */
    double getAccelZ();

    
    /** Method to get gyroscope data in X

    @return double value in degrees/s [-250, 250]
    */
    double getGyroX();

    /** Method to get gyroscope data in Y

    @return double value in degrees/s [-250, 250]
    */
    double getGyroY();

    /** Method to get gyroscope data in Z

    @return double value in degrees/s [-250, 250]
    */
    double getGyroZ();
    
    /** Method to get temperature

    @return double value degrees C
    */
    double getTemp();

  private:
    MPU6050 imu_; 
};
#endif // IMU9DOF