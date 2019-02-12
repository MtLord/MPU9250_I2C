/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>

#include <Wire.h>

#define SerialPort SerialUSB

MPU9250_DMP imu;
float yawdata=0;
uint8_t SendtData[4];

void SendIMUData()
{
  SendtData[0]=((unsigned char *)&yawdata)[0] ;
  SendtData[1]=((unsigned char *)&yawdata)[1] ;
  SendtData[2]=((unsigned char *)&yawdata)[2] ;
  SendtData[3]=((unsigned char *)&yawdata)[3] ; 
  Wire.beginTransmission(8); 
  Wire.write(SendtData[0]); 
  Wire.write(SendtData[1]);
  Wire.write(SendtData[2]);
  Wire.write(SendtData[3]);
   Wire.endTransmission(); 
}
void setup() 
{
  //Wire.begin();// Slave ID #8
  SerialPort.begin(115200);
  Wire.begin();
  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              125); 
               SerialPort.println("DMP Init sucecess!");
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

}




void loop() 
{
  if ( imu.fifoAvailable() )
    {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
        {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      yawdata=imu.yaw;
        SendIMUData();
         }
 
   }
 
}
