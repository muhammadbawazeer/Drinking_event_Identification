#ifndef IMU_h
#define IMU_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Wire.h>

typedef struct {
  double X, Y;
} coordinate;


class IMU
{
  private:
   
    const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
    const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

    
    Kalman kalmanX; // Create the Kalman instances
    Kalman kalmanY;
    double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
  
    // i2c pins for IMU sensor
    byte PD_SCK;  // Power Down and Serial Clock Input Pin
    byte DOUT;    // Serial Data Output Pin
    uint8_t i2cData[14]; // Buffer for I2C data

    uint32_t Timer;
    
    double Y[2000], X[2000], Extreme[6]; // storing displacements for possible drinking motion
    
    // multinomial classification weights using 6 features with output of 3 classes, obtained from our conducted research
    double Coef_0[6] = {-6.58910403, -0.07649387, -0.03332975 , 0.33974072,  0.98064618, -5.16652708}; 
    double Coef_1[6] = { 9.60422708 , 1.26480346,  2.29206196,  5.08289579, -0.55983905 , 3.8874351 };
    double Coef_2[6] = {-3.01512306 ,-1.18830959 ,-2.2587322,  -5.42263651 ,-0.42080712 , 1.27909198}; // drinking weights
    double Inter[3] = { 6.97278704, -10.10732886  , 3.13454183};

  public:
    // hyper-parameters
    
    float Min_amplitude = 70.46, Min_angle_threshold = 27.6, Min_duration = 1864, Max_duration = 7810, Max_count_extreme = 6; // values obtained from conducted exploration using dataset obtained from simulation
    coordinate Angle; 
    
    IMU(byte dout, byte pd_sck); 

    IMU(); 

    virtual ~IMU(); 

    void init(byte dout, byte pd_sck); // Initialisation or Constructor

    void read_IMU(); // read IMU sensor

    void calibration(); // Calibrating IMU Sensor

    // functions for capturing, filtering and classifying drinking activity
    
    bool track_activity(double current_angle_y); // track displacements of an activity
      
    bool check_extreme(int curr_extreme,int angle, int direction_movement); // Peak and Valley detection
      
    bool feature_extraction_and_classification(double mean_Y, double mean_X, double mean_extreme, int index_max); // Extracting Features and perform classification with softmax function
    
    bool filter_activity(uint32_t duration, int length_extreme); // filter motions
        
    int logreg_softmax_classification(double mean_y, double standard_dev_extreme, double ratio, float standard_dev_y, double mean_x, double standard_dev_x); // done, {0:drinking, other: non-drinking}

    
    void refresh_variables(); // set to 0's
    
    uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);
    uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
    uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);

};

#endif /* IMU_h */ 








