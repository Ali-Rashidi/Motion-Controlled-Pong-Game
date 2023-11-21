/* Read quaternion and roll, pitch, yaw from MPU6050
* Robotics Course semester fall 2022 _ MiniProject #1
*/ 

#include "I2Cdev.h"
#include <Kalman.h>
#include "MPU6050_6Axis_MotionApps20.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

//unsigned long timer;
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO.

#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;  
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;                 // [w, x, y, z]         quaternion container
VectorFloat gravity;          // [x, y, z]            gravity vector
float ypr[3];                 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int32_t gyro[3] ; // Gyro data
unsigned long timer = micros();
// ================================================================
//                          INITIAL SETUP                       
// ================================================================

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(30);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        dmpReady = true;
    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
   mpu.dmpGetQuaternion(&q, fifoBuffer);
   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

   mpu.dmpGetGyro(gyro ,fifoBuffer );


  
}


// ================================================================
//                        MAIN PROGRAM LOOP                       
// ================================================================

void loop() {
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("raw w : ");
            Serial.print(q.w);
            Serial.print(",");
            Serial.print(" raw x : ");
            Serial.print(q.x);
            Serial.print(",");
            Serial.print(" raw y : ");
            Serial.print(q.y);
            Serial.print(",");
            Serial.print(" raw z : ");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
            timer = micros();
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetGyro(gyro ,fifoBuffer );
            float roll = ypr[2] * 180/M_PI;
            float pitch = ypr[1] * 180/M_PI;
            float yaw = ypr[0] * 180/M_PI;
            float gyroX = ((double)mpu.getRotationX()/131.0) ; // sec/deg
            float gyroY = ((double)mpu.getRotationY()/131.0) ;
            float gyroZ = ((double)mpu.getRotationZ()/131.0) ;
            float kalmanRoll= kalmanR(roll);
            float kalmanPitch= kalmanP(pitch);
            float kalmanYaw= kalmanY(yaw);


            // raw sensor data
            Serial.print("raw roll : ");
            Serial.print(roll); // roll
            Serial.print(",");
            Serial.print(" raw pitch : " );
            Serial.print(pitch); 
            Serial.print(",");
            Serial.print(" raw yaw : ");
            Serial.print(yaw); 
            Serial.print(",");

            // Kalman filtered data
            Serial.print(" Kalman roll : ");
            Serial.print(kalmanRoll); // roll
            Serial.print(",");
            Serial.print(" Kalman pitch : " );
            Serial.print(kalmanPitch); 
            Serial.print(",");
            Serial.print(" Kalman yaw : ");
            Serial.println(kalmanYaw); 
           
            
            
        #endif
        
    }

}


float kalmanR(float ur){

  // constatns
  static const double Rr = 100; // noise cov
  static const double Hr = 1.0 ; // measurement map scalar
  static double Qr = 10 ; // error cov
  static double Pr = 0 ; // initial error cov
  static double u_hatr = 0 ; // initial estimation
  static double kr = 0 ; // initial Kalman gain


  kr= Pr*Hr/(Hr*Pr*Hr+Rr); // update kalman gain
  u_hatr = u_hatr + kr * (ur-Hr*u_hatr) ; // update estimation
  Pr = (1- kr*Hr)*Pr+Qr; // update error cov.

  return u_hatr;
}

float kalmanP(float up){

  // constatns
  static const double Rp = 10 ; // noise cov
  static const double Hp = 1.0 ; // measurement map scalar
  static double Qp = 10 ; // error cov
  static double Pp = 0 ; // initial error cov
  static double u_hatp = 0 ; // initial estimation
  static double kp = 0 ; // initial Kalman gain


  kp= Pp*Hp/(Hp*Pp*Hp+Rp); // update kalman gain
  u_hatp = u_hatp + kp * (up-Hp*u_hatp) ; // update estimation
  Pp = (1- kp*Hp)*Pp+Qp; // update error cov.

  return u_hatp;
}

float kalmanY(float uy){

  // constatns
  static const double Ry = 10 ; // noise cov
  static const double Hy = 1.0 ; // measurement map scalar
  static double Qy = 1 ; // error cov
  static double Py = 0 ; // initial error cov
  static double u_haty = 0 ; // initial estimation
  static double ky = 0 ; // initial Kalman gain


  ky= Py*Hy/(Hy*Py*Hy+Ry); // update kalman gain
  u_haty = u_haty + ky * (uy-Hy*u_haty) ; // update estimation
  Py = (1- ky*Hy)*Py+Qy; // update error cov.

  return u_haty;
}



