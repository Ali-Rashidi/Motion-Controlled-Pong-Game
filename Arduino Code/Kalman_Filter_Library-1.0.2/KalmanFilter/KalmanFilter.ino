#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
/* Read quaternion and roll, pitch, yaw from MPU6050
* Robotics Course semester fall 2022 _ MiniProject #1 */ 

// ==== KALMAN PARAMETERS ====

#include <Kalman.h>
using namespace BLA;

// ==== YPR ====

// Dimensions of the matrices
#define NstateYPR 3 // length of the state vector
#define NobsYPR 3   // length of the measurement vector

// measurement std (to be characterized from your sensors)
#define n1YPR 0.2 // noise on the 1st measurement component
#define n2YPR 0.1 // noise on the 2nd measurement component 
#define n3YPR 0.1 // noise on the 3nd measurement component

// model std (~1/inertia). Freedom you give to relieve your evolution equation
#define m1YPR 0.01
#define m2YPR 0.02
#define m3YPR 0.02


// ==== QUATERNION ====

#define NstateQuat 4 // length of the state vector
#define NobsQuat 4   // length of the measurement vector

#define n1Quat 0.2 // noise on the 1st measurement component
#define n2Quat 0.1 // noise on the 2nd measurement component 
#define n3Quat 0.1 // noise on the 3nd measurement component
#define n4Quat 0.1 // noise on the 4nd measurement component

#define m1Quat 0.01
#define m2Quat 0.02
#define m3Quat 0.02
#define m4Quat 0.02

KALMAN<NstateYPR,NobsYPR> KYPR; // your Kalman filter
BLA::Matrix<NobsYPR> obsYPR; // observation vector

KALMAN<NstateQuat,NobsQuat> KQuat; // your Kalman filter
BLA::Matrix<NobsQuat> obsQuat; // observation vector

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO.
//#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;  
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;                 // [w, x, y, z]         quaternion container
VectorFloat gravity;          // [x, y, z]            gravity vector
float ypr[3];                 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// ================================================================
//                          INITIAL SETUP                       
// ================================================================

void setup() {
  // ====
  // YPR
  // ====
  // example of evolution matrix. Size is <Nstate,Nstate>
  KYPR.F = {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};
  // example of measurement matrix. Size is <Nobs,Nstate>
  KYPR.H = {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};
  // example of measurement covariance matrix. Size is <Nobs,Nobs>
  KYPR.R = {n1YPR*n1YPR, 0.0, 0.0,
            0.0, n2YPR*n2YPR, 0.0,
            0.0, 0.0, n3YPR*n3YPR};
  // example of model covariance matrix. Size is <Nstate,Nstate>
  KYPR.Q = {m1YPR*m1YPR, 0.0, 0.0,
            0.0, m2YPR*m2YPR, 0.0,
            0.0, 0.0, m3YPR*m3YPR};

  // ==== 
  // QUATERNION 
  // ====
  KQuat.F = {1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0};
  // example of measurement matrix. Size is <Nobs,Nstate>
  KQuat.H = {1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0};
  // example of measurement covariance matrix. Size is <Nobs,Nobs>
  KQuat.R = {n1Quat*n1Quat, 0.0, 0.0, 0.0,
             0.0, n2Quat*n2Quat, 0.0, 0.0,
             0.0, 0.0, n3Quat*n3Quat, 0.0,
             0.0, 0.0, 0.0, n4Quat*n4Quat};
  // example of model covariance matrix. Size is <Nstate,Nstate>
  KQuat.Q = {m1Quat*m1Quat, 0.0, 0.0, 0.0,
             0.0, m2Quat*m2Quat, 0.0, 0.0,
             0.0, 0.0, m3Quat*m3Quat, 0.0,
             0.0, 0.0, 0.0, m4Quat*m4Quat};

  
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
        mpu.CalibrateGyro(6);
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
}


// ================================================================
//                        MAIN PROGRAM LOOP                       
// ================================================================
void loop() {
  // GRAB MEASUREMENT and WRITE IT INTO 'obs'
  get_sensor_data();
  // PRINT RESULTS: measures and estimated state
  #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // APPLY KALMAN FILTER  
      KYPR.update(obsYPR);          
      Serial.print(obsYPR(0)); // roll
      Serial.print(",");
      Serial.print(obsYPR(1)); // pitch
      Serial.print(",");
      Serial.print(obsYPR(2)); // yaw
      Serial.print(",");
      Serial.print(KYPR.x(0)); // Kalman roll
      Serial.print(",");
      Serial.print(KYPR.x(1)); // Kalman pitch
      Serial.print(",");
      Serial.println(KYPR.x(2)); // Kalman yaw
  #endif
  
  #ifdef OUTPUT_READABLE_QUATERNION
      KQuat.update(obsQuat); 
      Serial.print(obsQuat(0)); // q.w
      Serial.print(",");
      Serial.print(obsQuat(1)); // q.x
      Serial.print(",");
      Serial.print(obsQuat(2)); // q.y
      Serial.print(",");
      Serial.print(obsQuat(3)); // q.z
      Serial.print(",");
      Serial.print(KQuat.x(0)); // Kalman q.w
      Serial.print(",");
      Serial.print(KQuat.x(1)); // Kalman q.x
      Serial.print(",");
      Serial.print(KQuat.x(2)); // Kalman q.y
      Serial.print(",");
      Serial.println(KQuat.x(3)); // Kalman q.z 
   #endif       
}


void get_sensor_data() {
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            obsQuat(0) = q.w;
            obsQuat(1) = q.x;
            obsQuat(2) = q.y;
            obsQuat(3) = q.z;
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            float roll = ypr[2] * 180/M_PI;
            float pitch = ypr[1] * 180/M_PI;
            float yaw = ypr[0] * 180/M_PI;
            obsYPR(0) = roll;
            obsYPR(1) = pitch;
            obsYPR(2) = yaw;
        #endif
    }
}
