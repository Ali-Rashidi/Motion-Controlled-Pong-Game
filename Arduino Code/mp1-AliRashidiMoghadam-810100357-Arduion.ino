/* Read quaternion and roll, pitch, yaw from MPU6050
* Robotics Course semester fall 2022 _ MiniProject #1
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

long loopTimer, loopTimer2;
int temperature;
double accelPitch;
double accelRoll;
double accelPitch2=0;
double accelRoll2=0;
double accelPitch3=0;
double accelRoll3=0;
long acc_x, acc_y, acc_z;
double accel_x, accel_y, accel_z;
double gyroRoll, gyroPitch, gyroYaw;

int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
double rotation_x, rotation_y, rotation_z;
double freq, dt;
double tau = 0.96;  // angle = tau (angle + gyro*dt) + (1-tau)*(acc) // smaller tau tracks faster changes
double k = 0.00001;
double comp1_roll = 0;
double comp1_pitch = 0;
double comp1_yaw = 0;

double comp2_roll = 0;
double comp2_pitch = 0;
double comp2_yaw = 0;
double comp2_roll2 = 0;
double comp2_pitch2 = 0;
double comp2_yaw2 = 0;

// 250 deg/s --> 131.0, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
long scaleFactorGyro = 65.5;

// 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
long scaleFactorAccel = 8192;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

//unsigned long timer;
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO.

#define OUTPUT_READABLE_YAWPITCHROLL
#define kalman_filter // uncommnet to print the data
#define comp1_filter  // uncommnet to print the data
#define comp2_filter  // uncommnet to print the data

bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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
  while (!Serial)
    ;

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
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  setup_mpu_6050_registers();

  // Take 1000 readings for each coordinate and then find average offset
  for (int cal_int = 0; cal_int < 1000; cal_int++) {
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

    delay(3);
  }

  // Average the values
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

  delay(200);

  // Reset the loop timer
  loopTimer = micros();
  loopTimer2 = micros();
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
    float kalmanW = kalmanw(q.w);
    float kalmanX = kalmanx(q.x);
    float kalmanYy = kalmany(q.y);
    float kalmanZ = kalmanz(q.z);

    // raw sensor data
    //Serial.print("raw w : ");
    Serial.print(q.w);
    Serial.print(",");
    //Serial.print(" raw x : ");
    Serial.print(q.x);
    Serial.print(",");
    //Serial.print(" raw y : ");
    Serial.print(q.y);
    Serial.print(",");
    //Serial.print(" raw z : ");
    Serial.print(q.z);
    Serial.print(",");

// Kalman filtered data
#ifdef kalman_filter
    //Serial.print("Kalman w : ");
    Serial.print(kalmanW);
    Serial.print(",");
    //Serial.print(" Kalman x : ");
    Serial.print(kalmanX);
    Serial.print(",");
    //Serial.print(" Kalman y : ");
    Serial.print(kalmanYy);
    Serial.print(",");
    //Serial.print(" Kalman z : ");
    Serial.print(kalmanZ);
    Serial.print(",");


#endif
    //Serial.println();

#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //***Raw data
    float roll = ypr[2] * 180 / M_PI;
    float pitch =- ypr[1] * 180 / M_PI;
    float yaw =- ypr[0] * 180 / M_PI;

    //***Kalman
    float kalmanRoll = kalmanR(roll);
    float kalmanPitch = kalmanP(pitch);
    float kalmanYaw = kalmanY(yaw);

    //****Comp.1
    freq = 1 / ((micros() - loopTimer2) * 1e-6);
    loopTimer2 = micros();
    dt = 1 / freq;

    // Read the raw acc data from MPU-6050
    read_mpu_6050_data();

    // Subtract the offset calibration value
    gyro_x -= gyro_x_cal;
    gyro_y -= gyro_y_cal;
    gyro_z -= gyro_z_cal;

    // Convert to instantaneous degrees per second
    rotation_x = (double)gyro_x / (double)scaleFactorGyro;
    rotation_y = (double)gyro_y / (double)scaleFactorGyro;
    rotation_z = (double)gyro_z / (double)scaleFactorGyro;

    // Convert to g force
    accel_x = (double)acc_x / (double)scaleFactorAccel;
    accel_y = (double)acc_y / (double)scaleFactorAccel;
    accel_z = (double)acc_z / (double)scaleFactorAccel;

    // 1st order Complementary filter
    accelPitch = atan2(accel_y, accel_z) * RAD_TO_DEG;
    accelRoll = atan2(accel_x, accel_z) * RAD_TO_DEG;

    comp1_pitch = (tau) * (comp1_pitch + rotation_y * dt) + (1 - tau) * (accelPitch);
    comp1_roll = (tau) * (comp1_roll - rotation_x * dt) + (1 - tau) * (accelRoll);
    comp1_yaw = gyroYaw;


    
    gyroPitch += rotation_x * dt;
    gyroRoll -= rotation_y * dt;
    gyroYaw += rotation_z * dt;

    // raw sensor data
    //Serial.print("raw roll : ");
    Serial.print(roll);  // roll
    Serial.print(",");
    //Serial.print(" raw pitch : ");
    Serial.print(pitch);
    Serial.print(",");
    //Serial.print(" raw yaw : ");
    Serial.print(yaw);
    Serial.print(",");

// Kalman filter data
#ifdef kalman_filter
    //Serial.print(" Kalman roll : ");
    Serial.print(kalmanRoll);  // roll
    Serial.print(",");
    //Serial.print(" Kalman pitch : ");
    Serial.print(kalmanPitch);
    Serial.print(",");
    //Serial.print(" Kalman yaw : ");
    Serial.print(kalmanYaw);
    Serial.print(",");

#endif

// Comp.1 data
#ifdef comp1_filter
    //Serial.print(" Comp.1 roll : ");
    Serial.print(comp1_roll);  // roll
    Serial.print(",");
    //Serial.print(" Comp.1 pitch : ");
    Serial.print(comp1_pitch);
    Serial.print(",");
    ///Serial.print(" Comp.1 yaw : ");
    Serial.print(comp1_yaw);


#endif

//#ifdef comp2_filter
 //   Serial.print(",");
  //  Serial.print(" Comp.2 roll : ");
  //  Serial.print(comp2_roll);  // roll
    //Serial.print(",");
    //Serial.print(" Comp.2 pitch : ");
   // Serial.print(comp2_pitch);
   // Serial.print(",");
    //Serial.print(" Comp.2 yaw : ");
   // Serial.print(comp2_yaw);
//#endif

    while (micros() - loopTimer <= 4000)
      ;
    loopTimer = micros();
  
    




#endif
Serial.println();
  }
}

/////////////////*******************************///////////////////////////////////////////////

/////////////////*******************************///////////////////////////////////////////////

//////////////////////////////////functions////////////////////////////////////////////////////


/////////////////*******************************///////////////////////////////////////////////

/////////////////*******************************///////////////////////////////////////////////

/////////////////*******************************///////////////////////////////////////////////

float kalmanR(float ur) {

  // constatns
  static const double Rr = 100;  // noise cov
  static const double Hr = 1.0;  // measurement map scalar
  static double Qr = 10;         // error cov
  static double Pr = 0;          // initial error cov
  static double u_hatr = 0;      // initial estimation
  static double kr = 0;          // initial Kalman gain


  kr = Pr * Hr / (Hr * Pr * Hr + Rr);         // update kalman gain
  u_hatr = u_hatr + kr * (ur - Hr * u_hatr);  // update estimation
  Pr = (1 - kr * Hr) * Pr + Qr;               // update error cov.

  return u_hatr;
}

float kalmanP(float up) {

  // constatns
  static const double Rp = 10;   // noise cov
  static const double Hp = 1.0;  // measurement map scalar
  static double Qp = 10;         // error cov
  static double Pp = 0;          // initial error cov
  static double u_hatp = 0;      // initial estimation
  static double kp = 0;          // initial Kalman gain


  kp = Pp * Hp / (Hp * Pp * Hp + Rp);         // update kalman gain
  u_hatp = u_hatp + kp * (up - Hp * u_hatp);  // update estimation
  Pp = (1 - kp * Hp) * Pp + Qp;               // update error cov.

  return u_hatp;
}

float kalmanY(float uy) {

  // constatns
  static const double Ry = 10;   // noise cov
  static const double Hy = 1.0;  // measurement map scalar
  static double Qy = 1;          // error cov
  static double Py = 0;          // initial error cov
  static double u_haty = 0;      // initial estimation
  static double ky = 0;          // initial Kalman gain


  ky = Py * Hy / (Hy * Py * Hy + Ry);         // update kalman gain
  u_haty = u_haty + ky * (uy - Hy * u_haty);  // update estimation
  Py = (1 - ky * Hy) * Py + Qy;               // update error cov.

  return u_haty;
}

float kalmanw(float uw) {

  // constatns
  static const double Rw = 10;   // noise cov
  static const double Hw = 1.0;  // measurement map scalar
  static double Qw = 1;          // error cov
  static double Pw = 0;          // initial error cov
  static double u_hatw = 0;      // initial estimation
  static double kw = 0;          // initial Kalman gain


  kw = Pw * Hw / (Hw * Pw * Hw + Rw);         // update kalman gain
  u_hatw = u_hatw + kw * (uw - Hw * u_hatw);  // update estimation
  Pw = (1 - kw * Hw) * Pw + Qw;               // update error cov.

  return u_hatw;
}


float kalmanx(float ux) {

  // constatns
  static const double Rx = 10;   // noise cov
  static const double Hx = 1.0;  // measurement map scalar
  static double Qx = 1;          // error cov
  static double Px = 0;          // initial error cov
  static double u_hatx = 0;      // initial estimation
  static double kx = 0;          // initial Kalman gain


  kx = Px * Hx / (Hx * Px * Hx + Rx);         // update kalman gain
  u_hatx = u_hatx + kx * (ux - Hx * u_hatx);  // update estimation
  Px = (1 - kx * Hx) * Px + Qx;               // update error cov.

  return u_hatx;
}



float kalmany(float uY) {

  // constatns
  static const double RY = 10;   // noise cov
  static const double HY = 1.0;  // measurement map scalar
  static double QY = 1;          // error cov
  static double PY = 0;          // initial error cov
  static double u_hatY = 0;      // initial estimation
  static double kY = 0;          // initial Kalman gain


  kY = PY * HY / (HY * PY * HY + RY);         // update kalman gain
  u_hatY = u_hatY + kY * (uY - HY * u_hatY);  // update estimation
  PY = (1 - kY * HY) * PY + QY;               // update error cov.

  return u_hatY;
}

float kalmanz(float uz) {

  // constatns
  static const double Rz = 10;   // noise cov
  static const double Hz = 1.0;  // measurement map scalar
  static double Qz = 1;          // error cov
  static double Pz = 0;          // initial error cov
  static double u_hatz = 0;      // initial estimation
  static double kz = 0;          // initial Kalman gain


  kz = Pz * Hz / (Hz * Pz * Hz + Rz);         // update kalman gain
  u_hatz = u_hatz + kz * (uz - Hz * u_hatz);  // update estimation
  Pz = (1 - kz * Hz) * Pz + Qz;               // update error cov.

  return u_hatz;
}
void read_mpu_6050_data() {
  // Subroutine for reading the raw data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  // Read data --> Temperature falls between acc and gyro registers
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure the accelerometer
  // Wire.write(0x__);
  // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();

  // Configure the gyro
  // Wire.write(0x__);
  // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}