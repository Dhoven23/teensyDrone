#include "Config.h"
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Servo.h>

/* ================================================================== =======================
  Declare globals */

int cycletime = 0,
    count = 0,
    pos = 0,
    t = 0,
    i = 0,
    E_old,
    tim_old = 0;

float e1 = 0,
      e2 = 0,
      e3 = 0,
      e4 = 0,
      Ecal[4] = {0, 0, 0, 0};

uint16_t counter = 0;

volatile float liDARold = 0,
               _lidar = 0;

unsigned int checksum = 0,
             check2 = 0,
             check1,
             altSet = 20,
             Xrot = 360,
             Yrot = 360;

volatile int liDARval = 0,
             strength = 0;

double pitch,
       roll,
       yaw,
       d_roll,
       d_pitch,
       d_yaw,
       alt = 0,
       d_alt,
       dt = 0,
       I;

double EulerHist[3][5],
       initAlt = 10;

double X_Full [6] = {0, 0, 0, 0, 0, 0}, // RAM1 arrays
        R [6] = {0, 0, 0, 0, 0, 0},
        X_int [6],
        X_old [6];



char Dcode[3];

uint16_t Ncode;

String STATE = "STARTUP";

double U [4] = {20, 0, 0, 0},
       Ucal[4],
       Rcal[3] = {0, 0, 0};

bool STOP_FLAG = false;
bool TAKEOFF_FLAG = true;
bool VERT_SPEED = false;


/* ================================================================== ======================
  Declare Library Objects

*/

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Servo esc1,
      esc2,
      esc3,
      esc4;


/*-------------------------------------------------------------------------
   Write external functions
  ---------------------------------------------------------------------------*/
FASTRUN void get_IMU_sample() {

  /* get quaternions */

  imu::Quaternion quat = bno.getQuat();

  volatile double q0 = quat.w();
  volatile double q1 = quat.x();
  volatile double q2 = quat.y();
  volatile double q3 = quat.z();

  //quaternion conversion

  roll = (-Rcal[0]) - atan2(2.0 * (q3 * q2 + q0 * q1), 1.0 - 2.0 * (q1 * q1 + q2 * q2)); //* (180/PI);
  pitch = (-Rcal[1]) + asin(2.0 * (q2 * q0 - q3 * q1));// * (180/PI);
  yaw = (-Rcal[2]) - atan2(2.0 * (q3 * q0 + q1 * q2), -1.0 + 2.0 * (q0 * q0 + q1 * q1)); //* (180/PI);

  for (int i = 0; i < 6; i++) {
    X_old[i] = X_Full[i];
  }

  X_Full[0] = roll;
  X_Full[1] = pitch;
  X_Full[2] = yaw;

  // Compute Derivatives using 5 point stencil
  double h = dt;
  for (int i = 0; i < 3; i++) {
    for (int j = 1; j < 5; j++) {
      EulerHist[i][j] = EulerHist[i][j - 1];
    }
    EulerHist[i][0] = X_Full[i];
    double temp;
    temp = (-1) * EulerHist[i][0] + (8) * EulerHist[i][1] + (-8) * EulerHist[i][3] + (1) * EulerHist[i][4];
    temp /= 12 * h; X_Full[i + 3] = temp;
  }
}

void get_Distance_sample() {

  if (LIDAR_SERIAL.available() >= 9) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
  {
    if ((0x59 == LIDAR_SERIAL.read()) && (0x59 == LIDAR_SERIAL.read())) // byte 1and byte 2
    {
      unsigned int t1 = LIDAR_SERIAL.read(); // byte 3 = Dist_L
      unsigned int t2 = LIDAR_SERIAL.read(); // byte 4 = Dist_H
      t2 <<= 8;
      t2 += t1;
      liDARval = t2;
      t1 = LIDAR_SERIAL.read(); // byte 5 = Strength_L
      t2 = LIDAR_SERIAL.read(); // byte 6 = Strength_H
      t2 <<= 8;
      t2 += t1;
      strength = t2;
      for (int i = 0; i < 3; i++)LIDAR_SERIAL.read(); // ignore remaining bytes
    }
  }
}

FASTRUN void DerivativeComp() {
  for (int i = 0; i < 3; i++) {
    R[i + 3] = D_COMP * (X_Full[i] - R[i]);
  }
}

FASTRUN void ELQR_calc() {

  for (int i = 1; i < 4; i++) {
    double iter = 0;
    for (int j = 0; j < 3; j++) {
      iter += K[i][j] * ((X_Full[j] - R[j]) + LQR_E * (X_int[j]));
    }
    for (int j = 3; j < 6; j++) {
      iter += K[i][j] * ((X_Full[j] - R[j]) + LQR_P * (X_int[j]));
    }
    if (abs(iter) < SLEW_LIMIT) {
      U[i] = iter;
      U[i] -= Ucal[i];
    }
    else if (iter > 0) {
      U[i] = SLEW_LIMIT;
    }
    else if (iter < 0) {
      U[i] = -SLEW_LIMIT;
    }

  }
}

FASTRUN void IntegralTracker() {

  for (int i = 0; i < 3; i++) {
    double iter = (dt / 2) * (X_old[i] + X_Full[i]);
    if ((abs(iter) + abs(X_int[i])) < INTEGRATOR_CLAMP) {
      X_int[i] += iter;
      X_int[i + 3] = X_Full[i];
    } else if ((iter + X_int[i]) < -INTEGRATOR_CLAMP) {
      X_int[i] += 0.01;
      X_int[i + 3] = X_Full[i];
    } else if ((iter + X_int[i]) > INTEGRATOR_CLAMP) {
      X_int[i] -= 0.01;
      X_int[i + 3] = X_Full[i];
    }
  }
  for (int i = 0; i < 3; i++) {
    if (abs(X_int[i] - R[i]) > INTEGRATOR_CLAMP) {
      if ((X_int[i] - R[i]) > 0) {
        X_int[i] = INTEGRATOR_CLAMP + R[i] - 0.01;
      } else if ((X_int[i] - R[i]) < 0) {
        X_int[i] = -(INTEGRATOR_CLAMP + R[i]) + 0.01;
      }
    }
  }
}

bool goingToCrash() {
  return false;
}

bool upsideDown() {
  bool check = false;
  for (int i = 0; i < 2; i++) {
    if (abs(X_Full[i]) > PI / 6) {
      check = true;
    }
  }
  return check;
}

void STOP() {
  Serial.println("------ CRASH CONDITION DETECTED!! -------");
  e1 = 0;
  e2 = 0;
  e3 = 0;
  e4 = 0;
  esc1.write(30);
  esc2.write(30);
  esc3.write(30);
  esc4.write(30);
}
void OS() {
  if (upsideDown()) {
    STOP();
    STOP_FLAG = true;
  } else if (goingToCrash()) {
    STOP();
  } else {}
}

void vertSpeedHold() {
  double ERR = 0 - d_alt;
  e1 += ERR * V_SPD;
  e2 += ERR * V_SPD;
  e3 += ERR * V_SPD;
  e4 += ERR * V_SPD;
}

void commandESCs() {
  // Motor Mixing Algorithm

  float _e1 = U[0] - U[1] + U[2] + U[3];
  float _e2 = U[0] - U[1] - U[2] - U[3];
  float _e3 = U[0] + U[1] - U[2] + U[3];
  float _e4 = U[0] + U[1] + U[2] - U[3];

  _e1 = map(_e1, 0, 180, 1000, 2000);
  _e2 = map(_e2, 0, 180, 1000, 2000);
  _e3 = map(_e3, 0, 180, 1000, 2000);
  _e4 = map(_e4, 0, 180, 1000, 2000);

  e1 *= (SLEW_FILTER);
  e2 *= (SLEW_FILTER);
  e3 *= (SLEW_FILTER);
  e4 *= (SLEW_FILTER);

  e1 += (1 - SLEW_FILTER) * _e1;
  e2 += (1 - SLEW_FILTER) * _e2;
  e3 += (1 - SLEW_FILTER) * _e3;
  e4 += (1 - SLEW_FILTER) * _e4;

  if ((Ecal[0] + Ecal[1] + Ecal[2] + Ecal[3]) == 0) {

    float eAv = (e1 + e2 + e3 + e4) / 4;
    Ecal[0] = eAv - e1;
    Ecal[1] = eAv - e2;
    Ecal[2] = eAv - e3;
    Ecal[3] = eAv - e4;
  }

  e1 += Ecal[0];
  e2 += Ecal[1];
  e3 += Ecal[2];
  e4 += Ecal[3];

  if ((e1 < MAXVAL) && (e1 > MINVAL)) {
    esc1.writeMicroseconds((int)e1);
  } else if (e1 < MINVAL) {
    e1 = MINVAL + 1;
  } else if (e1 > MAXVAL) {
    e1 = MAXVAL - 1;
  }

  if ((e2 < MAXVAL) && (e2 > MINVAL)) {
    esc2.writeMicroseconds((int)e2);
  } else if (e2 < MINVAL) {
    e2 = MINVAL + 1;
  } else if (e2 > MAXVAL) {
    e2 = MAXVAL - 1;
  }
  if ((e3 < MAXVAL) && (e3 > MINVAL)) {
    esc3.writeMicroseconds((int)e3);
  } else if (e3 < MINVAL) {
    e3 = MINVAL + 1;
  } else if (e3 > MAXVAL) {
    e3 = MAXVAL - 1;
  }
  if ((e4 < MAXVAL) && (e4 > MINVAL)) {
    esc4.writeMicroseconds((int)e4);
  } else if (e4 < MINVAL) {
    e4 = MINVAL + 1;
  } else if (e4 > MAXVAL) {
    e4 = MAXVAL - 1;
  }
}


void printData() {

  //TELEMETRY1.print(millis());
  TELEMETRY1.print(", 1:,"), TELEMETRY1.print((int)e1);
  TELEMETRY1.print(", 2:,"), TELEMETRY1.print((int)e2);
  TELEMETRY1.print(", 3:,"), TELEMETRY1.print((int)e3);
  TELEMETRY1.print(", 4:,"), TELEMETRY1.print((int)e4);

  TELEMETRY1.print(", roll:,"), TELEMETRY1.print(X_Full[0], 3);
  TELEMETRY1.print(", pitch:,"), TELEMETRY1.print(X_Full[1], 3);
  TELEMETRY1.print(", yaw:,"), TELEMETRY1.print(X_Full[2], 3);
  TELEMETRY1.print(", "), TELEMETRY1.print(X_Full[3], 3);
  TELEMETRY1.print(", "), TELEMETRY1.print(X_Full[4], 3);
  TELEMETRY1.print(", "), TELEMETRY1.print(X_Full[5], 3);
  TELEMETRY1.print("Alt: ,"), TELEMETRY1.print(alt);
  if (TAKEOFF_FLAG) {
    TELEMETRY1.print(",Integrators ON, ");
  } else {
    TELEMETRY1.print(",Integrators OFF, ");
  }
  if (VERT_SPEED) {
    TELEMETRY1.print(", VERT_SPD_HOLD = true");
  } else {
    TELEMETRY1.print(", VERT_SPD_HOLD = false");
  }
  TELEMETRY1.println();
}


void receiveData() {

  if (CMD_SERIAL.available() >= 5) {

    if ((CMD_SERIAL.read() == 0x20) && (CMD_SERIAL.read() == 0x20)) {

      char temp = CMD_SERIAL.read();

      uint16_t t1 = CMD_SERIAL.read();
      uint16_t t2 = CMD_SERIAL.read();

      t2 <<= 8;
      t1 += t2;

      if (!((Dcode[0] == 'N') || (Dcode[1] == 'A'))) {
        Ncode = t1;

        if (temp == 'a') {
          Dcode[0] = '-', Dcode[1] = 'x';
          R[0] += 0.01;
          digitalWrite(13, HIGH);
        } else if (temp == 'w') {
          Dcode[0] = '+', Dcode[1] = 'y';
          R[1] += 0.01;
          digitalWrite(13, HIGH);
        } else if (temp == 's') {
          Dcode[0] = '-', Dcode[1] = 'y'; R[1] -= 0.01;
          digitalWrite(13, HIGH);
        } else if (temp == 'd') {
          Dcode[0] = '+',
                     Dcode[1] = 'x',
                                R[0] -= 0.01;
          digitalWrite(13, HIGH);
        } else if (temp == 'q') {
          U[3] += 1;
        } else if (temp == 'e') {
          U[3] -= 1;
        } else if (temp == 'V') {
          VERT_SPEED = !VERT_SPEED;
        } else if (temp == 'I') {
          TAKEOFF_FLAG = !(TAKEOFF_FLAG);
        } else if (temp == 'H') {
          Dcode[0] = 'H',
                     Dcode[1] = 'O';
          R[0] = Rcal[0];
          R[1] = Rcal[1];
          X_int[0] = 0;
          X_int[1] = 0;
          X_int[2] = 0;
          STOP_FLAG = false;

          digitalWrite(13, HIGH);
        } else if (temp == 'Y') {
          STOP_FLAG = false;
        } else if (temp == 'r') {
          Dcode[0] = '+',
                     Dcode[1] = 'z';
          U[0] += t1;
          VERT_SPEED = false;
        } else if (temp == 'f') {
          Dcode[0] = '-',
                     Dcode[1] = 'z',
                                U[0] -= t1;
          VERT_SPEED = false;

          digitalWrite(13, HIGH);
        } else if (temp == 'K') {
          U[0] = 0;
          STOP_FLAG = true;

        } else {
          Dcode[0] = 'N', Dcode[1] = 'A';
        }

      }
      digitalWrite(13, LOW);
    }
  } else {
    digitalWrite(13, LOW);
  }
}