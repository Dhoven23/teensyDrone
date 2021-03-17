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

struct Signal {
  float e1 = 0,
        e2 = 0,
        e3 = 0,
        e4 = 0,
        Ecal[4] = {0, 0, 0, 0}; 

  double U [4] = {IDLE_SPEED, 0, 0, 0},
         Ucal[4];
};

u_int16_t counter = 0;

volatile float liDARold = 0,
               _lidar = 0;

unsigned int checksum = 0,
             check2 = 0,
             check1,
             Xrot = 360,
             Yrot = 360;

volatile int liDARval = 0,
             strength = 0;

struct Euler{
  double x = 0,
         y = 0,
         z = 0;

  double dx = 0,
         dy = 0,
         dz = 0;

  double Hist[3][5];
};

struct State{
  double Full[6] = {0, 0, 0, 0, 0, 0},
         Integral[6],
         Old[6];
  double Error[6];
};

struct Altitude {
  double alt = 0,
         d_alt = 0,
         initAlt = 0;
};

struct Setpoint{
  double R[6] = {0,0,0,0,0,0};
  double Alt = 0;
  double Rcal[3] = {0, 0, 0};
  double verticalSpeed = 0;
};

char Dcode[3];

double dt = 0;

uint16_t Ncode;

String STATE = "STARTUP";

bool STOP_FLAG = false;
bool TAKEOFF_FLAG = true;
bool VERT_SPEED = false;


State X;              // Vehicle Full state struct
Euler euler;          // Euler angle struct
Setpoint setPoint;    // setpoint struct
Altitude altitude;    // altitude data
Signal signal;        // controller output data

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

  euler.x = (-setPoint.Rcal[0]) - atan2(2.0 * (q3 * q2 + q0 * q1), 1.0 - 2.0 * (q1 * q1 + q2 * q2)); //* (180/PI);
  euler.y = (-setPoint.Rcal[1]) + asin(2.0 * (q2 * q0 - q3 * q1));// * (180/PI);
  euler.z = (-setPoint.Rcal[2]) - atan2(2.0 * (q3 * q0 + q1 * q2), -1.0 + 2.0 * (q0 * q0 + q1 * q1)); //* (180/PI);

  for (int i = 0; i < 6; i++) {
    X.Old[i] = X.Full[i];
  }

  X.Full[0] = euler.x;
  X.Full[1] = euler.y;
  X.Full[2] = euler.z;

  // Compute Derivatives using 5 point stencil
  double h = dt;
  for (int i = 0; i < 3; i++) {
    for (int j = 1; j < 5; j++) {
      euler.Hist[i][j] = euler.Hist[i][j - 1];
    }
    euler.Hist[i][0] = X.Full[i];
    double temp;
    temp = (-1) * euler.Hist[i][0] + (8) * euler.Hist[i][1] + (-8) * euler.Hist[i][3] + (1) * euler.Hist[i][4];
    temp /= 12 * h; 
    X.Full[i + 3] *= DERIVATIVE_FILT;
    X.Full[i + 3] += (1-DERIVATIVE_FILT) * temp;
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
    setPoint.R[i + 3] = D_COMP * (X.Full[i] - setPoint.R[i]);
  }
}

FASTRUN void ELQR_calc() {

  for (int i = 1; i < 4; i++) {
    double iter = 0;
    for (int j = 0; j < 3; j++) {
      iter += K[i][j] * ((X.Full[j] - setPoint.R[j]) + LQR_E * (X.Integral[j]));
    }
    for (int j = 3; j < 6; j++) {
      iter += K[i][j] * ((X.Full[j] - setPoint.R[j]) + LQR_P * (X.Integral[j]));
    }
    if (abs(iter) < SLEW_LIMIT) {
      signal.U[i] = iter;
      signal.U[i] -= signal.Ucal[i];
    }
    else if (iter > 0) {
      signal.U[i] = SLEW_LIMIT;
    }
    else if (iter < 0) {
      signal.U[i] = -SLEW_LIMIT;
    }

  }
}

FASTRUN void IntegralTracker() {

  for (int i = 0; i < 3; i++) {
    double iter = (dt / 2) * (X.Old[i] + X.Full[i]);
    if ((abs(iter) + abs(X.Integral[i])) < INTEGRATOR_CLAMP) {
      X.Integral[i] += iter;
      X.Integral[i + 3] = X.Full[i];
    } else if ((iter + X.Integral[i]) < -INTEGRATOR_CLAMP) {
      X.Integral[i] += 0.01;
      X.Integral[i + 3] = X.Full[i];
    } else if ((iter + X.Integral[i]) > INTEGRATOR_CLAMP) {
      X.Integral[i] -= 0.01;
      X.Integral[i + 3] = X.Full[i];
    }
  }
  for (int i = 0; i < 3; i++) {
    if (abs(X.Integral[i] - setPoint.R[i]) > INTEGRATOR_CLAMP) {
      if ((X.Integral[i] - setPoint.R[i]) > 0) {
        X.Integral[i] = INTEGRATOR_CLAMP + setPoint.R[i] - 0.01;
      } else if ((X.Integral[i] - setPoint.R[i]) < 0) {
        X.Integral[i] = -(INTEGRATOR_CLAMP + setPoint.R[i]) + 0.01;
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
    if (abs(X.Full[i]) > PI / 6) {
      check = true;
    }
  }
  return check;
}

void STOP() {
  Serial.println("------ CRASH CONDITION DETECTED!! -------");
  signal.e1 = 0;
  signal.e2 = 0;
  signal.e3 = 0;
  signal.e4 = 0;
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
  double ERR = setPoint.verticalSpeed - altitude.d_alt;
  signal.e1 += ERR * V_SPD;
  signal.e2 += ERR * V_SPD;
  signal.e3 += ERR * V_SPD;
  signal.e4 += ERR * V_SPD;
}

void commandESCs() {
  // Motor Mixing Algorithm

  float _e1 = signal.U[0] - signal.U[1] + signal.U[2] + signal.U[3];
  float _e2 = signal.U[0] - signal.U[1] - signal.U[2] - signal.U[3];
  float _e3 = signal.U[0] + signal.U[1] - signal.U[2] + signal.U[3];
  float _e4 = signal.U[0] + signal.U[1] + signal.U[2] - signal.U[3];

  _e1 = map(_e1, 0, 180, 1000, 2000);
  _e2 = map(_e2, 0, 180, 1000, 2000);
  _e3 = map(_e3, 0, 180, 1000, 2000);
  _e4 = map(_e4, 0, 180, 1000, 2000);

  signal.e1 *= (SLEW_FILTER);
  signal.e2 *= (SLEW_FILTER);
  signal.e3 *= (SLEW_FILTER);
  signal.e4 *= (SLEW_FILTER);

  signal.e1 += (1 - SLEW_FILTER) * _e1;
  signal.e2 += (1 - SLEW_FILTER) * _e2;
  signal.e3 += (1 - SLEW_FILTER) * _e3;
  signal.e4 += (1 - SLEW_FILTER) * _e4;

  if ((signal.Ecal[0] + signal.Ecal[1] + signal.Ecal[2] + signal.Ecal[3]) == 0) {

    float eAv = (signal.e1 + signal.e2 + signal.e3 + signal.e4) / 4;
    signal.Ecal[0] = eAv - signal.e1;
    signal.Ecal[1] = eAv - signal.e2;
    signal.Ecal[2] = eAv - signal.e3;
    signal.Ecal[3] = eAv - signal.e4;
  }

  signal.e1 += signal.Ecal[0];
  signal.e2 += signal.Ecal[1];
  signal.e3 += signal.Ecal[2];
  signal.e4 += signal.Ecal[3];

  if ((signal.e1 < MAXVAL) && (signal.e1 > MINVAL)) {
    esc1.writeMicroseconds((int)signal.e1);
  } else if (signal.e1 < MINVAL) {
    signal.e1 = MINVAL + 1;
  } else if (signal.e1 > MAXVAL) {
    signal.e1 = MAXVAL - 1;
  }

  if ((signal.e2 < MAXVAL) && (signal.e2 > MINVAL)) {
    esc2.writeMicroseconds((int)signal.e2);
  } else if (signal.e2 < MINVAL) {
    signal.e2 = MINVAL + 1;
  } else if (signal.e2 > MAXVAL) {
    signal.e2 = MAXVAL - 1;
  }
  if ((signal.e3 < MAXVAL) && (signal.e3 > MINVAL)) {
    esc3.writeMicroseconds((int)signal.e3);
  } else if (signal.e3 < MINVAL) {
    signal.e3 = MINVAL + 1;
  } else if (signal.e3 > MAXVAL) {
    signal.e3 = MAXVAL - 1;
  }
  if ((signal.e4 < MAXVAL) && (signal.e4 > MINVAL)) {
    esc4.writeMicroseconds((int)signal.e4);
  } else if (signal.e4 < MINVAL) {
    signal.e4 = MINVAL + 1;
  } else if (signal.e4 > MAXVAL) {
    signal.e4 = MAXVAL - 1;
  }
}

void printData() {

  TELEMETRY1.print(millis());
  TELEMETRY1.print(", 1:,"), TELEMETRY1.print((int)signal.e1);
  TELEMETRY1.print(", 2:,"), TELEMETRY1.print((int)signal.e2);
  TELEMETRY1.print(", 3:,"), TELEMETRY1.print((int)signal.e3);
  TELEMETRY1.print(", 4:,"), TELEMETRY1.print((int)signal.e4);

  TELEMETRY1.print(", roll:,"), TELEMETRY1.print(X.Full[0], 3);
  TELEMETRY1.print(", pirch:,"), TELEMETRY1.print(X.Full[1], 3);
  TELEMETRY1.print(", yaw:,"), TELEMETRY1.print(X.Full[2], 3);
  TELEMETRY1.print(", "), TELEMETRY1.print(X.Full[3], 3);
  TELEMETRY1.print(", "), TELEMETRY1.print(X.Full[4], 3);
  TELEMETRY1.print(", "), TELEMETRY1.print(X.Full[5], 3);
  TELEMETRY1.print("Alt:,"), TELEMETRY1.print(altitude.alt);
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
          setPoint.R[0] += 0.01;
          digitalWrite(13, HIGH);
        } else if (temp == 'w') {
          Dcode[0] = '+', Dcode[1] = 'y';
          setPoint.R[1] += 0.01;
          digitalWrite(13, HIGH);
        } else if (temp == 's') {
          Dcode[0] = '-', Dcode[1] = 'y'; 
          setPoint.R[1] -= 0.01;
          digitalWrite(13, HIGH);
        } else if (temp == 'd') {
          Dcode[0] = '+', Dcode[1] = 'x';
          setPoint.R[0] -= 0.01;
          digitalWrite(13, HIGH);
        } else if (temp == 'q') {
          signal.U[3] += 1;
        } else if (temp == 'e') {
          signal.U[3] -= 1;
        } else if (temp == 'V') {
          VERT_SPEED = !VERT_SPEED;
        } else if (temp == 'I') {
          TAKEOFF_FLAG = !(TAKEOFF_FLAG);
        } else if (temp == 'H') {
          Dcode[0] = 'H',
                     Dcode[1] = 'O';
          setPoint.R[0] = setPoint.Rcal[0];
          setPoint.R[1] = setPoint.Rcal[1];
          STOP_FLAG = false;

          digitalWrite(13, HIGH);
        } else if (temp == 'Y') {
          STOP_FLAG = false;
        } else if (temp == 'r') {
          Dcode[0] = '+',
                     Dcode[1] = 'z';
          signal.U[0] += t1;
          VERT_SPEED = false;
        } else if (temp == 'f') {
          Dcode[0] = '-',
                     Dcode[1] = 'z';
                                signal.U[0] -= t1;
          VERT_SPEED = false;

          digitalWrite(13, HIGH);
        } else if (temp == 'K') {
          signal.U[0] = 0;
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