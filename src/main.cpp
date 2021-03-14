#include <Arduino.h>
#include "SensorFunctions.h"

/* --------------------------------------------------------------------------------------
  Main Controller Code to run on Teensy 4.0
  Author: Daniel Hoven Date: 3/15/2021 Project: Senior Capstone
  --------------------------------------------------------------------------------------*/
/*= == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == = == == == == == == == =
       SETUP */

void set_liDAR() {
  LIDAR_SERIAL.write(0x42);
  LIDAR_SERIAL.write(0x57);
  LIDAR_SERIAL.write(0x02);
  LIDAR_SERIAL.write(0x00);
  LIDAR_SERIAL.write(0x00);
  LIDAR_SERIAL.write(0x00);
  LIDAR_SERIAL.write(0x01);
  LIDAR_SERIAL.write(0x06);
}

void calibrateESCs() {

  esc1.attach(ESC1);
  esc2.attach(ESC2);
  esc3.attach(ESC3);
  esc4.attach(ESC4);
  delay(100);
  esc1.write(30);
  esc2.write(30);
  esc3.write(30);
  esc4.write(30);
  delay(1000);

  for (int i = 30; i > 20; i--) {
    esc1.write(i);
    esc2.write(i);
    esc3.write(i);
  }
  for (int i = 20; i < 35; i++) {
    esc1.write(i);
    esc2.write(i);
    esc3.write(i);
    esc4.write(i);
    delay(150);
  }
  delay(2000);
  esc1.write(40);
  esc2.write(40);
  esc3.write(40);
  esc4.write(40);
  delay(2000);
}


void setup()

{
  /* Open Serial Ports*/

  SERIAL_USB.begin(250000);
  LIDAR_SERIAL.begin(115200);
  RADIO_SERIAL.begin(RADIO_BAUDRATE);
  delay(100);

  /* put liDAR in std. output mode */

  set_liDAR();

  while (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    SERIAL_USB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(200);
  }

  SERIAL_USB.print("IMU found\n");
  bno.setExtCrystalUse(true);

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 6; j++) {
      K[i][j] *= LQRmult;
    }
  }

  calibrateESCs();
  delay(500);

  get_IMU_sample();
  delay(10);

  get_IMU_sample();
  R[0] = roll;
  R[1] = pitch;
  R[2] = yaw;

  Rcal[0] = R[0];
  Rcal[1] = R[1];
  Rcal[2] = R[2];
  delay(500);
  bool check = false;
  while (!check) {
    get_Distance_sample();
    int t1 = liDARval;
    delay(100);
    get_Distance_sample();
    int t2 = liDARval;
    if (t1 == t2) {
      check = true;
    }
    //initAlt = (t1 + t2) / 2;
  }

}

/*-------------------------------------------------------------------------------
   main
  ------------------------------------------------------------------------------*/
void loop(void) {

  dt = micros() - t;
  t = micros();
  dt = dt / 1000000;


  get_Distance_sample();

  _lidar = (1 - filtAlt) * liDARval + filtAlt * liDARold;
  liDARold = _lidar;
  double _alt = _lidar * cos(roll) * cos(pitch);
  d_alt *= filtAlt;
  d_alt += (1 - filtAlt) * (_alt - alt) / dt;
  alt = _alt;


  get_IMU_sample();
  ELQR_calc();
 

  if (VERT_SPEED == true) {
    vertSpeedHold();
  }
  receiveData();

  if(millis()%50<5){
    printData();
  }

  // Make sure Vehicle isn't dying
  OS(); // Oh Sh*t method

  if ((TAKEOFF_FLAG) && (millis() > 2000)) {
    IntegralTracker();
  }

  if (count < 1000) {
    for (int i = 1; i < 4; i++) {
      Ucal[i] = U[i];
    }

  } else {
    if (!STOP_FLAG) {
      commandESCs();
    } else {
      STOP();
    }

  }
  count++;
  delay(MAIN_DELAY);
}
