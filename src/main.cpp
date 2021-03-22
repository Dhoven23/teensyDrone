#include <Arduino.h>
#include "SensorFunctions.h"

/* --------------------------------------------------------------------------------------
  Main Controller Code to run on Teensy 4.0
  Author: Daniel Hoven
  Project: Senior Capstone
  --------------------------------------------------------------------------------------*/
/*= == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == = == == == == == == == =
SETUP 
*/

struct Global
{
  long int iterations = 0;
  int tMicros = 0;
  double dt = 0;

  String STATE = "STARTUP";

  bool STOP_FLAG = false,
       TAKEOFF_FLAG = true,
       VERT_SPEED = false;
};

Global global;

void setup()

{
  /* Open Serial Ports*/

  SERIAL_USB.begin(250000);
  LIDAR_SERIAL.begin(115200);
  RADIO_SERIAL.begin(RADIO_BAUDRATE);
  delay(100);

  /* put liDAR in std. output mode */

  set_liDAR();

  while (!bno.begin())
  {
 
    SERIAL_USB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(200);
  }

  SERIAL_USB.print("IMU found\n");
  bno.setExtCrystalUse(true);

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      K[i][j] *= LQRmult;
    }
  }

  calibrateESCs();
  delay(500);

  get_IMU_sample(global.dt, global.iterations);
  delay(10);

  get_IMU_sample(0.1, 0.1);
  setPoint.R[0] = euler.x;
  setPoint.R[1] = euler.y;
  setPoint.R[2] = euler.z;

  setPoint.Rcal[0] = setPoint.R[0];
  setPoint.Rcal[1] = setPoint.R[1];
  setPoint.Rcal[2] = setPoint.R[2];
  delay(500);

}

void loop(void)
{

  global.dt = micros() - global.tMicros;
  global.tMicros = micros();
  global.dt = global.dt / 1000000;

  get_IMU_sample(global.dt, global.iterations),
      get_Distance_sample(global.dt),
      ELQR_calc();

  if (VERT_SPEED == true)
  {
    vertSpeedHold();
  }
  receiveData();

  if (global.iterations % 5 == 0)
  {
    printData();
  }

  // Make sure Vehicle isn't dying
  OS(); // Oh Sh*t method

  if ((TAKEOFF_FLAG) && (millis() > 2000))
  {
    IntegralTracker(global.dt);
  }

  if (global.iterations < 1000)
  {
    for (int i = 1; i < 4; i++)
    {
      signal.Ucal[i] = signal.U[i];
    }
  }
  else
  {
    if (!STOP_FLAG)
    {
      commandESCs();
    }
    else
    {
      STOP();
    }
  }

  global.iterations++;

  delay(MAIN_DELAY);
}
