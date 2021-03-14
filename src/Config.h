/* ================================================================== ======================
  Define controller constants */

#define kp 2                               // Altitude PID proportional constant 
#define ki 0                              // Altitude PID integral constant
#define kd 0.01                          // Altitude PID derivative constant
#define filtAlt 0.95                    // Altitude Estimation Filter constant
#define LQRmult 0.475                  // Scaling factor for control law, varies between 0.5-1
#define LQR_P 0.1                     // LQR_P constant
#define LQR_E 1.75                   // Integrator 
#define INTEGRATOR_CLAMP 0.15       // Clamping term for integrator #define filtPID 0.95 
#define SLEW_LIMIT 10              // controller gimbal limit
#define SLEW_FILTER 0.85          // Controller rate limiter (0-1), higher = slower/stabler
#define D_COMP 0.075             // Coupling term for derivative. sets the derivative setpoints as a function of positional error
#define V_SPD 0.01              // Vertical speed reduction rate 

/* ================================================================== =======================
  Define communication setup */

#ifndef RADIO_BAUDRATE
#define RADIO_BAUDRATE 57600    // Telemetry radio baudrates (use 57600)
#endif

#define LIDAR_BAUDRATE 115200   // LiDAR sensor UART speed. default is 115200

#ifndef USB_BAUDRATE
#define USB_BAUDRATE 115200     // USB Serial port baudtate. (N/A for USB mode)
#endif

#ifndef SERIAL_USB
#define SERIAL_USB Serial       // Serial port for USB communication (always Serial)
#endif

#ifndef RADIO_SERIAL
#define RADIO_SERIAL Serial4    // Serial port for radio communication
#endif

#ifndef LIDAR_SERIAL
#define LIDAR_SERIAL Serial1    // Serial port for LiDAR sensor
#endif


#define CMD_SERIAL Serial       // Listen port for waypoints
#define TELEMETRY1 Serial4
#define TELEMETRY2 Serial


/* ================================================================== =======================
  Define motor setup */

#define ESC1 6       // Pin for ESC1
#define ESC2 7       // Pin for ESC2
#define ESC3 5       // Pin for ESC3
#define ESC4 4       // Pin for ESC4
#define MAXVAL 1500
#define MINVAL 900

/* Set the delay between iterations */
#define MAIN_DELAY 1

/* Define LQR gain matrix.
The rows define the inputs (U) to each Output value
The columns are the Full state (X) of the Vehicle
The linear math here is: 
                X' = AX 
                + K(R-X) 
                + K(R-Xint)

The Expansion of U is:
                        U[0] = total vehicle thrust
                        U[1] = moment about the X axis
                        U[2] = moment about the y axis
                        U[3] = moment about the z axis

So to "tune" and axis, say x, find the corresponding row, 
in this case the second row, and alter the values 
to choose how the moment about that axis is related to 
the respective columns of the vehicles state. (Curently the values
represent response to angular position about X, and angular speed about X)
*/

double K [4][6] = {{
    0, 0, 0, 0, 0, 0,
  }, {
    17.5, 0, 0, -100, 0, 0,
  }, {
    0, 17.5, 0, 0, -100, 0,
  }, {
    0, 0, 5, 0, -10, 0,
  },
};