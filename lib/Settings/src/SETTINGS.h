/* #region  Pin diretti ESP */
#define RPM_SENS 5
#define BAT 34

#define RUNNING_LED 12
#define ERROR_LED 14

// HIGH: FWD, LOW: BCK
#define MOTOR_RIGHT_DIR 27
#define MOTOR_LEFT_DIR 26
#define MOT_R_SPD 25
#define MOT_L_SPD 33
#define MOT_MAIN 19

#define MUX_COM 32
#define MUX0 16
#define MUX1 4
#define MUX2 0
#define MUX3 2
// #define MUX_ENABLE 17 NC in PCB
/* #endregion */

/* #region  Canali MUX */
#define US_ECHO_L 0
#define US_TRIG_L 1
#define US_ECHO_F 2
#define US_TRIG_F 3
#define US_ECHO_R 4
#define US_TRIG_R 5
#define IR_F 6
#define IR_L 7
/* #endregion */

/* #region  Giroscopio e accelerometro */
#define ACCEL_X_ERROR 0.46
#define ACCEL_Y_ERROR 0.32
#define ACCEL_Z_ERROR 0.38

#define ACCEL_SENSITIVITY 0.15 //± questo valore
#define GYRO_SENSITIVITY 0.02  //± questo valore

#define INVERT_ACC_X false
#define INVERT_ACC_Y true
#define INVERT_ACC_Z true
#define INVERT_YAW false
#define INVERT_PITCH false
#define INVERT_ROLL false
/* #endregion */

/* #region  Altre informazioni */
// #define ENABLE_WEBSERIAL

#define ENCODER_TEETH 14
#define WHEEL_DIAMETER 91 // mm

#define ENABLE_OBSTACLE_AVOIDANCE false
#define ENABLE_MOVEMENT_SENSORS false
#define ENABLE_ROTATION_SENSING false
#define ENABLE_ENCODER false
#define ENABLE_AUTO_NAVIGATION false
#define ENABLE_BORDERMODE false
#define GEAR_RATIO 0.2 // 1:5 = 0.2 | drive gear teeth : driven gear teeth
/* #endregion */

/* #region  Nomi costanti codice */
#define FORWARD_NO_VARIABLE_UPDATE 1 // default: 0 = forward normale
#define CHANNEL_LEFT 1
#define CHANNEL_RIGHT 2
#define CHANNEL_MAIN 3
#define FWD 1
#define BCK 0
#define MOT_BASE_VAL 255
#define MOT_MIN_VAL 150
#define US_SENS_DST_TRIG 10 // cm
#define GLOBAL_NAV_DELAY 150
#define FRONT 0
#define BACK 1
#define LEFT 2
#define RIGHT 3
#define ULTRASONIC 4
#define INFRARED 5
#define BOTH 6
#define MOTORS 8
#define WRITE 9
#define READ 10
#define MAIN 11
#define RUNNING 12
#define STOP 13
#define TOGGLE 14
/* #endregion */

#define ENABLE_LOGGING // commentare per disabilitare il logging delle informazioni nella porta seriale
