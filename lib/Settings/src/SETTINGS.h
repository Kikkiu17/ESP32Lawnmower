/* #region  Pin LED di stato */
#define RUNNING_LED 12
#define ERROR_LED 14
/* #endregion */

/* #region  Pin motori */
#define MOTOR_RIGHT_DIR 27 // HIGH: FWD, LOW: BCK
#define MOTOR_LEFT_DIR 26  // HIGH: FWD, LOW: BCK
#define MOT_R_SPD 25
#define MOT_L_SPD 33

#define MOT_MAIN 19
/* #endregion */

#define RPM_SENS 5

/* #region  Canali MUX */
#define IR_FWD_CH 8
#define US_ECHO_CH 9
#define US_TRIG_CH 10
#define IR_LEFT_CH 11
/* #endregion */

/* #region  Pin MUX */
#define MUX_COM 32
#define MUX0 16
#define MUX1 4
#define MUX2 0
#define MUX3 2
#define MUX_ENABLE 17
/* #endregion */

/* #region  Valori di errore e di sensibilità giroscopio e accelerometro */
#define ACCEL_X_ERROR 0.46
#define ACCEL_Y_ERROR 0.32
#define ACCEL_Z_ERROR 0.38

#define ACCEL_SENSITIVITY 0.15 //± questo valore
#define GYRO_SENSITIVITY 0.02  //± questo valore
/* #endregion */

/* #region  Altre informazioni */
#define ENCODER_TEETH 14
#define WHEEL_DIAMETER 91 // mm
#define ENABLE_OBSTACLE_AVOIDANCE true
#define ENABLE_MOVEMENT_SENSORS false
#define ENABLE_BORDERMODE false
// #define ENABLE_WEBSERIAL
#define GEAR_RATIO 0.2 // 1:5 = 0.2 | drive gear teeth : driven gear teeth
/* #endregion */

/* #region  Nomi costanti codice */
// default: 0 = forward normale
#define FORWARD_NO_VARIABLE_UPDATE 1

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
/* #endregion */

#define ENABLE_LOGGING // commentare per disabilitare il logging delle informazioni nella porta seriale
