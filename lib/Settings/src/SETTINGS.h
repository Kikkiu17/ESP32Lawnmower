#ifndef _SETTINGS_H_
#define _SETTINGS_H_
#include <stdint.h>

#define USING_USB_SERIAL true

/* #region  Pin diretti ESP */
#define RPM_SENS 5

#define RUNNING_LED 12
#define ERROR_LED 14

// HIGH: FWD, LOW: BCK
#define MOTOR_RIGHT_DIR 27
#define MOTOR_LEFT_DIR 26
#define MOT_R_SPD 25
#define MOT_L_SPD 33
#define MOT_MAIN 19

// Sensori velocità
#define LEFT_SPD_SENS 5
#define RIGHT_SPD_SENS 13

#define MUX_COM 32
#define MUX0 16
#define MUX1 4
#define MUX2 0
#define MUX3 2
#define INTERRUPT_PIN 34
// #define MUX_ENABLE 17 - NC in PCB
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
#define BAT 8
/* #endregion */

/* #region  IMU */
#define ACCEL_X_ERROR 0.46
#define ACCEL_Y_ERROR 0.32
#define ACCEL_Z_ERROR 0.38

#define ACCEL_SENSITIVITY 0.15 //± questo valore
#define GYRO_SENSITIVITY 0.02  //± questo valore

#define INVERT_ACC_X false
#define INVERT_ACC_Y false
#define INVERT_ACC_Z false
#define INVERT_YAW true
#define INVERT_PITCH false
#define INVERT_ROLL false
/* #endregion */

/* #region  Altre informazioni */
// #define ENABLE_WEBSERIAL
#define WHEEL_DIAMETER 162 // mm - old: 91
/* #endregion */

/* #region  Sensors.cpp */
#define ENABLE_OBSTACLE_AVOIDANCE true
#define ENABLE_MOVEMENT_SENSORS false
#define ENABLE_SPD_SENSORS true
#define ENABLE_BAT_VOLTAGE_SENSING false
/* #endregion */

/* #region  Navigation.cpp */
#define MPU_SPD_SENSORS_REFRESH_RATE 1  // ms
#define ENABLE_AUTO_NAVIGATION true     // necessita di ENABLE_MOVEMENT_SENSORS, ENABLE_ROTATION_SENSING, ENABLE_OBSTACLE_AVOIDANCE, ENABLE_SPD_SENSORS abilitati
#define ENABLE_ROTATION_SENSING true    // ferma il robot quando l'heading target viene raggiunto
#define ENABLE_ROTATION_LOOP true       // FEEDBACK LOOP - consente al modulo di navigazione di abbassare la velocità man mano si raggiunge la direzione target
#define LOG_OBSTACLES_TO_MAP false
#define LOG_MAP false
#define MAP_POINT_PROXIMITY_DST 200     // mm
/* #endregion */

/* #region  Nomi costanti codice */
#define FORWARD_NO_VARIABLE_UPDATE 1 // default: 0 = forward normale
#define CHANNEL_LEFT 1
#define CHANNEL_RIGHT 2
#define CHANNEL_MAIN 3
#define FWD 0
#define BCK 1
#define MOT_MAX_VAL 255
#define MOT_NORM_VAL 200
#define MOT_MIN_VAL 50
#define MOVEMENT_MOT_FREQ 50
#define MAIN_MOT_FREQ 150
#define US_SENS_DST_TRIG 588 // 10 cm; 10 / 0.017 = 588
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
#define NOT_FOUND 2147483646
#define MAX_CM 2147483646
// mappa
#define ACCESSIBLE 0
// mappa
#define OBSTACLE 1
// mappa
#define BORDER 2
#define AUTO 2147748
#define NONE 2147749
#define ALL 2147750
/* #endregion */

#define SHOW_MODULE_EXECUTION_TIME false
#define SHOW_HEAP_INFO false                // viene considerato solo se SHOW_MODULE_EXECUTION_TIME è true
#define MODULE_EXECUTION_TIME_REFRESH 250   // ms
#define ENABLE_LOGGING                      // commentare per disabilitare il logging delle informazioni nella porta seriale
#define USE_SD true

// per aumentare stack size loopTask (ora a 16384 byte) vedere \.platformio\packages\framework-arduinoespressif32\tools\sdk\include\config\sdkconfig.h
#endif
