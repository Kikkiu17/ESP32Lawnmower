#ifndef _SETTINGS_H_
#define _SETTINGS_H_
#include <stdint.h>

#define USING_USB_SERIAL true

// --- PIN ESP ---
#define RUNNING_LED 18
#define ERROR_LED 8
#define REF_BAT 1

// HIGH: FWD, LOW: BCK
#define MOT_R_CTRL1 48
#define MOT_R_CTRL2 45
#define MOT_L_CTRL1 21
#define MOT_L_CTRL2 47
#define MOT_MAIN 13

#define MUX_COM 2
#define MUX0 38
#define MUX1 37
#define MUX2 36
#define MUX3 35
// #define MUX_ENABLE 17 - NC in PCB

// --- CANALI MUX ---
// 0 inutilizzato - rotto sulla PCB
#define US_ECHO_L 1
#define US_TRIG_L 2
#define US_ECHO_F 3
#define US_TRIG_F 4
#define US_ECHO_R 5
#define US_TRIG_R 6

// --- IMU ---
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

// --- GENERALE ---
// #define ENABLE_WEBSERIAL
#define WHEEL_DIAMETER 159 // mm - old: 91
/* #endregion */

// --- SENSORS.CPP ---
#define ENABLE_OBSTACLE_AVOIDANCE true
#define ENABLE_MOVEMENT_SENSORS false
#define ENABLE_SPD_SENSORS true
#define ENABLE_BAT_VOLTAGE_SENSING false    // se false, il robot non si fermerà quando la batteria è scarica
#define MPU_CALIBRATION_TIME 30             // secondi

// --- NAVIGATION.CPP ---
#define MPU_SPD_SENSORS_REFRESH_RATE 2      // ms
#define ENABLE_AUTO_NAVIGATION true         // necessita di ENABLE_MOVEMENT_SENSORS, ENABLE_ROTATION_SENSING, ENABLE_OBSTACLE_AVOIDANCE, ENABLE_SPD_SENSORS abilitati
#define ENABLE_ROTATION_SENSING true        // ferma il robot quando l'heading target viene raggiunto
#define ENABLE_ROTATION_LOOP true           // FEEDBACK LOOP - consente al modulo di navigazione di abbassare la velocità man mano si raggiunge la direzione target
#define LOG_OBSTACLES_TO_MAP false
#define LOG_MAP false
#define MAP_POINT_PROXIMITY_DST 200         // mm

// --- QUEUESYS.CPP ---
#define MAX_QUEUE_SIZE 50
#define MAX_DATA_SIZE_PER_COMMAND 5

// --- COSTANTI ---
#define FORWARD_NO_VARIABLE_UPDATE 1 // default: 0 = forward normale
#define CH_R1 1
#define CH_R2 2
#define CH_L1 3
#define CH_L2 4
#define CHANNEL_MAIN 5
#define FWD 0
#define BCK 1
#define MOT_MAX_VAL 4095
#define MOT_NORM_VAL 2048
#define MOT_MIN_VAL 2048
#define MOVEMENT_MOT_FREQ 50
#define MAIN_MOT_FREQ 2000
#define US_SENS_DST_TRIG 883 // 10 cm; 10 / 0.017 = 588
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
// mappa
#define IGNORE 3
#define AUTO 2147748
#define NONE 2147749
#define ALL 2147750

#define SHOW_MODULE_EXECUTION_TIME false
#define SHOW_HEAP_INFO false                // viene considerato solo se SHOW_MODULE_EXECUTION_TIME è true
#define MODULE_EXECUTION_TIME_REFRESH 250   // ms
#define ENABLE_LOGGING                      // commentare per disabilitare il logging delle informazioni nella porta seriale
#define USE_SD true

// per aumentare stack size loopTask (ora a 16384 byte) vedere \.platformio\packages\framework-arduinoespressif32\tools\sdk\include\config\sdkconfig.h
#endif
