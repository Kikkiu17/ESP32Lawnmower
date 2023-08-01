#ifndef _SETTINGS_H_
#define _SETTINGS_H_
#include <stdint.h>

#define USING_USB_SERIAL true

/////////////////////////////////// PIN ESP ///////////////////////////////////
#define RUNNING_LED 18
#define ERROR_LED 8
#define REF_BAT 1

// HIGH: FWD, LOW: BCK
#define MOT_R_CTRL1 45
#define MOT_R_CTRL2 48
#define MOT_L_CTRL1 21
#define MOT_L_CTRL2 47
#define MOT_MAIN 13

#define MUX_COM 2
#define MUX0 38
#define MUX1 37
#define MUX2 36
#define MUX3 35
// #define MUX_ENABLE 17 - NC in PCB
//////////////////////////////////////////////////////////////////////



/////////////////////////////////// CANALI MUX///////////////////////////////////
// 0 inutilizzato - disconnesso sulla PCB
#define US_ECHO_L 1
#define US_TRIG_L 2
#define US_ECHO_F 3
#define US_TRIG_F 4
#define US_ECHO_R 5
#define US_TRIG_R 6
//////////////////////////////////////////////////////////////////////



/////////////////////////////////// GENERALE ///////////////////////////////////
// #define ENABLE_WEBSERIAL
#define WHEEL_DIAMETER 159.0                // mm
#define WHEEL_RADIUS 79.5                   // mm; WHEEL_DIAMETER / 2.0
#define ROBOT_WIDTH 298.0                   // mm
#define ROBOT_ROTATION_RADIUS 149           // mm; ROBOT_WIDTH / 2.0
//////////////////////////////////////////////////////////////////////



/////////////////////////////////// IMU ///////////////////////////////////
#define INVERT_ACC_X false
#define INVERT_ACC_Y false
#define INVERT_ACC_Z false
#define INVERT_YAW false
#define INVERT_PITCH false
#define INVERT_ROLL false

// YAW COMPENSATION
#define YAW_COMP_START_VALUE -0.60
#define YAW_COMP_DRIFT 0.0047
#define YAW_COMP_LOOP 1000  // ms
//////////////////////////////////////////////////////////////////////



/////////////////////////////////// SENSORS.CPP ///////////////////////////////////
#define ENABLE_OBSTACLE_AVOIDANCE true
#define ENABLE_MOT_ENCODERS true
#define ENABLE_BAT_VOLTAGE_SENSING true     // se false, il robot non si fermerà quando la batteria è scarica
#define MOV_MOTOR_GEAR_RATIO 200.0          // reduction ratio dal motore elettrico all'uscita del riduttore
#define RPM_SENSOR_MAX_VALUE 40.0           // difficilmente i motori arriveranno a 40 RPM con massimo 14 V
#define DEFAULT_BAT_CHECK_TIME 500          // ms
#define INACTIVE_TIME_THRESHOLD (uint32_t)(/* ms: */600000 / DEFAULT_BAT_CHECK_TIME)   // tick; 10 minuti
#define TIME_BETW_INACTIVE_BEEPS (/* ms: */3000 - DEFAULT_BAT_CHECK_TIME)
//////////////////////////////////////////////////////////////////////



/////////////////////////////////// NAVIGATION.CPP ///////////////////////////////////
#define MPU_MOT_ENCODERS_REFRESH_RATE 2     // ms
#define ENABLE_AUTO_NAVIGATION true         // necessita di ENABLE_ROTATION_SENSING, ENABLE_OBSTACLE_AVOIDANCE, ENABLE_MOT_ENCODERS abilitati
#define ENABLE_ROTATION_SENSING true        // ferma il robot quando l'heading target viene raggiunto
#define ENABLE_ROTATION_LOOP false          // FEEDBACK LOOP - consente al modulo di navigazione di abbassare la velocità man mano si raggiunge la direzione target
#define LOG_OBSTACLES_TO_MAP false          // se i sensori frontali rilevano un ostacolo in qualsiasi momento, lo scrivono nella mappa
#define LOG_MAP false
#define MAP_POINT_PROXIMITY_DST 200         // mm
// -> MAP PROCESSING
#define MAP_RAM_USAGE_PRCTG 90              // % di utilizzo della PSRAM per processare la mappa
#define SD_RAM_USAGE_SAFETY_BUFFER 35000    // byte di sicurezza da lasciare riservati per l'operazione dell'SD
// -> MAP SETTINGS
#define MAP_POINT_SIZE 6                    // sizeof(Point) (byte)
#define MAP_BLOCK_SIZE 1536                 // MAP_POINT_SIZE * 256 (byte)
#define POINTS_PER_BLOCK 256
#define MAP_SQUARE_WIDTH 256
//////////////////////////////////////////////////////////////////////


/////////////////////////////////// COSTANTI ///////////////////////////////////
#define FORWARD_NO_VARIABLE_UPDATE 1        // default: 0 = forward normale
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
#define US_SENS_DST_TRIG 883                // 15 cm; 15 / 0.017 = 883
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
#define RUN 15
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
//////////////////////////////////////////////////////////////////////

// per aumentare stack size loopTask (ora a 16384 byte) vedere \.platformio\packages\framework-arduinoespressif32\tools\sdk\include\config\sdkconfig.h
#endif
