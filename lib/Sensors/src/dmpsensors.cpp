#include <Sensors.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Status.h>
#include <Motors.h>
#include <Core.h>
#include <Navigation.h>
#include <BluetoothSerial.h>
#include <Mux.h>
#ifdef ENABLE_WEBSERIAL
#include <WebSerial.h>
#endif

// classi librerie
Motors sensormotors;
Status sensorstatus;
MPU6050 mpu;
Core sensorscore;
NAV sensorsnav;
BluetoothSerial sensorserial;
Mux sensormux;
uint64_t t2 = 0;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
// #define INTERRUPT_PIN 4 // usato da S1 (MUX) in PCB

// variabili
uint16_t forward_filter_acc_data = 0;
uint16_t backwards_filter_acc_data = 0;
uint8_t times_acc_data_is_gathered = 0;
float avg_acc = 0;
bool filtering_active = false;

bool motors_rotating = false;
bool motors_rotating_check = false;
uint32_t time3 = millis();

uint32_t time1_movement_sensor = 0;
bool go_once_movement_sensor = false;

bool allow_second_edge = false;
bool allow_rising_edge = false;
unsigned long time1 = 0;
unsigned long time2 = 0;
// float RPS_REQUIRED_PERIOD = 1000 / encoder_TEETH; // deprecated

int movement_checked = 0;
bool check_movement = false;
int not_moving_warnings = 0;
char movement_check_axis;
const int movements_to_check = 100;
int total_acceleration = 0;
int stop_acc_1 = 0;
int stop_acc_2 = 0;
bool sensor_autorun = false;

bool check_rotation = false;
int32_t last_heading = 0;
int32_t last_heading_stop_check = 0;
int rotation_checked = 0;
bool not_rotating = false;
const int rotations_to_check = 350;

bool US_front_obstacle_detected = false;

bool moving = false;

int32_t accX = 0;
int32_t accY = 0;
int32_t accZ = 0;
float yaw = 0;
float pitch = 0;
float roll = 0;

int32_t zeroX = 0;
int32_t zeroY = 0;
int32_t zeroZ = 0;
float zeroP = 0;
float zeroR = 0;
float zeroW = 0; // yaW, heading

float maxX = 0;
float maxY = 0;
float maxZ = 0;

float minX = 0;
float minY = 0;
float minZ = 0;

uint8_t stop_sensor_direction = FRONT;
uint8_t stop_sensor_type = INFRARED;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

struct Ultrasonic_Sensor
{
    bool get_front_obstacle = false;
    bool FRONT_checked = false;
    bool RIGHT_checked = false;
    bool LEFT_checked = false;
    bool check_next = false;
    uint8_t request_type = DEFAULT;
    uint32_t time_ref = 0;
    bool enable = false;
};

struct Ultrasonic_request
{
    bool active = false;
    bool returned = false;
    uint8_t sensor_direction = FRONT;
    float distance_measured = false;
    uint8_t request_type = DEFAULT;
    bool confirm_distance = false;
};

struct Infrared_Sensor
{
    bool enable = false;
    uint32_t timer = millis();
    bool obstacle_detected = false;
    uint8_t request_type = DEFAULT;
};

struct Encoder
{
    bool active = false;
    float traveled_distance_raw = 0;
    float traveled_distance = 0;
    const uint32_t wheel_circumference = WHEEL_DIAMETER * PI * 100;
    const uint32_t traveled_distance_constant = wheel_circumference;
    uint16_t last_encoder_period;
    float wheel_current_rps = 0;
    float wheel_current_spd = 0;
    float last_wheel_spd = 0;
    float last_traveled_distance = 0;
    uint32_t time4 = millis();

};

Ultrasonic_request US_REQ;
Ultrasonic_Sensor Ultrasonic;
Infrared_Sensor Infrared;
Encoder encoder;

void Sensors::begin()
{
    /* #region  Inizializzazione IMU */
    Wire.begin(21, 22);
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#ifdef ENABLE_LOGGING
    // sensorserial.println(F("Initializing I2C devices..."));
#endif
    mpu.initialize();
    // pinMode(INTERRUPT_PIN, INPUT); // pin interrupt non connesso

// verify connection
#ifdef ENABLE_LOGGING
    // sensorserial.println(F("Testing device connections..."));
    sensorscore.println(mpu.testConnection() ? (char *)"MPU6050 READY" : (char *)"MPU6050 FAIL");
#endif

// load and configure the DMP
#ifdef ENABLE_LOGGING
    // sensorserial.println(F("Initializing DMP..."));
#endif
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // sensorserial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        // sensorserial.println(F(")..."));
        
        // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING); // pin interrupt non connesso
        mpuIntStatus = mpu.getIntStatus();

#ifdef ENABLE_LOGGING
        sensorscore.println((char *)"DMP READY");
#endif
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        sensorscore.println((char *)"DMP FAIL");
    }
    /* #endregion */

    pinMode(RPM_SENS, INPUT);
    pinMode(BAT, INPUT);

    if (getBatVoltage() < 12.0)
        sensorscore.lowBat();
}

int Sensors::getFWDInfrared()
{
    // invertito: 0 = libero; 1 = ostacolo
    return !sensormux.readDigital(IR_F);
}

uint32_t TEMP_TIME = 0;

void Sensors::update()
{
    uint32_t start_time = micros();

    if (millis() - TEMP_TIME > 30000)
    {
        TEMP_TIME = millis();
        /*float raw_air_temp = mpu.getTemperature();
        float air_temp = (raw_air_temp / 340) + 36.53; // da datasheet MPU6050*/
        if (getBatVoltage() < 12.0)
            sensorscore.lowBat();
    }

    if (Infrared.enable && ENABLE_OBSTACLE_AVOIDANCE)
    {
        if (millis() - Infrared.timer > 10)
        {
            uint8_t FWD_IR = !sensormux.readDigital(IR_F);

            if (FWD_IR == 1)
            {
                // ferma il robot se il sensore infrarossi anteriore rileva un ostacolo
                sensorscore.println((char *)"(SENS) IR SENSOR STOP");
                FWD_IR = 0;
                stop_sensor_type = INFRARED;
                stop_sensor_direction = FRONT;
                if (Infrared.request_type == MOTORS)
                {
                    sensorsnav.obstacleDetectedBeforeMoving(stop_sensor_type, stop_sensor_direction);
                    sensormotors.stop();
                    stopMoving();
                }
                else if (US_REQ.request_type == DEFAULT)
                {
                    sensorsnav.obstacleDetectedWhileMoving(stop_sensor_type, stop_sensor_direction);
                    sensormotors.stop();
                    stopMoving();
                }
                Infrared.enable = false;
                //Infrared.obstacle_detected = true;
            }
            else
            {
                // se il sensore infrarossi non rileva niente, usa il sensore ultrasuoni
                getFrontUSObstacle(Infrared.request_type);
                Infrared.enable = false;
                //Infrared.obstacle_detected = false;
            }
        }
    }

    if (Ultrasonic.enable)
    {
        //sensorscore.println("US ENABLE");
        if (US_REQ.active)
        {
            //sensorscore.println("US REQ ACTIVE");
            if (US_REQ.returned)
            {
                //sensorscore.println("US REQ RETURNED");
                US_REQ.returned = false;
                US_REQ.active = false;

                if (!US_REQ.confirm_distance)
                {
                    //sensorscore.println("US REQ !CONFIRM DISTANCE");
                    if (US_REQ.distance_measured < US_SENS_DST_TRIG)
                    {
                        US_REQ.confirm_distance = true;
                        Ultrasonic.get_front_obstacle = false;
                        US_REQ.active = true;
                        sensormux.requestUSDistance(US_REQ.sensor_direction);
                    }
                    else
                        Ultrasonic.check_next = true;
                }
                else
                {
                    //sensorscore.println("US CONFIRM DISTANCE");
                    if (US_REQ.distance_measured < US_SENS_DST_TRIG)
                    {
                        //sensorscore.println("US REQ DST < SENS TRIG");
                        //US_front_obstacle_detected = true;
                        stop_sensor_direction = US_REQ.sensor_direction;
                        stop_sensor_type = ULTRASONIC;
                        US_REQ.confirm_distance = false;
                        sensorscore.println((char *)"(SENS) US SENSOR STOP", US_REQ.distance_measured);
                        if (Infrared.request_type == MOTORS)
                        {
                            sensorsnav.obstacleDetectedBeforeMoving(stop_sensor_type, stop_sensor_direction);
                            sensormotors.stop();
                            stopMoving();
                        }
                        else if (US_REQ.request_type == DEFAULT)
                        {
                            sensorsnav.obstacleDetectedWhileMoving(stop_sensor_type, stop_sensor_direction);
                            sensormotors.stop();
                            stopMoving();
                        }
                        Ultrasonic.enable = false;
                    }
                    else
                    {
                        Ultrasonic.check_next = true;
                        US_REQ.confirm_distance = false;
                        Ultrasonic.get_front_obstacle = true;
                    }
                }
            }
        }

        if (Ultrasonic.get_front_obstacle)
        {
            if (!Ultrasonic.FRONT_checked)
            {
                if (millis() - Ultrasonic.time_ref > 10)
                {
                    Ultrasonic.time_ref = millis();
                    US_REQ.active = true;
                    US_REQ.sensor_direction = FRONT;
                    US_REQ.request_type = Ultrasonic.request_type;
                    Ultrasonic.FRONT_checked = true;
                    sensormux.requestUSDistance(FRONT);
                }
            }
            else if (!Ultrasonic.RIGHT_checked)
            {
                if (Ultrasonic.check_next)
                {
                    if (millis() - Ultrasonic.time_ref > 10)
                    {
                        Ultrasonic.time_ref = millis();
                        Ultrasonic.check_next = false;
                        US_REQ.active = true;
                        US_REQ.sensor_direction = RIGHT;
                        US_REQ.request_type = Ultrasonic.request_type;
                        Ultrasonic.RIGHT_checked = true;
                        sensormux.requestUSDistance(RIGHT);
                    }
                }
            }
            else if (!Ultrasonic.LEFT_checked)
            {
                if (Ultrasonic.check_next)
                {
                    if (millis() - Ultrasonic.time_ref > 10)
                    {
                        Ultrasonic.time_ref = millis();
                        Ultrasonic.check_next = false;
                        US_REQ.active = true;
                        US_REQ.sensor_direction = LEFT;
                        US_REQ.request_type = Ultrasonic.request_type;
                        Ultrasonic.LEFT_checked = true;
                        sensormux.requestUSDistance(LEFT);
                    }
                }
            }
            else if (Ultrasonic.check_next)
            {
                Ultrasonic.check_next = false;
                Ultrasonic.LEFT_checked = false;
                Ultrasonic.RIGHT_checked = false;
                Ultrasonic.FRONT_checked = false;
                Ultrasonic.get_front_obstacle = false;
                Ultrasonic.enable = false;
                US_REQ.active = false;
            }
        }
    }
    
    // le funzioni aspettano 100ms prima di controllare se c'è movimento, così danno il tempo al robot di cambiare direzione
    if (moving && !go_once_movement_sensor)
    {
        time1_movement_sensor = millis();
        go_once_movement_sensor = true;
    }
    else if (!moving)
    {
        go_once_movement_sensor = false;
    }

    // controllo moving ogni ms
    if (millis() - time3 > 1)
    {
        if (ENABLE_MOVEMENT_SENSORS)
        {
            if (go_once_movement_sensor)
            {
                if (millis() - time1_movement_sensor > 100)
                {
                    if (moving)
                    {
                        /* #region  Controlla se il robot viene fermato dall'esterno */

                        // IL VALORE DI HEADING È STATO MOLTIPLICATO PER 100 NELLA FUNZIONE getHeading()

                        int32_t heading = getHeading();
                        int a = getAccY();

                        char direction = sensormotors.getDirection();

                        if (direction == 't')
                        {
                            moving = false;
                            return;
                        }

                        if (direction == 'w')
                        {
                            /* #region  Check se il robot viene fermato di colpo */
                            if (filtering_active)
                            {
                                forward_filter_acc_data += a;
                                times_acc_data_is_gathered++;
                                if (times_acc_data_is_gathered > 15)
                                {
                                    avg_acc = forward_filter_acc_data / times_acc_data_is_gathered;
                                    forward_filter_acc_data = 0;
                                    times_acc_data_is_gathered = 0;
                                    filtering_active = false;

                                    if (avg_acc > 350)
                                    {
                                        sensorserial.println("TYPE_SUDDEN_STOP");
                                        sensorstatus.setError(true);
                                        sensormotors.stop();
                                        stopMoving();
                                        sensorsnav.suddenStop(direction);
                                        return;
                                    }
                                }
                            }
                            else if (a > 350)
                            {
                                if (!filtering_active)
                                {
                                    if (forward_filter_acc_data == 0 && times_acc_data_is_gathered == 0)
                                    {
                                        filtering_active = true;
                                        forward_filter_acc_data += a;
                                        times_acc_data_is_gathered++;
                                    }
                                }
                            }
                            /* #endregion */
                        }
                        else if (direction == 's')
                        {
                            /* #region  Check se il robot viene fermato di colpo */
                            if (filtering_active)
                            {
                                a *= -1;
                                backwards_filter_acc_data += a;
                                times_acc_data_is_gathered++;
                                if (times_acc_data_is_gathered > 10)
                                {
                                    avg_acc = backwards_filter_acc_data / times_acc_data_is_gathered;
                                    backwards_filter_acc_data = 0;
                                    times_acc_data_is_gathered = 0;
                                    filtering_active = false;

                                    if (avg_acc > 350)
                                    {
                                        sensorstatus.setError(true);
                                        sensormotors.stop();
                                        stopMoving();
                                        sensorsnav.suddenStop(direction);
                                        return;
                                    }
                                }
                            }
                            else if (a < -350)
                            {
                                a *= -1;
                                if (!filtering_active)
                                {
                                    if (backwards_filter_acc_data == 0 && times_acc_data_is_gathered == 0)
                                    {
                                        filtering_active = true;
                                        backwards_filter_acc_data += a;
                                        times_acc_data_is_gathered++;
                                    }
                                }
                            }
                            /* #endregion */
                        }
                        else if (direction == 'd' || direction == 'a')
                        {
                            /* #region  Check se il robot viene fermato di colpo */
                            /* #region  Get Absolute Heading */
                            if (heading < 0)
                            {
                                heading *= -1;
                            }

                            if (last_heading < 0)
                            {
                                last_heading_stop_check *= -1;
                            }
                            /* #endregion */

                            if (heading != last_heading_stop_check)
                            {
                                /* #region  Get Absolute Difference */
                                int32_t difference = heading - last_heading_stop_check;

                                if (difference < 0)
                                {
                                    difference *= -1;
                                }
                                /* #endregion */

                                if (difference < 5)
                                {
                                    sensorstatus.setError(true);
                                    sensormotors.stop();
                                    stopMoving();
                                    sensorscore.println((char *)"(SENS) NOT ROTATING");
                                    return;
                                }
                            }

                            last_heading_stop_check = heading;
                            /* #endregion */
                        }
                        /* #endregion */

                        if (direction == 'w')
                        {
                            if (!Infrared.enable && !Ultrasonic.enable && !US_REQ.active)
                                checkFrontObstacle();
                        }
                    }
                }
            }

            // controlla se il robot si sta muovendo, se è stata chiamata la funzione checkMovement
            if (check_movement)
            {
                if (movement_checked < movements_to_check)
                {
                    movement_checked++;

                    int a = 0;
                    if (movement_check_axis == 'x')
                    {
                        a = getAccX();
                    }
                    else if (movement_check_axis == 'y')
                    {
                        a = getAccY();
                    }
                    else if (movement_check_axis == 'z')
                    {
                        a = getAccZ();
                    }

                    if (a < 275 && a > -275)
                    {
                        not_moving_warnings++;
                    }

                    if (not_moving_warnings > movements_to_check - 25)
                    {
                        //sensorscore.printTimestamp();
                        //sensorserial.print("TYPE_CANNOT_START_MOVE:1;");
                        sensorstatus.setError(true);
                        sensormotors.stop();
                    }
                }
                else
                {
                    check_movement = false;
                    not_moving_warnings = 0;
                    movement_checked = 0;
                    moving = true;
                }
            }

            // controlla se il robot sta ruotando, se è stata chiamata la funzione checkRotation
            if (check_rotation)
            {
                if (rotation_checked < rotations_to_check)
                {
                    rotation_checked++;

                    // TUTTI I VALORI SONO MOLTIPLICATI PER 100 DALLA FUNZIONE getHeading()

                    int32_t heading = getHeading();

                    if (rotation_checked > rotations_to_check - 25)
                    {
                        /* #region  Get Absolute Heading */
                        if (heading < 0)
                        {
                            heading *= -1;
                        }

                        if (last_heading < 0)
                        {
                            last_heading *= -1;
                        }
                        /* #endregion */

                        if (heading != last_heading)
                        {
                            /* #region  Get Absolute Difference */
                            int32_t difference = heading - last_heading;

                            if (difference < 0)
                            {
                                difference *= -1;
                            }
                            /* #endregion */

                            if (difference < 5)
                            {
                                not_rotating = true;
                                sensorscore.println((char *)"(SENS) CANNOT START ROTATE");
                            }
                        }

                        if (not_rotating)
                        {
                            sensorstatus.setError(true);
                            sensormotors.stop();
                        }
                    }

                    last_heading = heading;
                }
                else
                {
                    check_rotation = false;
                    not_rotating = false;
                    rotation_checked = 0;
                    last_heading = 0;
                    moving = true;
                }
            }
        }
        else
        {
            // se i sensori di movimento sono disabilitati, usa solo i sensori di ostacoli per fermare il robot
            char direction = sensormotors.getDirection();
            if (direction == 'w')
            {
                if (!Infrared.enable && !Ultrasonic.enable && !US_REQ.active)
                    checkFrontObstacle();
            }
        }

        time3 = millis();
    }

    char direction = sensormotors.getDirection();

    // ottieni i metri percorsi dall'encoder
    if (ENABLE_ENCODER)
    {
        if (direction == 'w' || direction == 's')
        {
            int16_t encoder_period = getEncoderPeriod();

            if (encoder_period > 5 && encoder_period < 4500)
            {
                uint16_t revolution_time = encoder_period * ENCODER_TEETH;
                float encoder_current_rps = 1000.00 / (float)revolution_time;
                encoder.wheel_current_rps = encoder_current_rps * GEAR_RATIO;
                encoder.wheel_current_spd = encoder.wheel_circumference * encoder.wheel_current_rps;

                if (encoder.last_wheel_spd != encoder.wheel_current_spd)
                {
                    uint32_t time_to_subtract = millis();

                    if (motors_rotating && !motors_rotating_check)
                    {
                        motors_rotating_check = true;
                        time_to_subtract = millis();
                        encoder.time4 = time_to_subtract;
                    }

                    uint16_t delta_t_ms = time_to_subtract - encoder.time4;

                    // S(t) = S0 + vt
                    encoder.traveled_distance_raw += encoder.wheel_current_spd * delta_t_ms;
                    encoder.traveled_distance = encoder.traveled_distance_raw / 1000000;
                    encoder.last_traveled_distance = encoder.wheel_current_spd * delta_t_ms / 1000000;

                    encoder.time4 = millis();
                }

                encoder.last_wheel_spd = encoder.wheel_current_spd;
            }
        }
    }

    t2 = micros() - start_time;

    /*if (sensor_autorun)
        sensorsnav.setLateralObstacle(getLeftInfrared());*/
}

uint32_t Sensors::getTime()
{
    return t2;
}

void Sensors::getValues()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = ypr[0] * 180 / M_PI;
        pitch = ypr[1] * 180 / M_PI;
        roll = ypr[2] * 180 / M_PI;
#ifdef ENABLE_LOGGING
        /*sensorserial.print("TYPE_YAW_PITCH_ROLL:");
        sensorserial.print(yaw);
        sensorserial.print(",");
        sensorserial.print(pitch);
        sensorserial.print(",");
        sensorserial.print(roll);
        sensorserial.print(";");*/
#endif
#endif

#ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        accX = aaReal.x;
        accY = aaReal.y;
        accZ = aaReal.z;
#ifdef ENABLE_LOGGING
        /*sensorserial.print("TYPE_ACC_XYZ:");
        sensorserial.print(aaReal.x);
        sensorserial.print(",");
        sensorserial.print(aaReal.y);
        sensorserial.print(",");
        sensorserial.print(aaReal.z);
        sensorserial.print(";");*/
#endif
#endif
    }
}

void Sensors::setZero()
{
    sensorserial.println();
    sensorserial.println("SET ZERO");
    sensorserial.println();
    // aggiorna i valori 10 volte così si stabilizzano
    for (uint8_t i = 0; i < 10; i++)
    {
        getValues();
        delay(1);
    }

    // ottiene i valori di accelerazione e inclinazione sugli assi 50 volte in modo da prendere il valore massime e minimo
    // così da avere una finestra per lo 0

    for (uint8_t i = 0; i < 50; i++)
    {
        float X = getAccX();
        float Y = getAccY();
        float Z = getAccZ();

        // aggiunge e sottrae 10 così lo 0 è più stabile
        if (X > maxX)
            maxX = X + 10;
        if (Y > maxY)
            maxY = Y + 10;
        if (Z > maxZ)
            maxZ = Z + 10;

        if (X < minX)
            minX = X - 10;
        if (Y < minY)
            minY = Y - 10;
        if (Z < minZ)
            minZ = Z - 10;

        delay(1);
    }

    if (maxX == 0)
        maxX = minX + 10;
    if (maxY == 0)
        minY = minY + 10;
    if (maxZ == 0)
        minZ = minZ + 10;

    delay(100);

    if (dmpReady)
    {
        zeroX += getAccX() * -1;
        zeroY += getAccY() * -1;
        zeroZ += getAccZ() * -1;
        zeroP += getPitch() * -1;
        zeroR += getRoll() * -1;
        zeroW += getHeading() * -1;
    }

// log di tutti i valori massimi e minimi nella porta seriale
#ifdef ENABLE_LOGGING
    sensorscore.printStartDataPacket();
    sensorscore.printTimestamp();
    sensorserial.print("TYPE_ACC_MAX_XYZ:");
    sensorserial.print(maxX);
    sensorserial.print(",");
    sensorserial.print(maxY);
    sensorserial.print(",");
    sensorserial.print(maxZ);
    sensorserial.print(";");
    sensorserial.print("TYPE_ACC_MIN_XYZ:");
    sensorserial.print(minX);
    sensorserial.print(",");
    sensorserial.print(minY);
    sensorserial.print(",");
    sensorserial.print(minZ);
    sensorserial.print(";");
    sensorserial.print("TYPE_ZERO_XYZ_YPR:");
    sensorserial.print(zeroX);
    sensorserial.print(",");
    sensorserial.print(zeroY);
    sensorserial.print(",");
    sensorserial.print(zeroZ);
    sensorserial.print(",");
    sensorserial.print(zeroW);
    sensorserial.print(",");
    sensorserial.print(zeroP);
    sensorserial.print(",");
    sensorserial.print(zeroR);
    sensorserial.print(";");
    sensorscore.printStopDataPacket();
#endif
}

int Sensors::getAccX()
{
    getValues();

    int accel = accX;
    if (accel > maxX || accel < minX)
    {
        accel += zeroX;
        if (INVERT_ACC_X)
            accel *= -1;
        return accel;
    }
    else
    {
        return 0; // ritorna 0 se l'accelerazione è dentro la finestra dello zero
    }
}

int Sensors::getAccY()
{
    getValues();

    int accel = accY;
    if (accel > maxY || accel < minY)
    {
        accel += zeroY;
        if (INVERT_ACC_Y)
            accel *= -1;
        return accel;
    }
    else
        return 0; // ritorna 0 se l'accelerazione è dentro la finestra dello zero
}

int Sensors::getAccZ()
{
    getValues();

    int accel = aaReal.z;
    if (accel > maxZ || accel < minZ)
    {
        accel += zeroZ;
        if (INVERT_ACC_Z)
            accel *= -1;
        return accel;
    }
    else
        return 0; // ritorna 0 se l'accelerazione è dentro la finestra dello zero
}

int32_t Sensors::getRoll()
{
    getValues();
    int32_t incl = (roll + zeroR) * 100;
    if (INVERT_ROLL)
        incl *= -1;
    return incl;
}

int32_t Sensors::getPitch()
{
    getValues();
    int32_t incl = (pitch + zeroP) * 100;
    if (INVERT_PITCH)
        incl *= -1;
    return incl;
}

int32_t Sensors::getHeading()
{
    getValues();
    int32_t incl = (yaw + zeroW) * 100;
    if (INVERT_YAW)
        incl *= -1;
    return incl;
}

int Sensors::getEncoderPeriod()
{
    uint16_t diff = 0;
    bool allow_return = false;

    if (digitalRead(RPM_SENS) == 0)
        allow_rising_edge = true;

    if (allow_rising_edge)
    {
        if (digitalRead(RPM_SENS) == 1)
        {
            allow_rising_edge = false;
            if (allow_second_edge)
            {
                allow_second_edge = false;
                time2 = millis();
                diff = time2 - time1;

                allow_return = true;
            }
            else
            {
                allow_second_edge = true;
                time1 = millis();
            }
        }
    }

    if (allow_return)
        return diff;
    else
        return -1;
}

void Sensors::checkMovement(char axis)
{
    check_movement = true;
    check_rotation = false;
    movement_check_axis = axis;
    Ultrasonic.check_next = false;
    Ultrasonic.LEFT_checked = false;
    Ultrasonic.RIGHT_checked = false;
    Ultrasonic.FRONT_checked = false;
}

void Sensors::checkRotation()
{
    check_rotation = true;
    check_movement = false;
}

void Sensors::stopMoving()
{
    moving = false;
    Ultrasonic.enable = false;
    US_REQ.active = false;
    not_rotating = true;
    check_rotation = false;
    check_movement = false;
}

void Sensors::setMotorsRotating()
{
    motors_rotating = true;
}

void Sensors::setMotorsStop()
{
    motors_rotating = false;
    motors_rotating_check = false;
}

float Sensors::getTraveledDistance()
{
    return encoder.traveled_distance;
}

void Sensors::resetTraveledDistance()
{
    encoder.traveled_distance_raw = 0;
    encoder.traveled_distance = 0;
    encoder.last_traveled_distance = 0;
}

int Sensors::getLeftInfrared()
{
    // invertito: 0 = libero; 1 = ostacolo
    return !sensormux.readDigital(IR_L);
}

void Sensors::setAutoRun(bool state)
{
    sensor_autorun = state;
}

void Sensors::returnUSDistance(float dst)
{
    US_REQ.returned = true;
    US_REQ.distance_measured = dst;
}

void Sensors::getFrontUSObstacle(uint8_t req_type)
{
    if (!Ultrasonic.enable && !US_REQ.active)
    {
        Ultrasonic.get_front_obstacle = true;
        Ultrasonic.request_type = req_type;
        Ultrasonic.time_ref = millis();
        Ultrasonic.enable = true;
    }
}

void Sensors::checkFrontObstacle(uint8_t request_type)
{
    Infrared.request_type = request_type;
    Ultrasonic.request_type = request_type;
    US_REQ.request_type = request_type;
    Infrared.enable = true; // Ultrasonic.enable viene controllato da Infrared
    Infrared.timer = millis();
}

float Sensors::getLastTraveledDistance()
{
    return encoder.last_traveled_distance;
}

float Sensors::getBatVoltage()
{
    float tot_voltage = 0.00;
    uint64_t start_time = micros();
    // tempo per ottenere le misurazioni: circa 6ms
    for (int i = 0; i < 50; i++)
        tot_voltage += analogRead(BAT);
    uint64_t diff = micros() - start_time;
    return tot_voltage / 11475;
}
