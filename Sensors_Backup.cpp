#include <Sensors.h>
#include <SPI.h>
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

#include "MPU6050.h"

// classi librerie
Motors sensormotors;
Status sensorstatus;
MPU6050 mpu;
Core sensorcore;
NAV sensornav;
BluetoothSerial sensorserial;
Mux sensormux;
uint64_t t2 = 0;
TaskHandle_t MPU6050Status;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL

// variabili
bool mpu_ready = false;
int32_t forward_filter_acc_data = 0;
int32_t backwards_filter_acc_data = 0;
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
// 14.5 mm encder

int movement_checked = 0;
bool check_movement = false;
int not_moving_warnings = 0;
char movement_check_axis;
const int movements_to_check = 100;
int total_acceleration = 0;
int stop_acc_1 = 0;
int stop_acc_2 = 0;
bool sensor_autorun = false;
uint16_t inactivity_count = 0;

bool check_rotation = false;
int32_t last_heading = 0;
int32_t last_heading_stop_check = 0;
int rotation_checked = 0;
bool not_rotating = false;
const int rotations_to_check = 50;

uint64_t *sensor_packetptr;

bool US_front_obstacle_detected = false;

bool moving = false;
float temp_acc = 0;

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

int32_t maxX = 0;
int32_t maxY = 0;
int32_t maxZ = 0;

int32_t minX = 0;
int32_t minY = 0;
int32_t minZ = 0;

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
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void IRAM_ATTR dmpDataReady()
{
    mpuInterrupt = true;
}

bool selftest[3] = {false};

struct Encoder
{
    bool active = false;
    float traveled_distance_raw = 0;
    float traveled_distance = 0;
    const float wheel_circumference = WHEEL_DIAMETER * PI * 100;
    const float traveled_distance_constant = wheel_circumference;
    uint16_t last_encoder_period;
    float wheel_current_rps = 0;
    float wheel_current_spd = 0;
    float last_wheel_spd = 0;
    float last_traveled_distance = 0;
    uint32_t last_read_time = 0;
    uint32_t time4 = millis();
};

struct SetZero
{
    bool active = false;
    uint8_t arr_idx = 0;
    int32_t arrX[50] = {};
    int32_t arrY[50] = {};
    int32_t arrZ[50] = {};
    int32_t old_accX = 0;
    int32_t maxX = 0;
    int32_t maxY = 0;
    int32_t maxZ = 0;
    int32_t minX = 0;
    int32_t minY = 0;
    int32_t minZ = 0;
};

struct SensorPacket
{
    struct
    {
        uint64_t us_f = 0;
        uint64_t us_l = 0;
        uint64_t us_r = 0;
        uint64_t ir_f = 0;
        uint64_t ir_l = 0;
        bool check_next_dst = false;
    } obstacle;

    struct
    {
        uint64_t value = 0;
        uint64_t n_values = 0;
        uint64_t total = 0;
        bool waiting = false;

    } bat;

    uint64_t read_digital = 0;
    uint64_t read_analog = 0;

    struct
    {
        uint64_t id = 0;
        uint64_t last_id = 0;
        bool is_polling = false;
    } info;
};

Encoder encoder;
SetZero setzero;
SensorPacket senspacket;

void MPU6050StatusFunction(void *param)
{
    BluetoothSerial taskserial;
    taskserial.print("Waiting for MPU6050");
    Serial.print("Waiting for MPU6050");
    uint8_t time_passed = 0;
    bool once_mpu = false;
    for (;;)
    {
        if (!mpu_ready)
        {
            if (time_passed > 20) // 10 secondi
            {
                taskserial.println("MPU is unresponsive");
                Serial.println("MPU is unresponsive");
                vTaskDelete(NULL);
            }
            delay(500);
            taskserial.print(".");
            Serial.print(".");
            time_passed++;
        }
        else
        {
            if (!once_mpu)
            {
                taskserial.println();
                taskserial.println("MPU6050 READY");
                taskserial.print("Waiting for DMP");
                Serial.println();
                Serial.println("MPU6050 READY");
                Serial.print("Waiting for DMP");
                once_mpu = true;
                time_passed = 0;
            }
            else
            {
                if (!dmpReady)
                {
                    if (time_passed > 20) // 10 secondi
                    {
                        taskserial.println("DMP is unresponsive");
                        Serial.println("DMP is unresponsive");
                        vTaskDelete(NULL);
                    }
                    delay(500);
                    taskserial.print(".");
                    Serial.print(".");
                    time_passed++;
                }
                else
                {
                    taskserial.println();
                    taskserial.println("DMP READY");
                    Serial.println();
                    Serial.println("DMP READY");
                    vTaskDelete(NULL);
                }
            }
        }
    }
}

void Sensors::begin()
{
    sensorcore.println("SELF TEST SENSORI");
    xTaskCreatePinnedToCore(MPU6050StatusFunction, "MPU6050Status", 2000, NULL, 24, &MPU6050Status, 0);
    /* #region  Inizializzazione MPU6050 */
    Wire.begin(21, 22);
    Wire.setClock(400000);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    if (mpu.testConnection())
    {
        selftest[0] = true;
        mpu_ready = true;
    }
    sensorcore.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(82);
    mpu.setYGyroOffset(-52);
    mpu.setZGyroOffset(-29);
    mpu.setXAccelOffset(-2570);
    mpu.setYAccelOffset(-221);
    mpu.setZAccelOffset(1408);

    if (devStatus == 0)
    {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        sensorcore.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        selftest[1] = true;
        sensorcore.println("DMP OK");
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        sensorcore.print(F("DMP Initialization failed - code"), devStatus);
        sensorcore.println("MPU FAIL");
    }
    /* #endregion */

    pinMode(RPM_SENS, INPUT_PULLDOWN);
    pinMode(BAT, INPUT);

    sensor_packetptr = sensormux.getPacketPointer();

    //if (getBatADC() > 0)
    //{
    //    selftest[2] = true;
    //    sensorcore.println("BAT OK");
        //if (getBatADC() < 2740)
        //    sensorcore.lowBat();
    //}
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

    if (millis() - TEMP_TIME > 30)
    {
        inactivity_count++;

        TEMP_TIME = millis();
        if (!senspacket.bat.waiting && !senspacket.info.is_polling)
        {
            senspacket.bat.waiting = true;
            senspacket.info.last_id = senspacket.info.id;
            sensormux.readAnalog(BAT);
        }
        else if (senspacket.bat.waiting && !senspacket.info.is_polling)
        {
            senspacket.bat.waiting = false;
            senspacket.read_analog = *(sensor_packetptr + 7);
            senspacket.info.id = *(sensor_packetptr + 8);
            if (senspacket.info.id > senspacket.info.last_id)
            {
                senspacket.info.last_id = senspacket.info.id;
                senspacket.bat.n_values++;
                if (senspacket.bat.n_values == 20)
                {
                    if (senspacket.bat.total / 20 < 2600 && senspacket.bat.total / 20 > 700)
                        sensorcore.lowBat();
                    senspacket.bat.total = 0;
                    senspacket.bat.n_values = 0;
                }
                else if (senspacket.read_analog < 4000 && senspacket.read_analog > 700)
                    senspacket.bat.total += senspacket.read_analog;
                else
                    senspacket.bat.total += 3300;
            }
        }
        else if (senspacket.bat.waiting && senspacket.info.is_polling)
            senspacket.bat.waiting = false;

        if (inactivity_count > 20000) // 10 minuti
        {
            
        }
    }

    // US_F, US_L, US_R, IR_F, IR_L, BAT, READ_DIGITAL, READ_ANALOG, PACKET_ID
    if (senspacket.info.is_polling)
    {
        senspacket.obstacle.us_f = *(sensor_packetptr);
        senspacket.obstacle.us_l = *(sensor_packetptr + 1);
        senspacket.obstacle.us_r = *(sensor_packetptr + 2);
        senspacket.obstacle.ir_f = *(sensor_packetptr + 3);
        senspacket.obstacle.ir_f = *(sensor_packetptr + 4);
        senspacket.bat.value = *(sensor_packetptr + 5);
        senspacket.read_digital = *(sensor_packetptr + 6);
        senspacket.read_analog = *(sensor_packetptr + 7);
        senspacket.info.id = *(sensor_packetptr + 8);

        if (senspacket.info.id > senspacket.info.last_id)
        {
            if (senspacket.obstacle.us_f < US_SENS_DST_TRIG)
            {
                if (!senspacket.obstacle.check_next_dst)
                    senspacket.obstacle.check_next_dst = true;
                else
                {
                    senspacket.obstacle.check_next_dst = false;
                    sensorcore.println(F("(Sensors.cpp) US FRONT STOP"));
                    sensornav.obstacleDetectedWhileMoving(ULTRASONIC, FRONT);
                }
            }
            else if (senspacket.obstacle.us_l < US_SENS_DST_TRIG)
            {
                if (!senspacket.obstacle.check_next_dst)
                    senspacket.obstacle.check_next_dst = true;
                else
                {
                    senspacket.obstacle.check_next_dst = false;
                    sensorcore.println(F("(Sensors.cpp) US LEFT STOP"));
                    sensornav.obstacleDetectedWhileMoving(ULTRASONIC, LEFT);
                }
            }
            else if (senspacket.obstacle.us_r < US_SENS_DST_TRIG)
            {
                if (!senspacket.obstacle.check_next_dst)
                    senspacket.obstacle.check_next_dst = true;
                else
                {
                    senspacket.obstacle.check_next_dst = false;
                    sensorcore.println(F("(Sensors.cpp) US RIGHT STOP"));
                    sensornav.obstacleDetectedWhileMoving(ULTRASONIC, RIGHT);
                }
            }

            senspacket.bat.n_values++;
            if (senspacket.bat.n_values == 20)
            {
                if (senspacket.bat.total / 20 < 2600 && senspacket.bat.total / 20 > 700)
                    sensorcore.lowBat();
                senspacket.bat.total = 0;
                senspacket.bat.n_values = 0;
            }
            else if (senspacket.bat.value < 4000 && senspacket.bat.value > 700)
                senspacket.bat.total += senspacket.bat.value;
            else
                senspacket.bat.total += 3300;
        }

        senspacket.info.last_id = senspacket.info.id;
    }

    // le funzioni aspettano 100ms prima di controllare se c'è movimento, così danno il tempo al robot di cambiare direzione
    if (moving && !go_once_movement_sensor)
    {
        time1_movement_sensor = millis();
        go_once_movement_sensor = true;
    }
    else if (!moving)
        go_once_movement_sensor = false;

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
                        // IL VALORE DI HEADING È STATO MOLTIPLICATO PER 100 NELLA FUNZIONE getHeading()

                        int32_t heading = getHeading();
                        int a = getAccY();

                        char direction = sensormotors.getDirection();

                        if (direction == 't')
                        {
                            moving = false;
                            senspacket.info.is_polling = false;
                            return;
                        }

                        if (direction == 'w')
                        {
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

                                    if (avg_acc > SUDDEN_STOP_ACCELERATION)
                                    {
                                        sensorcore.println(F("(Sensors.cpp) SUDDEN STOP"));
                                        sensorstatus.setError(true);
                                        sensornav.externalStop();
                                        sensornav.suddenStop(direction);
                                        return;
                                    }
                                }
                            }
                            else if (a > SUDDEN_STOP_ACCELERATION)
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
                        }
                        else if (direction == 's')
                        {
                            if (filtering_active)
                            {
                                a *= -1;
                                backwards_filter_acc_data += a;
                                times_acc_data_is_gathered++;
                                if (times_acc_data_is_gathered > 15)
                                {
                                    avg_acc = backwards_filter_acc_data / times_acc_data_is_gathered;
                                    backwards_filter_acc_data = 0;
                                    times_acc_data_is_gathered = 0;
                                    filtering_active = false;

                                    if (avg_acc > SUDDEN_STOP_ACCELERATION)
                                    {
                                        sensorcore.println(F("(Sensors.cpp) SUDDEN STOP"));
                                        sensorstatus.setError(true);
                                        sensornav.externalStop();
                                        sensornav.suddenStop(direction);
                                        return;
                                    }
                                }
                            }
                            else if (a < -SUDDEN_STOP_ACCELERATION)
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
                        }
                        else if (direction == 'd' || direction == 'a')
                        {
                            if (heading < 0)
                                heading *= -1;
                            if (last_heading < 0)
                                last_heading_stop_check *= -1;

                            if (heading != last_heading_stop_check)
                            {
                                int32_t difference = heading - last_heading_stop_check;

                                if (difference < 0)
                                    difference *= -1;

                                if (difference < 5)
                                {
                                    sensorstatus.setError(true);
                                    sensornav.externalStop();
                                    sensorcore.println(F("(Sensors.cpp) NOT ROTATING"));
                                    return;
                                }
                            }

                            last_heading_stop_check = heading;
                        }

                        if (direction == 'w')
                        {
                            
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
                                if (heading < 0)
                                    heading *= -1;
                                if (last_heading < 0)
                                    last_heading *= -1;

                                int32_t difference = heading - last_heading;

                                if (difference < 0)
                                    difference *= -1;

                                if (difference < 5)
                                {
                                    not_rotating = true;
                                    rotation_checked = 0;
                                }

                                if (not_rotating)
                                {
                                    sensorstatus.setError(true);
                                    sensornav.externalStop();
                                    rotation_checked = 0;
                                    sensorcore.println(F("(Sensors.cpp) CANNOT START ROTATE"));
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
                            senspacket.info.is_polling = true;
                            sensormux.sensPacketUpdate(true);
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
                        a = getAccX();
                    else if (movement_check_axis == 'y')
                        a = getAccY();
                    else if (movement_check_axis == 'z')
                        a = getAccZ();

                    if (a < 275 && a > -275)
                    {
                        not_moving_warnings++;
                    }

                    if (not_moving_warnings > movements_to_check - 25)
                    {
                        sensorcore.println(F("(Sensors.cpp) CANNOT START MOVE"));
                        sensorstatus.setError(true);
                        sensornav.externalStop();
                    }
                }
                else
                {
                    check_movement = false;
                    not_moving_warnings = 0;
                    movement_checked = 0;
                    moving = true;
                    senspacket.info.is_polling = true;
                    sensormux.sensPacketUpdate(true);
                }
            }
        }
        else
        {
            // se i sensori di movimento sono disabilitati, usa solo i sensori di ostacoli per fermare il robot
            char direction = sensormotors.getDirection();
            if (direction == 'w' && ENABLE_OBSTACLE_AVOIDANCE)
            {
                if (!senspacket.info.is_polling)
                {
                    senspacket.info.is_polling = true;
                    sensormux.sensPacketUpdate(true);
                }
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

            if (millis() - encoder.last_read_time > 250)
                sensornav.motorStall();

            if (encoder_period > 5 && encoder_period < 4500)
            {
                encoder.last_read_time = millis();
                uint16_t revolution_time = encoder_period * ENCODER_TEETH;
                float encoder_current_rps = 1000.00 / (float)revolution_time;
                //encoder.wheel_current_rps = encoder_current_rps * GEAR_RATIO; (GEAR RATIO 1)
                encoder.wheel_current_spd = encoder.wheel_circumference * encoder_current_rps; // mm/s

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

                    // S(t) = S + S0 + vt
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

    if (setzero.active)
    {
        if (setzero.arr_idx != 49)
        {
            getValues();
            if (accX != setzero.old_accX)
            {
                setzero.old_accX = accX;
                setzero.arrX[setzero.arr_idx] = accX;
                setzero.arrY[setzero.arr_idx] = accY;
                setzero.arrZ[setzero.arr_idx] = accZ;
                setzero.arr_idx++;
            }
        }
        else
        {
            setzero.active = false;
            setzero.arr_idx = 0;
            // prende il valore massimo e minimo per ogni asse
            for (int i = 0; i < 50; i++)
            {
                if (setzero.arrX[i] > setzero.maxX)
                    setzero.maxX = setzero.arrX[i];
                if (setzero.arrY[i] > setzero.maxY)
                    setzero.maxY = setzero.arrY[i];
                if (setzero.arrZ[i] > setzero.maxZ)
                    setzero.maxZ = setzero.arrZ[i];

                if (setzero.arrX[i] < setzero.minX)
                    setzero.minX = setzero.arrX[i];
                if (setzero.arrY[i] < setzero.minY)
                    setzero.minY = setzero.arrY[i];
                if (setzero.arrZ[i] < setzero.minZ)
                    setzero.minZ = setzero.arrZ[i];
            }

            // aggiunge e sottrae 10 a massimo e minimo per dare più margine
            setzero.maxX += 10;
            setzero.maxY += 10;
            setzero.maxZ += 10;
            setzero.minX -= 10;
            setzero.minY -= 10;
            setzero.minZ -= 10;

            if (setzero.maxX == 0)
                setzero.maxX = setzero.minX + 10;
            if (setzero.maxY == 0)
                setzero.maxY = setzero.minY + 10;
            if (setzero.maxZ == 0)
                setzero.maxZ = setzero.minZ + 10;

            zeroX += accX * -1;
            zeroY += accY * -1;
            zeroZ += accZ * -1;
            zeroW += yaw * -1;
            zeroP += pitch * -1;
            zeroR += roll * -1;
            maxX = setzero.maxX;
            maxY = setzero.maxY;
            maxZ = setzero.maxZ;
            minX = setzero.minX;
            minY = setzero.minY;
            minZ = setzero.minZ;

            setzero = {}; // reset di tutti i valori temporeanei per la calibrazione

            /* #region  Log valori di calibrazione */
            sensorcore.println(F("MAX X"), maxX);
            sensorcore.println(F("MAX Y"), maxY);
            sensorcore.println(F("MAX Z"), maxZ);
            sensorcore.println(F("MIN X"), minX);
            sensorcore.println(F("MIN Y"), minY);
            sensorcore.println(F("MIN Z"), minZ);
            sensorcore.println(F("ZERO X"), zeroX);
            sensorcore.println(F("ZERO Y"), zeroY);
            sensorcore.println(F("ZERO Z"), zeroZ);
            sensorcore.println(F("ZERO W"), zeroW);
            sensorcore.println(F("ZERO P"), zeroP);
            sensorcore.println(F("ZERO R"), zeroR);
            /* #endregion */
        }
    }
}

uint32_t Sensors::getTime()
{
    return t2;
}

void Sensors::setZero()
{
    setzero.active = true;
    sensorserial.println();
    sensorserial.println("CALIBRATING");
    sensorserial.println();
    Serial.println();
    Serial.println("CALIBRATING");
    Serial.println();
}

void Sensors::getValues()
{
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = ypr[0] * 180 / PI;
        pitch = ypr[1] * 180 / PI;
        roll = ypr[2] * 180 / PI;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        accX = aaReal.x;
        accY = aaReal.y;
        accZ = aaReal.z;
    }
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
        return 0; // ritorna 0 se l'accelerazione è dentro la finestra dello zero
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

int IRAM_ATTR Sensors::getEncoderPeriod()
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
}

void Sensors::checkRotation()
{
    check_rotation = true;
    check_movement = false;
}

void Sensors::resetMovementVars()
{
    moving = false;
    not_rotating = false;
    check_rotation = false;
    check_movement = false;
    rotation_checked = 0;
    not_moving_warnings = 0;
    movement_checked = 0;
    times_acc_data_is_gathered = 0;
    senspacket.info.is_polling = false;
    sensormux.sensPacketUpdate(false);
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

float Sensors::getLastTraveledDistance()
{
    return encoder.last_traveled_distance;
}

uint16_t Sensors::getBatADC()
{
    float tot_voltage = 0.00;
    // tempo per ottenere le misurazioni: circa 6ms
    for (int i = 0; i < 50; i++)
        tot_voltage += analogRead(BAT);
    return tot_voltage / 50;
}
