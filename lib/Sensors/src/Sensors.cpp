#include <SPI.h>
#include "Wire.h"
#include <FunctionalInterrupt.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
TaskHandle_t mpu_update_handle;
MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

#include <AS5600.h>
#include <Sensors.h>
#include <Status.h>
#include <Motors.h>
#include <Core.h>
#include <Navigation.h>
#include <Mux.h>

// l'mpu e l'encoder sinistro usano entrambi il bus 0 (sda 11, scl 12) anche se sulla pcb dovrebbero usare bus diversi (sistemato in rev3)
// questo perché l'esp32s3 ha solo 2 bus I2C quindi non potrebbe gestire tutte e 3 le periferiche
Motors sensormotors;
Core sensorcore;
NAV sensornav;
Mux sensormux;
uint64_t t2 = 0;

uint32_t sens_refresh_time = millis();
uint32_t inactivity_count = 0;
uint64_t *sensor_packetptr;

float yaw = 0;
float yaw_compensation = YAW_COMP_START_VALUE;
float pitch = 0;
float roll = 0;

class EncoderData
{
    public:
        int64_t revolutions = 0;
        uint32_t read_time = 0;
        uint32_t last_read_time = 0;
        uint64_t absolute_revolutions = 0;
        int64_t total_angle = 0;
        int64_t scaled_angle = 0;
        int64_t last_scaled_angle = 0;
        int32_t diff = 0;
        // w = vt / r
        // r = 147 mm
        float angular_velocity = 0;
};

class Encoder
{
    private:
        float rpm = 0;
        float last_rpm = 0;
        uint32_t ENCA = 0;
        uint32_t ENCB = 0;
        uint32_t pos = 0;
        bool first = false;
        
        uint32_t pulse_start_time = 0;
        uint32_t pulse_deltaT = 0;

        // @returns periodo di un "dente" in microsecondi
        uint32_t IRAM_ATTR measureTimePeriod()
        {
            first = false;
            pos = 0;
            while (true)
            {
                if (!first && pos == 0)
                {
                    if (digitalRead(ENCA) == 0)
                        first = true;
                }
                else if (pos == 0)
                {
                    if (digitalRead(ENCA) == 1)
                    {
                        pulse_start_time = micros();
                        pos++;
                        first = false;
                    }
                }
                else if (pos == 1)
                {
                    if (!first)
                    {
                        if (digitalRead(ENCA) == 0)
                            first = true;
                    }
                    else
                    {
                        if (digitalRead(ENCA) == 1)
                            return micros() - pulse_start_time;
                    }
                }
            }
        }

    public:
        Encoder(int32_t sig1, int32_t sig2)
        {
            ENCA = sig1; ENCB = sig2;
            pinMode(ENCA, INPUT);
            pinMode(ENCB, INPUT);   // in questa configurazione ENCB non viene usato. di solito si usa per determinare la direzione di rotazione
        };

        float getRPM()
        {
            // measureTimePeriod() misura il periodo di 1 "dente" magnetico. si moltiplica per 11 perché ce ne sono appunto 11
            // nell'encoder. quindi 11 per ogni rotazione
            rpm = (60.0 / ((float)measureTimePeriod() * 11.0 / 1e6)) / MOV_MOTOR_GEAR_RATIO;
            if (rpm < RPM_SENSOR_MAX_VALUE) // filtra eventuali valori estremamente alti dati da errori di misurazione
            {
                last_rpm = rpm;
                return rpm;
            }
            else
                return last_rpm;
        }
        
        // @returns velocità assoluta in mm/s
        float getSpeed() { return 499.26 * (getRPM() / 60.0); } // 499.26 = 2 * PI * WHEEL_RADIUS
};

struct SetZero
{
    bool active = false;
    uint8_t arr_idx = 0;
    int32_t arrYaw[500] {};
    int32_t arrPitch[500] = {};
    int32_t arrRoll[500] = {};
    int32_t old_yaw = 0;
    int32_t zero_yaw = 0;
    int32_t zero_pitch = 0;
    int32_t zero_roll = 0;
    int32_t max_yaw = 0;
    int32_t max_pitch = 0;
    int32_t max_roll = 0;
    int32_t min_yaw = 0;
    int32_t min_pitch = 0;
    int32_t min_roll = 0;
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
        uint64_t check_dst_packet = 0;
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

Encoder encleft(10, 9);
Encoder encright(5, 4);
EncoderData rightEncoder;
EncoderData leftEncoder;
SetZero setzero;
SensorPacket senspacket;
Sensors::Robot robot;

void mpuUpdate(void* params)
{
    uint32_t yaw_timer = 0;
    uint32_t received_notification = 0;
    bool mpu_run = true;
    while (1)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &received_notification, 0) == pdTRUE)
        {
            if (received_notification == STOP)
                mpu_run = false;
            else if (received_notification == RUN)
                mpu_run = true;
        }

        if (mpu_run)
        {
            fifoCount = mpu.getFIFOCount();

            if (fifoCount == 1024)
            {
                mpu.resetFIFO();
                Serial.println(F("FIFO overflow!"));
            }
            else
            {
                if (fifoCount % packetSize != 0)
                    mpu.resetFIFO();
                else
                {
                    while (fifoCount >= packetSize)
                    {
                        mpu.getFIFOBytes(fifoBuffer,packetSize);
                        fifoCount -= packetSize;
                    }
                }

                mpu.dmpGetQuaternion(&q,fifoBuffer);
                mpu.dmpGetGravity(&gravity,&q);
                mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);          
                
                yaw = ypr[0]*180/PI + yaw_compensation;
                pitch = ypr[1]*180/PI;
                roll = ypr[2]*180/PI;
            }

            if (millis() - yaw_timer > YAW_COMP_LOOP)
            {
                yaw_compensation += YAW_COMP_DRIFT;
                yaw_timer = millis();
            }
        }

        vTaskDelay(6 / portTICK_PERIOD_MS);
    }
}

void Sensors::begin()
{
    Wire.begin(11, 12);
    Wire.setClock(800000L);

    sensorcore.print(F("(Sensors) Initializing MPU6050,"));
    mpu.initialize();
    sensorcore.println(F(" OK"));
    sensorcore.print(F("(Sensors) Initializing DMP"));
    if (mpu.dmpInitialize() == 1)
    {
        sensorcore.println(F(" CANNOT INITIALIZE DMP!"));
        while (true)
            delay(10);
    }
    sensorcore.println(F(" OK"));
    sensorcore.print(F("(Sensors) Calibrating IMU, "));
    /*mpu.setXAccelOffset(-3314);
    mpu.setYAccelOffset(-1360);
    mpu.setZAccelOffset(767);
    mpu.setXGyroOffset(115);
    mpu.setYGyroOffset(1);
    mpu.setZGyroOffset(8);*/
    mpu.CalibrateAccel();
    mpu.CalibrateGyro();
    sensorcore.println(F(" DONE"));
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();
    sensor_packetptr = sensormux.getPacketPointer();

    pinMode(REF_BAT, INPUT);

    xTaskCreatePinnedToCore(mpuUpdate, "mpuUpdate", 4096, NULL, 10, &mpu_update_handle, 0);
}

uint32_t temp_time = 0;
uint32_t pause_time = 0;

void pausedEvent()
{
    sensornav.resume();
}

void Sensors::update()
{
    uint32_t start_time = esp_timer_get_time();
    uint32_t direction = sensormotors.getDirection();

    if (direction != STOP)
        inactivity_count = 0;

    if (millis() - temp_time > robot.bat_check_time)
    {
        if (direction == STOP)
            inactivity_count++;

        if (analogRead(REF_BAT) < 2600 && analogRead(REF_BAT) > 1500 && ENABLE_BAT_VOLTAGE_SENSING)
            sensorcore.lowBat();
            
        if (inactivity_count > INACTIVE_TIME_THRESHOLD)
        {
            robot.is_inactive = true;
            sensormotors.playInactiveSound();
            robot.bat_check_time = DEFAULT_BAT_CHECK_TIME + (TIME_BETW_INACTIVE_BEEPS - DEFAULT_BAT_CHECK_TIME);
        }
        else if (robot.is_inactive)
        {
            robot.is_inactive = false;
            robot.bat_check_time = DEFAULT_BAT_CHECK_TIME;
        }

        temp_time = millis();
    }

    // US_F, US_L, US_R, IR_F, IR_L, BAT, READ_DIGITAL, READ_ANALOG, PACKET_ID
    if (senspacket.info.is_polling && ENABLE_OBSTACLE_AVOIDANCE)
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
            if (senspacket.obstacle.us_f < US_SENS_DST_TRIG && senspacket.obstacle.us_f != 0)
            {
                if (!senspacket.obstacle.check_next_dst)
                {
                    senspacket.obstacle.check_dst_packet = senspacket.info.id;
                    senspacket.obstacle.check_next_dst = true;
                }
                else if (senspacket.info.id == senspacket.obstacle.check_dst_packet + 1)
                {
                    senspacket.obstacle.check_next_dst = false;
                    sensorcore.println(F("(Sensors.cpp) US FRONT STOP"));
                    sensornav.obstacleDetectedWhileMoving(ULTRASONIC, FRONT);
                }
                else
                    senspacket.obstacle.check_next_dst = false;
            }
            else if (senspacket.obstacle.us_l < US_SENS_DST_TRIG && senspacket.obstacle.us_l != 0)
            {
                if (!senspacket.obstacle.check_next_dst)
                {
                    senspacket.obstacle.check_dst_packet = senspacket.info.id;
                    senspacket.obstacle.check_next_dst = true;
                }
                else if (senspacket.info.id == senspacket.obstacle.check_dst_packet + 1)
                {
                    senspacket.obstacle.check_next_dst = false;
                    sensorcore.println(F("(Sensors.cpp) US LEFT STOP"));
                    sensornav.obstacleDetectedWhileMoving(ULTRASONIC, LEFT);
                }
                else
                    senspacket.obstacle.check_next_dst = false;
            }
            else if (senspacket.obstacle.us_r < US_SENS_DST_TRIG && senspacket.obstacle.us_r != 0)
            {
                if (!senspacket.obstacle.check_next_dst)
                {
                    senspacket.obstacle.check_dst_packet = senspacket.info.id;
                    senspacket.obstacle.check_next_dst = true;
                }
                else if (senspacket.info.id == senspacket.obstacle.check_dst_packet + 1)
                {
                    senspacket.obstacle.check_next_dst = false;
                    sensorcore.println(F("(Sensors.cpp) US RIGHT STOP"));
                    sensornav.obstacleDetectedWhileMoving(ULTRASONIC, RIGHT);
                }
                else
                    senspacket.obstacle.check_next_dst = false;
            }
        }

        senspacket.info.last_id = senspacket.info.id;
    }

    if (millis() - sens_refresh_time > MPU_MOT_ENCODERS_REFRESH_RATE)
    {

        if (direction == FWD && ENABLE_OBSTACLE_AVOIDANCE)
        {
            if (!senspacket.info.is_polling)
            {
                sensorcore.println("STARTING POLLING");
                senspacket.info.is_polling = true;
                sensormux.sensPacketUpdate(true);
            }
        }

        if (ENABLE_MOT_ENCODERS)
        {
            if (robot.enable_speed_encoders)
            {
                // cm/s
                float leftspd = encleft.getSpeed() / 10.0;
                // cm/s
                float rightspd = encright.getSpeed() / 10.0;

                if (robot.iterations_ignored == 2 && leftspd < RPM_SENSOR_MAX_VALUE && rightspd < RPM_SENSOR_MAX_VALUE)
                {
                    if (direction == BCK)
                    {
                        leftspd *= -1;
                        rightspd *= -1;
                    }
                    else if (direction == RIGHT)
                        rightspd *= -1;
                    else if (direction == LEFT)
                        leftspd *= -1;

                    float robot_speed = (leftspd + rightspd) / 2.0;
                    float deltaT = (float)(micros() - robot.last_angle_time) / 1e6;
                    robot.traveled_distance_raw = robot_speed * deltaT;
                    robot.last_angle_time = micros();
                    robot.traveled_distance += robot.traveled_distance_raw;
                    robot.last_traveled_distance = robot.traveled_distance_raw;

                    // CALCOLO HEADING CON ENCODER
                    /*float left_radius = (ROBOT_WIDTH / 10.0) * abs(leftspd / (abs(leftspd) + abs(rightspd)));
                    float robot_w = 0;

                    if (leftspd < 0 && rightspd > 0 || rightspd < 0 && leftspd > 0)
                        robot_w = leftspd / left_radius;
                    else
                        robot_w = (leftspd - rightspd) / (ROBOT_ROTATION_RADIUS / 10.0);

                    float inst = degrees((robot_w) * deltaT);
                    robot.angle += inst;
                    while (robot.angle >  180.0) robot.angle -= 360.0;
                    while (robot.angle < -180.0) robot.angle += 360.0;*/


                    //Serial.printf("dst: %f, ang: %f, inst: %f, left: %f, right: %f -- LSPD: %f, RSPD: %f deltatime: %f\n", robot.traveled_distance, robot.angle, inst, left_w, right_w, leftspd, rightspd, deltaT);
                    //Serial.printf("ang: %f, deltaT: %f, deltaR: %f, robot.traveled_distance: %f, dst: %f -- LRPM: %f, RRPM: %f\n", robot.angle, deltaT, deltaR, robot.traveled_distance, dst / 10.0, leftrpm, rightrpm);
                }
                else
                {
                    robot.last_angle_time = micros();
                    robot.iterations_ignored++; // diventa 0 quando viene chiamato setMotorsStop()
                }
            }
        }

        sens_refresh_time = millis();
    }

    t2 = esp_timer_get_time() - start_time;
}

uint32_t Sensors::getTime()
{
    return t2;
}

int32_t Sensors::getRoll()
{
    if (INVERT_ROLL)
        return roll * -100;
    return roll * 100;
}

int32_t Sensors::getPitch()
{
    if (INVERT_PITCH)
        return pitch * -100;
    return pitch * 100;
}

int32_t Sensors::getHeading()
{
    if (INVERT_YAW)
        return yaw * -100;
    return yaw * 100;
}

void Sensors::resetMovementVars()
{
    robot.enable_speed_encoders = false;
    senspacket.info.is_polling = false;
    sensormux.sensPacketUpdate(false);
}

void Sensors::setMotorsStop()
{
    robot.enable_speed_encoders = false;
    robot.iterations_ignored = 0;
}

float Sensors::getTraveledDistance()
{
    return robot.traveled_distance;
}

float Sensors::getLastTraveledDistance()
{
    return robot.last_traveled_distance;
}

void Sensors::resetTraveledDistance()
{
    robot.traveled_distance = 0;
    robot.last_traveled_distance = 0;
}

int64_t Sensors::invert180HDG(int64_t hdg)
{
    int64_t return_hdg = hdg + 18000;
    if (return_hdg - 36000 > 0)
        return return_hdg - 36000;
    else
        return return_hdg;
}

int64_t Sensors::convert360To180HDG(int64_t hdg, bool always_positive)
{
    if (hdg > 18000)
        if (always_positive && hdg - 36000 < 0)
            return (hdg - 36000) * -1;
        else
            return hdg - 36000;
    else
        return hdg;
}

void Sensors::startSensorPolling()
{
    if (!senspacket.info.is_polling)
    {
        sensorcore.println("STARTING POLLING");
        senspacket.info.is_polling = true;
        sensormux.sensPacketUpdate(true);
    }
}

void Sensors::enablePositionEncoders()
{
    robot.enable_speed_encoders = true;
}
