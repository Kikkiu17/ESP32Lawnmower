#include <Sensors.h>
#include <SPI.h>
#include "Wire.h"
#include <Status.h>
#include <Motors.h>
#include <Core.h>
#include <Navigation.h>
#include <BluetoothSerial.h>
#include <Mux.h>
#include <AS5600.h>
#include <MPU6050_light.h>

AS5600 encoder;
Motors sensormotors;
Core sensorcore;
NAV sensornav;
BluetoothSerial sensorserial;
Mux sensormux;
uint64_t t2 = 0;
MPU6050 mpu(Wire);

uint32_t sens_refresh_time = millis();
uint32_t inactivity_count = 0;
uint64_t *sensor_packetptr;

int32_t accX = 0;
int32_t accY = 0;
int32_t accZ = 0;
float yaw = 0;
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

EncoderData rightEncoder;
EncoderData leftEncoder;
SetZero setzero;
SensorPacket senspacket;
Sensors::Robot robot;

void Sensors::begin()
{
    Wire.begin();
    Wire.setClock(400000L);
    sensorcore.print(F("(Sensors) Initializing MPU6050,"));
    if (mpu.begin() == 0)
    {
        sensorcore.println(F(" OK"));
        sensorcore.print(F("(Sensors) Calibrating accelerometer / gyro,"));
        mpu.calcOffsets(true, true);
        sensorcore.println(F(" OK"));
        pinMode(INTERRUPT_PIN, INPUT);
        pinMode(LEFT_SPD_SENS, OUTPUT);
        pinMode(RIGHT_SPD_SENS, OUTPUT);
        sensor_packetptr = sensormux.getPacketPointer();
    }
    else
        sensorcore.println(F(" CANNOT INITIALIZE MPU!"));

    selectEncoderLeft();
    if (encoder.isMagnetDetected())
    {
        sensorcore.print(F("(Sensors) LEFT Magnet detected,"));
        if (!encoder.isMagnetTooStrong())
            if (!encoder.isMagnetTooWeak())
                sensorcore.println(F(" OK"));
            else
                sensorcore.println(F(" TOO WEAK"));
        else
            sensorcore.println(F(" TOO STRONG"));
    }
    else
        sensorcore.println(F("(Sensors) LEFT Magnet NOT DETECTED! Do not use positioning!"));

    selectEncoderRight();
    if (encoder.isMagnetDetected())
    {
        sensorcore.print(F("(Sensors) RIGHT Magnet detected,"));
        if (!encoder.isMagnetTooStrong())
            if (!encoder.isMagnetTooWeak())
                sensorcore.println(F(" OK"));
            else
                sensorcore.println(F(" TOO WEAK"));
        else
            sensorcore.println(F(" TOO STRONG"));
    }
    else
        sensorcore.println(F("(Sensors) RIGHT Magnet NOT DETECTED! Do not use positioning!"));
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
                    if (senspacket.bat.total / 20 < 2600 && senspacket.bat.total / 20 > 700) // sotto circa 12V
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

    if (millis() - sens_refresh_time > MPU_SPD_SENSORS_REFRESH_RATE)
    {
        mpu.update();
        char direction = sensormotors.getDirection();
        if (direction == 'w' && ENABLE_OBSTACLE_AVOIDANCE)
        {
            if (!senspacket.info.is_polling)
            {
                sensorserial.println("STARTING POLLING");
                senspacket.info.is_polling = true;
                sensormux.sensPacketUpdate(true);
            }
        }

        if (ENABLE_SPD_SENSORS)
        {
            if (direction == 'w' || direction == 's')
            {
                getEncoderRightAngle();
                getEncoderLeftAngle();
                if (robot.first_iteration_ignored)
                {
                    uint32_t left_time_diff = leftEncoder.read_time - leftEncoder.last_read_time;
                    uint32_t right_time_diff = rightEncoder.read_time - rightEncoder.last_read_time;
                    float left_velocity = (float)leftEncoder.diff / (float)left_time_diff * 0.45 /*(WHEEL_DIAMETER / 2) / 180 = 0.45*/ * PI;
                    float right_velocity = (float)rightEncoder.diff / (float)right_time_diff * 0.45 /*(WHEEL_DIAMETER / 2) / 180 = 0.45*/ * PI;
                    float robot_velocity = (left_velocity + right_velocity) / 2; // cm/s
                    robot.traveled_distance_raw = robot_velocity * (((float)left_time_diff + (float)right_time_diff) / 2000.0); // cm
                    robot.traveled_distance += robot.traveled_distance_raw;
                    robot.last_traveled_distance = robot.traveled_distance_raw;
                }
                else
                    robot.first_iteration_ignored = true;   // diventa false quando viene chiamato setMotorsStop()
            }
        }

        sens_refresh_time = millis();
    }

    t2 = micros() - start_time;
}

uint32_t Sensors::getTime()
{
    return t2;
}

void Sensors::getValues()
{
    // è generalmente meglio liberare tutti i registri in un colpo solo
    yaw = mpu.getAngleZ();
    pitch = mpu.getAngleY();
    roll = mpu.getAngleX();
    accX = mpu.getAccX();
    accY = mpu.getAccY();
    accZ = mpu.getAccZ();
}

int Sensors::getAccX()
{
    getValues();
    if (INVERT_ACC_X)
        accX *= -1;
    return accX;
}

int Sensors::getAccY()
{
    getValues();
    if (INVERT_ACC_Y)
        accY *= -1;
    return accY;
}

int Sensors::getAccZ()
{
    getValues();
    if (INVERT_ACC_Z)
        accZ *= -1;
    return accZ;
}

int32_t Sensors::getRoll()
{
    getValues();
    if (INVERT_ROLL)
        roll *= -100;
    return roll;
}

int32_t Sensors::getPitch()
{
    getValues();
    if (INVERT_PITCH)
        pitch *= -100;
    return pitch;
}

int32_t Sensors::getHeading()
{
    getValues();
    if (INVERT_YAW)
        yaw *= -100;
    return yaw;
}

void Sensors::resetMovementVars()
{
    senspacket.info.is_polling = false;
    sensormux.sensPacketUpdate(false);
}

void Sensors::setMotorsStop()
{
    robot.first_iteration_ignored = false;
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

void Sensors::getEncoderLeftAngle()
{
    digitalWrite(RIGHT_SPD_SENS, LOW);
    delayMicroseconds(1);
    digitalWrite(LEFT_SPD_SENS, HIGH);
    delayMicroseconds(15);
    leftEncoder.last_read_time = leftEncoder.read_time;
    leftEncoder.last_scaled_angle = leftEncoder.scaled_angle;
    leftEncoder.scaled_angle = (int64_t)(encoder.getScaledAngle() * 100);
    leftEncoder.read_time = millis();
    if (abs(leftEncoder.last_scaled_angle - leftEncoder.scaled_angle) > 18000)
    {
        leftEncoder.absolute_revolutions++;
        if (leftEncoder.last_scaled_angle > leftEncoder.scaled_angle)
            leftEncoder.revolutions--;
        else
            leftEncoder.revolutions++;
    }

    int32_t diff = leftEncoder.scaled_angle - leftEncoder.last_scaled_angle;
    while (diff <= -18000)
        diff += 36000;
    while (diff > 18000)
        diff -= 36000;

    leftEncoder.diff = diff;
    leftEncoder.total_angle += diff;
}

void Sensors::getEncoderRightAngle()
{
    digitalWrite(LEFT_SPD_SENS, LOW);
    delayMicroseconds(1);
    digitalWrite(RIGHT_SPD_SENS, HIGH);
    delayMicroseconds(15);
    rightEncoder.last_read_time = rightEncoder.read_time;
    rightEncoder.last_scaled_angle = rightEncoder.scaled_angle;
    rightEncoder.scaled_angle = (int64_t)(encoder.getScaledAngle() * 100);
    rightEncoder.read_time = millis();
    if (abs(rightEncoder.last_scaled_angle - rightEncoder.scaled_angle) > 18000)
    {
        rightEncoder.absolute_revolutions++;
        if (rightEncoder.last_scaled_angle > rightEncoder.scaled_angle)
            rightEncoder.revolutions--;
        else
            rightEncoder.revolutions++;
    }

    int32_t diff = rightEncoder.last_scaled_angle - rightEncoder.scaled_angle;
    while (diff <= -18000)
        diff += 36000;
    while (diff > 18000)
        diff -= 36000;

    rightEncoder.diff = diff;
    rightEncoder.total_angle += diff;
}

const void Sensors::selectEncoderLeft()
{
    digitalWrite(RIGHT_SPD_SENS, LOW);
    delayMicroseconds(10);
    digitalWrite(LEFT_SPD_SENS, HIGH);
    delayMicroseconds(10);
}

const void Sensors::selectEncoderRight()
{
    digitalWrite(LEFT_SPD_SENS, LOW);
    delayMicroseconds(10);
    digitalWrite(RIGHT_SPD_SENS, HIGH);
    delayMicroseconds(10);
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
