#include <SPI.h>
#include "Wire.h"
#include <MPU6050_light.h>

#include <AS5600.h>
#include <Sensors.h>
#include <Status.h>
#include <Motors.h>
#include <Core.h>
#include <Navigation.h>
#include <Mux.h>

// l'mpu e l'encoder sinistro usano entrambi il bus 0 (sda 11, scl 12) anche se sulla pcb dovrebbero usare bus diversi (sistemato in rev3)
// questo perché l'esp32s3 ha solo 2 bus I2C quindi non potrebbe gestire tutte e 3 le periferiche
AS5600* encleft = new AS5600(11, 12, 0);
AS5600* encright = new AS5600(9, 10, 1);
MPU6050 mpu = MPU6050(Wire);
Motors sensormotors;
Core sensorcore;
NAV sensornav;
Mux sensormux;
unsigned long t2 = 0;

unsigned int sens_refresh_time = millis();
unsigned int inactivity_count = 0;
unsigned long *sensor_packetptr;

float yaw = 0;
float pitch = 0;
float roll = 0;

class EncoderData
{
    public:
        long revolutions = 0;
        unsigned int read_time = 0;
        unsigned int last_read_time = 0;
        unsigned long absolute_revolutions = 0;
        long total_angle = 0;
        long scaled_angle = 0;
        long last_scaled_angle = 0;
        int diff = 0;
        // w = vt / r
        // r = 147 mm
        float angular_velocity = 0;
};

struct SetZero
{
    bool active = false;
    uint8_t arr_idx = 0;
    int arrYaw[500] {};
    int arrPitch[500] = {};
    int arrRoll[500] = {};
    int old_yaw = 0;
    int zero_yaw = 0;
    int zero_pitch = 0;
    int zero_roll = 0;
    int max_yaw = 0;
    int max_pitch = 0;
    int max_roll = 0;
    int min_yaw = 0;
    int min_pitch = 0;
    int min_roll = 0;
};

struct SensorPacket
{
    struct
    {
        unsigned long us_f = 0;
        unsigned long us_l = 0;
        unsigned long us_r = 0;
        unsigned long ir_f = 0;
        unsigned long ir_l = 0;
        bool check_next_dst = false;
        unsigned long check_dst_packet = 0;
    } obstacle;

    struct
    {
        unsigned long value = 0;
        unsigned long n_values = 0;
        unsigned long total = 0;
        bool waiting = false;

    } bat;

    unsigned long read_digital = 0;
    unsigned long read_analog = 0;

    struct
    {
        unsigned long id = 0;
        unsigned long last_id = 0;
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
    Wire.setClock(800000L);
    Wire1.setClock(800000L);
    if (encleft->isMagnetDetected())
    {
        sensorcore.print(F("(Sensors) LEFT Magnet detected,"));
        if (!encleft->isMagnetTooStrong())
            if (!encleft->isMagnetTooWeak())
                sensorcore.println(F(" OK"));
            else
                sensorcore.println(F(" TOO WEAK"));
        else
            sensorcore.println(F(" TOO STRONG"));
    }
    else
        sensorcore.println(F("(Sensors) LEFT Magnet NOT DETECTED! Positioning not available!"));

    if (encright->isMagnetDetected())
    {
        sensorcore.print(F("(Sensors) RIGHT Magnet detected,"));
        if (!encright->isMagnetTooStrong())
            if (!encright->isMagnetTooWeak())
                sensorcore.println(F(" OK"));
            else
                sensorcore.println(F(" TOO WEAK"));
        else
            sensorcore.println(F(" TOO STRONG"));
    }
    else
        sensorcore.println(F("(Sensors) RIGHT Magnet NOT DETECTED! Positioning not available!"));

    sensorcore.print(F("(Sensors) Initializing MPU6050,"));
    if (mpu.begin() == 0)
    {
        sensorcore.println(F(" OK"));
        sensorcore.print(F("(Sensors) Calibrating accelerometer / gyro,"));
        mpu.calcOffsets(true, true);
        sensorcore.println(F(" OK"));
        sensor_packetptr = sensormux.getPacketPointer();
    }
    else
        sensorcore.println(F(" CANNOT INITIALIZE MPU!"));

    pinMode(REF_BAT, INPUT);
}

unsigned int temp_time = 0;
unsigned int pause_time = 0;

void pausedEvent()
{
    mpu.calcOffsets();
    sensornav.resume();
}

void Sensors::update()
{
    unsigned int start_time = esp_timer_get_time();

    if (millis() - temp_time > 30)
    {
        if (!senspacket.info.is_polling)
            inactivity_count++;
        else
            inactivity_count = 0;

        if (analogRead(REF_BAT) < 2600 && analogRead(REF_BAT) > 1500)
            sensorcore.lowBat();
            
        if (inactivity_count > 20000) // 10 minuti
        {
            
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

    /*if (millis() - pause_time > MPU_CALIBRATION_TIME * 1000)
    {
        sensornav.pause(&pausedEvent);
        pause_time = millis() - 500;
    }*/

    if (millis() - sens_refresh_time > MPU_SPD_SENSORS_REFRESH_RATE)
    {
        mpu.update();
        unsigned int direction = sensormotors.getDirection();

        if (direction == FWD && ENABLE_OBSTACLE_AVOIDANCE)
        {
            if (!senspacket.info.is_polling)
            {
                sensorcore.println("STARTING POLLING");
                senspacket.info.is_polling = true;
                sensormux.sensPacketUpdate(true);
            }
        }

        if (ENABLE_SPD_SENSORS)
        {
            if (robot.enable_speed_encoders)
            {
                getEncoderRightAngle();
                getEncoderLeftAngle();
                if (robot.first_iteration_ignored)
                {
                    unsigned int left_time_diff = leftEncoder.read_time - leftEncoder.last_read_time;
                    unsigned int right_time_diff = rightEncoder.read_time - rightEncoder.last_read_time;
                    float left_velocity = (float)leftEncoder.diff / (float)left_time_diff * 0.45 /*(WHEEL_DIAMETER / 2) / 180 = 0.45*/ * PI;
                    float right_velocity = (float)rightEncoder.diff / (float)right_time_diff * 0.45 /*(WHEEL_DIAMETER / 2) / 180 = 0.45*/ * PI;
                    float robot_velocity = (left_velocity + right_velocity) / 2; // cm/s
                    robot.traveled_distance_raw = robot_velocity * (((float)left_time_diff + (float)right_time_diff) / 2000.0); // cm
                    robot.traveled_distance += robot.traveled_distance_raw;
                    robot.last_traveled_distance = robot.traveled_distance_raw;
                    leftEncoder.angular_velocity = left_velocity / 14.7; // tutto in cm
                    rightEncoder.angular_velocity = right_velocity / 14.7; // tutto in cm
                    unsigned long angle_time = millis();
                    float inst = degrees((leftEncoder.angular_velocity - rightEncoder.angular_velocity) * ((float)(millis() - robot.last_angle_time) / 1000.0));
                    robot.last_angle_time = angle_time;
                    robot.angle += inst;
                    Serial.printf("%f %f %f\n", robot.angle, right_velocity, left_velocity, inst, leftEncoder.angular_velocity, left_velocity, rightEncoder.angular_velocity, right_velocity, robot.traveled_distance_raw);
                }
                else
                {
                    robot.last_angle_time = millis();
                    robot.first_iteration_ignored = true;   // diventa false quando viene chiamato setMotorsStop()
                }
            }
        }

        sens_refresh_time = millis();
    }

    t2 = esp_timer_get_time() - start_time;
}

unsigned int Sensors::getTime()
{
    return t2;
}

void Sensors::getValues()
{
    // è generalmente meglio liberare tutti i registri in un colpo solo
    yaw = mpu.getAngleZ();
    pitch = mpu.getAngleY();
    roll = mpu.getAngleX();
}

int Sensors::getRoll()
{
    getValues();
    if (INVERT_ROLL)
        roll *= -1;
    return roll * 100;
}

int Sensors::getPitch()
{
    getValues();
    if (INVERT_PITCH)
        pitch *= -1;
    return pitch * 100;
}

int Sensors::getHeading()
{
    getValues();
    if (INVERT_YAW)
        yaw *= -1;
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
    leftEncoder.last_read_time = leftEncoder.read_time;
    leftEncoder.last_scaled_angle = leftEncoder.scaled_angle;
    leftEncoder.scaled_angle = (long)(encleft->getScaledAngle() * 100);
    leftEncoder.read_time = millis();
    if (abs(leftEncoder.last_scaled_angle - leftEncoder.scaled_angle) > 18000)
    {
        leftEncoder.absolute_revolutions++;
        if (leftEncoder.last_scaled_angle > leftEncoder.scaled_angle)
            leftEncoder.revolutions--;
        else
            leftEncoder.revolutions++;
    }

    int diff = leftEncoder.scaled_angle - leftEncoder.last_scaled_angle;
    while (diff <= -18000)
        diff += 36000;
    while (diff > 18000)
        diff -= 36000;

    leftEncoder.diff = diff;
    leftEncoder.total_angle += diff;
}

void Sensors::getEncoderRightAngle()
{
    rightEncoder.last_read_time = rightEncoder.read_time;
    rightEncoder.last_scaled_angle = rightEncoder.scaled_angle;
    rightEncoder.scaled_angle = (long)(encright->getScaledAngle() * 100);
    rightEncoder.read_time = millis();
    if (abs(rightEncoder.last_scaled_angle - rightEncoder.scaled_angle) > 18000)
    {
        rightEncoder.absolute_revolutions++;
        if (rightEncoder.last_scaled_angle > rightEncoder.scaled_angle)
            rightEncoder.revolutions--;
        else
            rightEncoder.revolutions++;
    }

    int diff = rightEncoder.last_scaled_angle - rightEncoder.scaled_angle;
    while (diff <= -18000)
        diff += 36000;
    while (diff > 18000)
        diff -= 36000;

    rightEncoder.diff = diff;
    rightEncoder.total_angle += diff;
}

long Sensors::invert180HDG(long hdg)
{
    long return_hdg = hdg + 18000;
    if (return_hdg - 36000 > 0)
        return return_hdg - 36000;
    else
        return return_hdg;
}

long Sensors::convert360To180HDG(long hdg, bool always_positive)
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
