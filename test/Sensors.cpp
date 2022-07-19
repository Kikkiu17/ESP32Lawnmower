#include <Sensors.h>

float XAccError;
float YAccError;
float ZAccError;

float XAcc;
float YAcc;
float ZAcc;

float XRotVel;
float YRotVel;
float ZRotVel;

float ZHeading;
float pitch;
float roll;

bool timer_started = false;
bool stop_count = false;
unsigned long start;
int pulses;

Adafruit_MPU6050 mpu;

Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

void Sensors::begin()
{
    Wire.begin(21, 22);

    if (!mpu.begin())
    {
        Serial.println("Chip MPU6050 non trovato");
        while (1)
        {
            delay(10);
        }
    }

    Serial.println("Chip MPU6050 trovato");

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    mpu_temp = mpu.getTemperatureSensor();
    mpu_temp->printSensorDetails();

    mpu_accel = mpu.getAccelerometerSensor();
    mpu_accel->printSensorDetails();

    mpu_gyro = mpu.getGyroSensor();
    mpu_gyro->printSensorDetails();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 1, true);
}

void Sensors::getAccelErrors()
{
    //trova i valori di errore

    float XAccTotal;
    float YAccTotal;
    float ZAccTotal;

    //fa una media tra 50 valori per ottenere l'errore
    for(int i = 0; i < 100; i++)
    {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        mpu_temp->getEvent(&temp);
        mpu_accel->getEvent(&accel);
        mpu_gyro->getEvent(&gyro);

        if(i >= 50)
        {
            XAccTotal += accel.acceleration.x;
            YAccTotal += accel.acceleration.y;
            ZAccTotal += accel.acceleration.z;
        }
    }

    XAccTotal = XAccTotal / 50;
    YAccTotal = YAccTotal / 50;
    ZAccTotal = ZAccTotal / 50;

    XAccError = XAccTotal * -1;
    YAccError = YAccTotal * -1;
    ZAccError = (ZAccTotal - 9.81) * -1;

    Serial.print("Valori di errore:   X ");
    Serial.print(XAccError);
    Serial.print("  Y ");
    Serial.print(YAccError);
    Serial.print("  Z ");
    Serial.println(ZAccError);
}

void Sensors::update(float t)
{
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    mpu_temp->getEvent(&temp);
    mpu_accel->getEvent(&accel);
    mpu_gyro->getEvent(&gyro);

    XAcc = accel.acceleration.x - ACCEL_X_ERROR;
    YAcc = accel.acceleration.y - ACCEL_Y_ERROR;
    ZAcc = accel.acceleration.z - ACCEL_Z_ERROR;

    float gyro_velocity = gyro.gyro.z;

    if (gyro_velocity > GYRO_SENSITIVITY || gyro_velocity < -GYRO_SENSITIVITY)
    {
        ZRotVel = gyro.gyro.z * 57.2958 + 1.6;

        ZHeading = ZHeading + (ZRotVel * t);
    }
}

float Sensors::getAccX()
{
    sensors_event_t accel;
    mpu_accel->getEvent(&accel);

    XAcc = accel.acceleration.x - ACCEL_X_ERROR;

    return XAcc;
}

float Sensors::getAccY()
{
    sensors_event_t accel;
    mpu_accel->getEvent(&accel);

    YAcc = accel.acceleration.y - ACCEL_Y_ERROR;

    return YAcc;
}

float Sensors::getAccZ()
{
    sensors_event_t accel;
    mpu_accel->getEvent(&accel);

    ZAcc = accel.acceleration.z - ACCEL_Z_ERROR;

    return ZAcc;
}

float Sensors::getRoll(float t)
{
    sensors_event_t gyro;
    mpu_gyro->getEvent(&gyro);

    float gyro_velocity = gyro.gyro.x;

    if (gyro_velocity > GYRO_SENSITIVITY || gyro_velocity < -GYRO_SENSITIVITY)
    {
        XRotVel = gyro.gyro.x * 57.2958 + 1.6;

        roll = roll + XRotVel * t;
    }

    return roll;
}

float Sensors::getPitch(float t)
{
    sensors_event_t gyro;
    mpu_gyro->getEvent(&gyro);

    float gyro_velocity = gyro.gyro.y;

    if (gyro_velocity > GYRO_SENSITIVITY || gyro_velocity < -GYRO_SENSITIVITY)
    {
        YRotVel = gyro.gyro.y * 57.2958 + 1.6;

        pitch = pitch + YRotVel * t;
    }

    return pitch;
}

float Sensors::getHeading(float t)
{
    sensors_event_t gyro;
    mpu_gyro->getEvent(&gyro);

    float gyro_velocity = gyro.gyro.z;

    if (gyro_velocity > GYRO_SENSITIVITY || gyro_velocity < -GYRO_SENSITIVITY)
    {
        ZRotVel = gyro.gyro.z * 57.2958 + 1.6;

        ZHeading = ZHeading + ZRotVel * t;
    }

    return ZHeading;
}

void Sensors::getRPMFrequency()
{
    pulses++;
    //Serial.println(pulses);
}
