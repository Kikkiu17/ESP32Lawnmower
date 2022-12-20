#include <MPU6050.h>
#include <Arduino.h>
#include "Wire.h"
#include <math.h>

float AccX, AccY, AccZ = 0;
float GyroX, GyroY, GyroZ = 0; // velocità angolare
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ = 0;
float accAngleX, accAngleY = 0;
float gyroAngleX, gyroAngleY, yaw1;
float roll1, pitch1, angle;
uint32_t currentTime, previousTime, elapsedTime = 0;

std::tuple<float, float, float> MPU6050::readAcc() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, 1); // il valore di ogni asse è salvato in 2 registri
    // vedere il datasheet - sensibilità di ±2g
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

    return std::make_tuple(AccX, AccY, AccZ);
}

std::tuple<float, float, float> MPU6050::readGyro() {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, 1);
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    return std::make_tuple(GyroX, GyroY, GyroZ);
}

std::tuple<float, float, float, float, float> MPU6050::calculateError() {
    uint32_t c = 0;
    while (c < 200) {
        readAcc();
        AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / M_PI);
        AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / M_PI);
        c++;
    }
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    c = 0;

    while (c < 200) {
        readGyro();
        GyroErrorX += GyroX;
        GyroErrorY += GyroY;
        GyroErrorZ += GyroZ;
        c++;
    }

    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
    Serial.println(F("Calibrated errors:"));
    Serial.print(F("Acc X: "));
    Serial.println(AccErrorX);
    Serial.print(F("Acc Y: "));
    Serial.println(AccErrorY);
    Serial.print(F("Gyro X: "));
    Serial.println(GyroErrorX);
    Serial.print(F("Gyro Y: "));
    Serial.println(GyroErrorY);
    Serial.print(F("Gyro Z: "));
    Serial.println(GyroErrorZ);

    if (AccErrorX == 0 || AccErrorY == 0 || GyroErrorX == 0 || GyroErrorY == 0 || GyroErrorZ == 0)
        Serial.println(F("Strange calibration data!"));    

    return std::make_tuple(AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ);
}

void MPU6050::initialize()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x10);
    Wire.endTransmission(true);
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);
}

uint8_t MPU6050::testConnection()
{
    Wire.beginTransmission(0x68);
    return Wire.endTransmission();
}

void MPU6050::update()
{
    readAcc();
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

    // === Read gyroscope (on the MPU6050) data === //
    previousTime = currentTime;
    currentTime = micros();
    elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
    readGyro();
    // Correct the outputs with the calculated error values
    GyroX -= GyroErrorX; //GyroErrorX is calculated in the calculateError() function
    GyroY -= GyroErrorY;
    GyroZ -= GyroErrorZ;
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY += GyroY * elapsedTime;
    yaw1 += GyroZ * elapsedTime;
    //combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
    roll1 = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch1 = 0.96 * gyroAngleY + 0.04 * accAngleY;
    angle = roll1; //if you mounted MPU6050 in a different orientation to me, angle may not = roll. It can roll, pitch, yaw or minus version of the three

    Serial.print("R: ");
    Serial.print(roll1);
    Serial.print(" P: ");
    Serial.print(pitch1);
    Serial.print(" Y: ");
    Serial.println(yaw1);
}
