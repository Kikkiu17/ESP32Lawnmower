#define _SENSORS_H_
#ifdef _SENSORS_H_

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <SETTINGS.h>

class Sensors
{
    public:
        void begin();
        void update();
        void setZero();

        int getAccX();
        int getAccY();
        int getAccZ();
        int32_t getRoll();
        int32_t getPitch();
        int32_t getHeading();
        void checkMovement(char);
        void checkRotation();
        void stopMoving();

        /**
         * @brief Controlla se c'è un ostacolo davanti al robot
         *
         * @param request_type Tipo di richiesta. Può essere: DEFAULT, MOTORS
         * @return true,
         * @return false
         */
        bool checkFrontObstacle(uint8_t request_type = DEFAULT);
        int getFWDInfrared();

        void getAccelErrors();
        float getUltrasonicDistance();
        void setMotorsRotating();
        void setMotorsStop();
        float getTraveledDistance();
        
        /**
         * @brief Ottiene quanta strada ha fatto l'ultima volta (non totale)
         * 
         * @return float 
         */
        float getLastTraveledDistance();
        void resetTraveledDistance();
        void setAutoRun(bool state);

        void returnUSDistance(float distance);

        uint32_t getTime();

    private:
        /**
         * @brief Ottiene il periodo di ogni segnale ricevuto dall'encoder
         * 
         * @return int Periodo encoder - restituisce -1 se non è ancora pronto
         */
        int getEncoderPeriod();
        int getLeftInfrared();
        void getValues();
        void getFrontUSObstacle(uint8_t request_type = DEFAULT);
};

#endif
