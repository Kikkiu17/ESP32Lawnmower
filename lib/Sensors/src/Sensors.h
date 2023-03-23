#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <Arduino.h>
#include <SETTINGS.h>

class Sensors
{
    public:
        void begin();
        void update();

        int getAccX();
        int getAccY();
        int getAccZ();
        int32_t getRoll();
        int32_t getPitch();
        int32_t getHeading();

        /**
         * @brief Reset delle variabili di movimento - non chiama motors.stop()
         * 
         */
        void resetMovementVars();

        /**
         * @brief Controlla se c'è un ostacolo davanti al robot
         *
         * @param request_type Tipo di richiesta. Può essere: DEFAULT, MOTORS
         */
        void checkFrontObstacle(uint8_t request_type = DEFAULT);

        float getUltrasonicDistance();
        void setMotorsStop();
        float getTraveledDistance();
        
        /**
         * @brief Ottiene quanta strada ha fatto l'ultima volta (non totale)
         * 
         * @return float 
         */
        float getLastTraveledDistance();
        void resetTraveledDistance();
        void returnUSDistance(float distance);
        uint32_t getTime();
        struct SelfTest* getSelfTestResults();

        class Robot
        {
            public:
                float traveled_distance_raw = 0;
                float traveled_distance = 0;
                float last_traveled_distance = 0;
                bool first_iteration_ignored = false;
                bool enable_speed_encoders = 0;
        };

        void startSensorPolling();
        void enablePositionEncoders();

    private:
        void getValues();
        void getFrontUSObstacle(uint8_t request_type = DEFAULT);
        void getAccelErrors();

        void getEncoderLeftAngle();
        void getEncoderRightAngle();

        const void selectEncoderLeft();
        const void selectEncoderRight();

        /**
         * Inverte l'heading in formato 180.
         * Accetta e restituisce sempre e solo valori positivi
         * Se first_half e second_half sono entrambi TRUE, inverte tutti gli heading. Es: 135° -> 45°; 45° -> 135°
         */
        int64_t invert180HDG(int64_t hdg);
         /**
         * @brief Converte un valore di heading in formato 360° in un valore di heading in formato 180°
         * 
         * @param heading Heading formato 360°
         * @param always_positive Indica se il risultato dev'essere sempre positivo (default: false)
         * @return int32_t Heading in formato 180°
         */
        int64_t convert360To180HDG(int64_t hdg, bool always_positive = false);
};

#endif
