#define _MOTORS_H_
#ifdef _MOTORS_H_

#include <Arduino.h>
#include <SETTINGS.h>

#define MIN_MOTOR_VALUE 60
#define STEPS_MULTIPLIER 10
#define ENCODER_DIAMETER 20.54
#define ACCELERATION_ACTIVATION 300 // il robot sa se si sta muovendo quando l'accelerazione è +- questo valore
// gear ratio - 1:5

class Motors
{
    public:
        void begin();
        void update();
        void forward();
        void backwards();
        void right();
        void left();
        void stop();
        char getDirection();

        /**
         * @brief Mantiene l'heading attuale.
         * 
         * @param reverse Determina se il robot sta andando indietro (true) o no (false, valore default)
         */
        void maintainHeading(bool reverse = false);

        /**
         * Imposta la velocità del motore selezionato.
         * 
         * @param speed Velocità (0-255).
         * @param motor Motore selezionato. Può essere: RIGHT, LEFT, BOTH, MAIN.
         */
        void setSpeed(uint8_t speed, uint8_t motor);

        void playStartSound();

        uint32_t getTime();
};

#endif