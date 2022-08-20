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
        /**
         * @brief Chiama automaticamente sensors.resetMovementVars()
         * 
         */
        void stop();
        char getDirection();

        /**
         * @brief Mantiene l'heading attuale
         * 
         * @param reverse Determina se il robot sta andando indietro (true) o no (false, valore default)
         */
        void maintainHeading(bool reverse = false);

        /**
         * @brief Imposta la velocità del motore selezionato
         * 
         * @param speed Velocità (0-255)
         * @param motor Motore selezionato. Può essere: RIGHT, LEFT, BOTH, MAIN
         */
        void setSpeed(uint8_t speed, uint8_t motor);

        void playStartSound();

        /**
         * @brief Accende / spegne il motore principale
         *
         * @param speed Velocità (0-255). Default: 170
         * @param motor_status Forza lo spegnimento o accensione del motore. Può essere: STOP, RUNNING. Default: TOGGLE
         * @return true se il motore è acceso
         * @return false se il motore è spento
         */
        bool toggleMainMotor(uint8_t spd = 170, uint8_t motor_status = TOGGLE);

        uint32_t getTime();
};

#endif