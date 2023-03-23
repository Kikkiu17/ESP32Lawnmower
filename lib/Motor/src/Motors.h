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
        /**
         * @param heading_to_maintain int32_t - default: AUTO
         */
        void forward(int32_t heading_to_maintain = AUTO);
        /**
         * @param heading_to_maintain int32_t - default: AUTO
         */
        void backwards(int32_t heading_to_maintain = AUTO);
        void right(bool pivot = false);
        void left(bool pivot = false);
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
        void maintainHeading();

        /**
         * @brief Imposta la velocità del motore selezionato
         * 
         * @param speed Velocità (0-255)
         * @param motor Motore selezionato. Può essere: RIGHT, LEFT, BOTH, MAIN
         */
        void setSpeed(uint32_t speed, uint32_t motor);

        void playStartSound();

        /**
         * @brief Accende / spegne il motore principale
         *
         * @param speed Velocità (0-4096). Default: 1800
         * @param motor_status Forza lo spegnimento o accensione del motore. Può essere: STOP, RUNNING. Default: TOGGLE
         * @return true se il motore è acceso
         * @return false se il motore è spento
         */
        bool toggleMainMotor(uint32_t spd = 2800, uint32_t motor_status = TOGGLE);

        int32_t getHeadingToMaintain();

        uint32_t getTime();

        /**
         * @brief Aggiunge dei gradi all'heading da mantenere, in modo che il robot curvi
         *
         * @param degs Gradi +- 180
         * @return nuovo heading da mantenere
         */
        int32_t addToHeadingToMaintain(int32_t degs);

        void playInitSound();
};

#endif