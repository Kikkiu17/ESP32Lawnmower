#define _NAVIGATION_H_
#ifdef _NAVIGATION_H_

#include <Arduino.h>
#include <SETTINGS.h>
#include <tuple>

class NAV
{
    public:
        void update();
        void begin();
        // Ruota per un certo numero di gradi automaticamente
        void rotateForDeg(int16_t heading);
        // Ruota fino a un certo heading, con la direzione di rotazione specificata
        void rotateToDeg(uint16_t heading, char direction);
        // Va avanti per la distanza specificata
        void goForward(uint32_t centimeters = 4294967295);
        // Va indietro per la distanza specificata
        void goBackwards(uint32_t centimeters = 4294967295);
        /**
         * @brief Imposta la flag "obstacle_detected" (solo se il robot si stava muovendo)
         *
         * @param sensor_type Tipo di sensore. Può essere: INFRARED, ULTRASONIC
         * @param sensor_direction Direzione del sensore. Può essere: LEFT, FRONT, RIGHT
         */
        void obstacleDetectedWhileMoving(uint8_t sensor_type, uint8_t sensor_direction);
        /**
         * @brief Imposta la flag "obstacle_detected_before_moving" (solo se il robot partiva da fermo)
         *
         * @param sensor_type Tipo di sensore. Può essere: INFRARED, ULTRASONIC
         * @param sensor_direction Direzione del sensore. Può essere: LEFT, FRONT, RIGHT
         */
        void obstacleDetectedBeforeMoving(uint8_t sensor_type, uint8_t sensor_direction);
        // Resetta le variabili (stop da seriale)
        void externalStop();
        // Deprecato
        void uturnDelay();
        // Restituisce l'heading da 0 a 35999 (moltiplicato per 100)
        uint16_t getHDG();
        // Abilita autorun
        void autoRun();
        // Imposta la flag "sudden_stop" (robot bloccato fisicamente, rilevato da IMU)
        void suddenStop(char direction);
        // Variabile per sensore IR laterale
        void setLateralObstacle(bool state);

        uint32_t getTime();

        // metodo public perché viene usato dal core0
        /**
         * @brief Converte l'heading in modo che sia relativo a 90, 180, 270 gradi
         *
         * @param heading Heading formato 360°
         * @param reference Heading di riferimento. Può essere: 90, 180, 270 (int)
         * @return uint16_t Heading relativo formato 360°
         */
        uint16_t convertRelativeHDG(uint16_t heading, uint16_t reference);

        // metodo public perché viene usato dal core0
        /**
         * @brief Inverte l'heading. Esempio: 0° -> 180°; 270° -> 90°
         *
         * @param heading Heading formato 360°
         * @return uint16_t Heading invertito formato 360°
         */
        uint16_t invertHDG(uint16_t heading);

        void test_func();
        void mapBorderMode(bool on_off = false);

        void sdspeedtest();
        void eraseSD(const __FlashStringHelper *path);
        void readBlock(uint32_t block);

    private:
        // Inutilizzato
        void checkDirectionsDeg();
        void resetMovementVars();
        void enableUTurn();
        void disableUTurn();
        void enableSuddenStopAvoid();
        void disableSuddenStopAvoid();
        // Restituisce HDG virtuale (0-35999)
        float getVirtualHDG(int16_t heading);
        // Restituisce la direzione in cui il robot deve ruotare
        char getRotationDirection(uint16_t current_heading, uint16_t target_heading);

        /**
         * @brief Ruota fino a che la variabile non soddisfi la condizione specificata (bool)
         * 
         * @param direction Direzione di rotazione
         * @param variable Variabile da controllare
         * @param condition Condizione che la variabile deve soddisfare
         */
        void rotateUntil(char direction, bool *variable, bool condition);

        /**
         * @brief Converte un valore di heading in formato 360° in un valore di heading in formato 180°
         * 
         * @param heading Heading formato 360°
         * @param always_positive Indica se il risultato dev'essere sempre positivo (default: false)
         * @return int32_t Heading in formato 180°
         */
        int32_t convertHDGTo180(uint16_t heading, bool always_positive = false);

        /**
         * Ferma il robot e reimposta solo le variabili di movimento principali (going fwd/bck, robot moving, rotating, dst traveled)
         * 
         */
        void stop();

        /**
         * Aggiunge un certo valore ai vettori di distanza percorsa attuali e li restituisce senza modificarli effettivamente
         *
         * @param value_to_add valore da aggiungere ai vettori
         * @param heading heading formato 360°
         * @return std::tuple<int32_t, int32_t> - primo valore: vettore X; secondo valore: vettore Y
         */
        std::tuple<int32_t, int32_t> addToVectors(int32_t value_to_add, uint32_t heading);

        /**
         * Scompone il vettore dato
         *
         * @param value_to_add lunghezza vettore
         * @param heading angolo vettore formato 360°
         * @return std::tuple<int32_t, int32_t> - primo valore: vettore X; secondo valore: vettore Y
         */
        std::tuple<int32_t, int32_t> getVectors(int32_t value_to_add, uint32_t heading);

        int countDigits(int number)
        {
            return int(log10(number) + 1);
        }

        /**
         * Cerca in che blocco della mappa il robot si trova, poi carica i valori in navmap.xvals - navmap.yvals - navmap.idvals
         * 
         * @return uint32_t - blocco
         */
        uint32_t getCurrentBlock();

        /**
         * Restituisce la distanza dal punto più vicino.
         * Include chiamata a getCurrentBlock()
         * 
         * @param point_type tipo di punto
         * @return std::tuple<uint32_t, uint32_t, uint32_t> - distanza dal punto, id punto, index punto
         */
        std::tuple<uint32_t, uint32_t, uint32_t> getClosestPointDst(uint32_t point_type = 200);
};

#endif