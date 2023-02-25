#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include <Arduino.h>
#include <SETTINGS.h>
#include <tuple>
#include <vector>

#define X 0
#define Y 1

class NAV
{
    public:
        struct CommandQueue
        {
            bool busy = false;
            std::vector<uint8_t> commands;
            std::vector<int32_t> data;
        };

        void update();
        void begin();
        // Ruota per un certo numero di gradi automaticamente
        void rotateForDeg(int32_t degs);
        // Ruota per un certo numero di gradi usando la ruota interna alla rotazione come perno
        void rotateForPivot(int32_t degs);
        // Ruota fino a un certo heading, con la direzione di rotazione specificata
        void rotateToDeg(uint32_t heading, char direction);
        /**
         * Va avanti per la distanza specificata
         *
         * @param centimeters Distanza da percorrere
         * @param heading_to_maintain int32_t - default: AUTO
         */
        void goForward(uint32_t centimeters = MAX_CM, int32_t heading_to_maintain = AUTO);
        /**
         * Va indietro per la distanza specificata
         *
         * @param centimeters Distanza da percorrere
         * @param heading_to_maintain int32_t - default: AUTO
         */
        void goBackwards(uint32_t centimeters = MAX_CM, int32_t heading_to_maintain = AUTO);
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
        uint32_t getHeading360();
        // Abilita autorun
        void autoRun();
        // Variabile per sensore IR laterale
        void setLateralObstacle(bool state);

        uint32_t getTime();

        // metodo public perché viene usato dal core0
        /**
         * @brief Converte l'heading relativo a 90, 180, 270 gradi in heading relativo a 0 gradi. Esempio:
         * 315° -> 45° (ref: 270)
         *
         * @param heading Heading formato 360°
         * @param reference Heading relativo di partenza. Può essere: 90, 180, 270
         * @return uint32_t relativo a 0° formato 360°
         */
        uint32_t convertFromRelativeHDGToRef0(uint32_t heading, uint32_t reference);

        /**
         * @brief Converte l'heading relativo a 0 gradi in heading relativo a 90, 180, 270 gradi. Esempio:
         * 45° -> 315° (ref 270)
         *
         * @param heading Heading formato 360°
         * @param reference Heading relativo di target. Può essere: 90, 180, 270
         * @param add_subtract Se true: aggiunge l'heading al riferimento (es: 45 + 270); se false: sottrae l'heading al riferimento (es: (270+90)-45)
         * @return uint32_t Heading relativo a target formato 360°
         */
        uint32_t convertFromRef0ToRelativeHDG(uint32_t heading, uint32_t reference, bool add_subtract = true);

        // metodo public perché viene usato dal core0
        /**
         * @brief Inverte l'heading. Esempio: 0° -> 180°; 270° -> 90°
         *
         * @param heading Heading formato 360°
         * @return uint32_t Heading invertito formato 360°
         */
        uint32_t invertHDG(uint32_t heading);
        void motorStall();
        void mapBorderMode(bool on_off = false);
        void sdspeedtest();
        void eraseSD(const char *path);
        // blocco iniziale: 1
        void readBlock(uint32_t block);

        uint32_t abs(uint32_t val)
        {
            return (val < 0) ? val * -1 : val;
        }
        uint32_t abs(int32_t val)
        {
            return (val < 0) ? val * -1 : val;
        }
        float abs(float val)
        {
            return (val < 0) ? val * -1 : val;
        }
        double abs(double val)
        {
            return (val < 0) ? val * -1 : val;
        }

        void scroll();

        // ferma le ruote del robot (non la logica) se si stava muovendo
        void pause();
        // mette in moto le ruote del robot (avanti) se si stava muovendo
        void resume();

        void mapLogging(bool);

        /**
         * @brief Converte un valore di heading in formato 360° in un valore di heading in formato 180°
         * 
         * @param heading Heading formato 360°
         * @param always_positive Indica se il risultato dev'essere sempre positivo (default: false)
         * @return int32_t Heading in formato 180°
         */
        int32_t convertHDGTo180(uint32_t heading, bool always_positive = false);

    private:
        void resetMovementVars();
        // Restituisce HDG virtuale (0-35999)
        float getVirtualHDG(int32_t heading);
        // Restituisce la direzione in cui il robot deve ruotare
        char getRotationDirection(uint32_t current_heading, int32_t target_heading_180);

        /**
         * @brief Ruota fino a che la variabile non soddisfi la condizione specificata (bool)
         * 
         * @param direction Direzione di rotazione
         * @param variable Variabile da controllare
         * @param condition Condizione che la variabile deve soddisfare
         */
        void rotateUntil(char direction, bool *variable, bool condition);

        /**
         * Ferma il robot e reimposta solo le variabili di movimento principali (going fwd/bck, robot moving, rotating, dst traveled)
         * 
         */
        void stop(bool clear_command_queue = true);

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
        std::tuple<int32_t, int32_t> getVectors(int32_t vector_module, uint32_t vector_direction);

        int countDigits(int number)
        {
            return int(log10(number) + 1);
        }

        /**
         * Cerca in che blocco della mappa il robot si trova, poi assegna i valori in navmap.xvals - navmap.yvals - navmap.idvals
         * 
         * @return uint32_t - blocco
         */
        uint32_t getCurrentBlock();

        /**
         * Cerca in che blocco della mappa le coordinate specificate si trovano, poi assegna i valori in navmap.xvals - navmap.yvals - navmap.idvals
         * 
         * @return uint32_t - blocco
         */
        uint32_t getCurrentBlockFakeXY(int32_t x, int32_t y);

        /**
         * Restituisce la distanza dal punto più vicino.
         * Include chiamata a getCurrentBlock()
         * 
         * @param point_type tipo di punto (0, 1, 2) - 200 == qualsiasi
         * @return std::tuple<uint32_t, uint32_t, uint32_t> - distanza dal punto, id punto, index punto
         */
        std::tuple<uint32_t, uint32_t, uint32_t> getClosestPointDst(uint32_t point_type = 200);

        /**
         * Restituisce la distanza dal punto più vicino, specificando X e Y attuali (non usa i valori reali).
         * Bisogna leggere il blocco in cui si vuole cercare prima di chiamare questa funzione
         * 
         * @param point_type tipo di punto (0, 1, 2) - 200 == qualsiasi
         * @return std::tuple<uint32_t, uint32_t, uint32_t> - distanza dal punto, id punto, index punto, index blocco
         */
        std::tuple<uint32_t, uint32_t, uint32_t, uint32_t> getClosestPointDstFakeXY(int32_t x, int32_t y, uint32_t point_type = 200);

        uint32_t getSDLastReadablePosition();
        // non chiama getSDLastReadablePosition()
        uint32_t getMaxBlock(bool fill_empty_block_space = false);

        /**
         * Ottiene il punto con valore X o Y più alto
         * 
         * @param axis X o Y
         * @return std::tuple<uint32_t, uint32_t, int32_t> - idx blocco, idx punto, valore punto (solo X o Y)
         */
        std::tuple<uint32_t, uint32_t, int32_t> getTopPoint(uint8_t axis);
        /**
         * Ottiene il punto con valore X o Y più basso
         *
         * @param axis X o Y
         * @return std::tuple<uint32_t, uint32_t, int32_t> - idx blocco, idx punto, valore punto (solo X o Y)
         */
        std::tuple<uint32_t, uint32_t, int32_t> getBottomPoint(uint8_t axis = X);
        /**
         * Ottiene il punto con valore X o Y più alto nel blocco specificato
         *
         * @param block
         * @param axis X o Y
         * @return std::tuple<uint32_t, uint32_t, int32_t> - idx blocco, idx punto, valore punto (solo X o Y)
         */
        std::tuple<uint32_t, uint32_t, int32_t> getTopPointOf(uint32_t block, uint8_t axis);
        /**
         * Ottiene il punto con valore X o Y più basso nel blocco specificato
         *
         * @param block
         * @param axis X o Y
         * @return std::tuple<uint32_t, uint32_t, int32_t> - idx blocco, idx punto, valore punto (solo X o Y)
         */
        std::tuple<uint32_t, uint32_t, int32_t> getBottomPointOf(uint32_t block, uint8_t axis);

        std::tuple<int32_t, int32_t> getPointXY(uint32_t block, uint32_t point_idx);

        /**
         * @return uint32_t distanza dal punto
         */
        uint32_t getPointDst(int32_t x, int32_t y);
        /**
         * @return uint32_t distanza dal punto
         */
        uint32_t getPointDst(uint32_t block, uint32_t point_idx);

        /**
         * @brief Gira il robot verso il punto
         * @return uint32_t heading 360
         */
        uint32_t setHeadingToPoint(int32_t x, int32_t y);
        /**
         * @brief Gira il robot verso il punto
         * @return uint32_t heading 360
         */
        uint32_t setHeadingToPoint(uint32_t block, uint32_t point_idx);

        void goToPoint(int32_t x, int32_t y);
        void goToPoint(uint32_t block, uint32_t point_idx);

        /**
         * @brief Unisce i blocchi della mappa. Es: max blocco 1: 2578; min blocco 2: 2772 - unisce i due blocchi renendo il max1: 2675; max2: 2676
         * 
         * @return std::vector<uint32_t> - blocchi uniti, due a due (1, 2, 2, 3, 5, 6, 6, 7)
         */
        uint32_t joinMapBlocks();

        int32_t convertCharVecToInt(std::vector<char> a);

        bool checkMapCompletion();

        /**
         * Inverte l'heading in formato 180. Es: 135° -> 45°; 45° -> 135°
         * Restituisce sempre valori positivi
         */
        uint32_t invert180HDG(int32_t heading);

        uint32_t getMeanDiff(uint32_t block, uint8_t axis);

        void addToCommandQueue(uint8_t cmd, std::vector<int32_t> data, CommandQueue *queue, bool push_forward = false);
        void addToCommandQueue(std::vector<uint8_t> cmds, std::vector<int32_t> data, CommandQueue *queue, bool push_forward = false);

        void displayQueue(CommandQueue *queue);

        /**
         * Ottiene il punto con valore X o Y più alto con X o Y specificato.
         *  
         * @param coordinate
         * @param axis X o Y
         * @return std::tuple<uint32_t, uint32_t, int32_t, int32_t> - idx blocco, idx punto, x punto, y punto
         */
        std::tuple<uint32_t, uint32_t, int32_t, int32_t> getTopPointWith(int32_t coordinate, uint8_t given_coordinate_axis);

        /**
         * Ottiene il punto con valore X o Y più basso con X o Y specificato.
         *  
         * @param coordinate
         * @param axis X o Y
         * @return std::tuple<uint32_t, uint32_t, int32_t, int32_t> - idx blocco, idx punto, x punto, y punto
         */
        std::tuple<uint32_t, uint32_t, int32_t, int32_t> getBottomPointWith(int32_t coordinate, uint8_t given_coordinate_axis);

        uint32_t getBlockContaining(int32_t coordinate, uint8_t axis);
};

#endif