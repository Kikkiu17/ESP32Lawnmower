#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include <Arduino.h>
#include "SETTINGS.h"
#include <tuple>
#include <vector>

#define X 0
#define Y 1

#define MAP_POINT_SIZE 6        // sizeof(NAV::Point) - byte
#define MAP_BLOCK_SIZE 1536     // MAP_POINT_SIZE * 256 - byte
#define POINTS_PER_BLOCK 256
#define MAP_SQUARE_WIDTH 256

class Point
{
public:
    short x, y, id = 0;
    Point() {}
    Point(short inx, short iny) { x = inx; y = iny; }
    Point(short inx, short iny, short inid) { x = inx; y = iny; id = inid; }

    bool operator==(const Point& other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point& other) const
    {
        return x != other.x || y != other.y;
    }

    bool operator<(const Point& other) const
    {
        return x < other.x || (!(other.x < x) && y < other.y);
    }
};

class NAV
{
public:
    struct CommandQueue
    {
        bool busy = false;
        std::vector<unsigned int> commands;
        std::vector<int> data;
    };

    class MapCreationInfo
    {
        public:
            unsigned int fill_start_pos = 0;
            unsigned int width = 0;
            unsigned int height = 0;
            int minx = 0;
            int miny = 0;
            int maxx = 0;
            int maxy = 0;
            unsigned int height_in_squares = 0;
            unsigned int width_in_squares = 0;
    };
    
    unsigned int abs(unsigned int val)
    {
        return (val < 0) ? val * -1 : val;
    }
    int abs(int val)
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

    /**
     * @brief Punto con anche informazioni su blocco, idx, id. Usare Point se queste informazioni non servono, dato che è più leggero (-14 B)
    */
    class MapPoint
    {
        public:
            int x, y, block = 0;
            uint16_t idx = 0;
            uint8_t id = 0;
            bool exists = true;
            MapPoint(int inx, int iny, int inblock, uint16_t inidx, unsigned int inid)
            {
                x = inx;
                y = iny;
                block = inblock,
                idx = inidx;
                id = inid;
            }

            MapPoint(bool does_exist) { exists = does_exist; }
            MapPoint() {}
    };

    void update();
    void begin();
    // Ruota per un certo numero di gradi automaticamente
    void rotateForDeg(int degs);
    // Ruota per un certo numero di gradi usando la ruota interna alla rotazione come perno
    void rotateForPivot(int degs);
    // Ruota fino a un certo heading, con la direzione di rotazione specificata
    void rotateToDeg(unsigned int heading, unsigned int direction);
    /**
     * Va avanti per la distanza specificata
     *
     * @param centimeters Distanza da percorrere
     * @param heading_to_maintain int - default: AUTO
     */
    void goForward(unsigned int centimeters = 2147483646, int heading_to_maintain = 2147748);
    /**
     * Va indietro per la distanza specificata
     *
     * @param centimeters Distanza da percorrere
     * @param heading_to_maintain int - default: AUTO
     */
    void goBackwards(unsigned int centimeters = 2147483646, int heading_to_maintain = 2147748);
    /**
     * @brief Imposta la flag "obstacle_detected" (solo se il robot si stava muovendo)
     *
     * @param sensor_type Tipo di sensore. Può essere: INFRARED, ULTRASONIC
     * @param sensor_direction Direzione del sensore. Può essere: LEFT, FRONT, RIGHT
     */
    void obstacleDetectedWhileMoving(unsigned int sensor_type, unsigned int sensor_direction);
    /**
     * @brief Imposta la flag "obstacle_detected_before_moving" (solo se il robot partiva da fermo)
     *
     * @param sensor_type Tipo di sensore. Può essere: INFRARED, ULTRASONIC
     * @param sensor_direction Direzione del sensore. Può essere: LEFT, FRONT, RIGHT
     */
    void obstacleDetectedBeforeMoving(unsigned int sensor_type, unsigned int sensor_direction);
    // Resetta le variabili (stop da seriale)
    void externalStop();
    // Deprecato
    void uturnDelay();
    // Restituisce l'heading da 0 a 35999 (moltiplicato per 100)
    unsigned int getHeading360();
    // Abilita autorun
    void autoRun();
    // Variabile per sensore IR laterale
    void setLateralObstacle(bool state);

    unsigned int getTime();

    // metodo public perché viene usato dal core0
    /**
     * @brief Converte l'heading relativo a 90, 180, 270 gradi in heading relativo a 0 gradi. Esempio:
     * 315° -> 45° (ref: 270)
     *
     * @param heading Heading formato 360°
     * @param reference Heading relativo di partenza. Può essere: 90, 180, 270
     * @return unsigned int relativo a 0° formato 360°
     */
    unsigned int convertFromRelativeHDGToRef0(unsigned int heading, unsigned int reference);

    /**
     * @brief Converte l'heading relativo a 0 gradi in heading relativo a 90, 180, 270 gradi. Esempio:
     * 45° -> 315° (ref 270)
     *
     * @param heading Heading formato 360°
     * @param reference Heading relativo di target. Può essere: 90, 180, 270
     * @param add_subtract Se true: aggiunge l'heading al riferimento (es: 45 + 270); se false: sottrae l'heading al riferimento (es: (270+90)-45)
     * @return unsigned int Heading relativo a target formato 360°
     */
    unsigned int convertFromRef0ToRelativeHDG(unsigned int heading, unsigned int reference, bool add_subtract = true);

    // metodo public perché viene usato dal core0
    /**
     * @brief Inverte l'heading. Esempio: 0° -> 180°; 270° -> 90°
     *
     * @param heading Heading formato 360°
     * @return unsigned int Heading invertito formato 360°
     */
    unsigned int invertHDG(unsigned int heading);
    void motorStall();
    void mapBorderMode(bool on_off = false);
    void sdspeedtest(unsigned int bytes);
    void eraseSD(const char *path);
    // blocco iniziale: 1 - usare open_file = false quando si leggono tanti blocchi di fila con un file molto grande (> 50 kb)
    bool readBlock(unsigned int block, bool open_file = true);

    void scroll();

    // ferma le ruote del robot (non la logica) se si stava muovendo
    void pause(void (*fun_pause)(void));
    // mette in moto le ruote del robot (avanti) se si stava muovendo
    void resume();

    void mapLogging(bool);

    /**
     * @brief Converte un valore di heading in formato 360° in un valore di heading in formato 180°
     * 
     * @param heading Heading formato 360°
     * @param always_positive Indica se il risultato dev'essere sempre positivo (default: false)
     * @return int Heading in formato 180°
     */
    int convertHDGTo180(unsigned int heading, bool always_positive = false);

    /**
     * @brief Es: angle1 = 1, angle2 = 3 -> diff: -2; uguale all'inverso
    */
    int getRealAngleDiff(int angle1, int angle2);

    void fillXBytes(unsigned int bytes);

    /**
     * @param width Larghezza dell'area riempita
     * @param height Altezza dell'area riempita
     * @param minx Minimo valore x dell'area riempita
     * @param miny Minimo valore y dell'area riempita
     * @param fill_start_position Posizione in byte dalla quale inizia il riempimento
    */
    void removeDuplicatesFromMap(int width, int height, int minx, int miny, unsigned int fill_start_position);
    
    void writePoint(Point* p);

    // il file dev'essere già aperto
    void readPoint(unsigned int seek, Point* p);

    bool readPointBlock(unsigned int block);

    // i dati di mapinfo devono essere aggiornati
    unsigned int getPointPositionInSD(Point p, bool is_fill = false);
    unsigned int readPointID(Point p);

private:
    void resetMovementVars();
    // Restituisce HDG virtuale (0-35999)
    float getVirtualHDG(int heading);
    // Restituisce la direzione in cui il robot deve ruotare, con heading 180
    unsigned int getRotationDirection(int current_heading, int target_heading_180);

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
    void stop(bool clear_command_queues = true);

    /**
     * Aggiunge un certo valore ai vettori di distanza percorsa attuali e li restituisce senza modificarli effettivamente
     *
     * @param value_to_add valore da aggiungere ai vettori
     * @param heading heading formato 360°
     * @return std::tuple<int, int> - primo valore: vettore X; secondo valore: vettore Y
     */
    std::tuple<int, int> addToVectors(int value_to_add, unsigned int heading);

    /**
     * Scompone il vettore dato
     *
     * @param value_to_add lunghezza vettore
     * @param heading angolo vettore formato 360°
     * @return std::tuple<int, int> - primo valore: vettore X; secondo valore: vettore Y
     */
    std::tuple<int, int> getVectors(int vector_module, unsigned int vector_direction);

    int countDigits(int number)
    {
        return int(log10(number) + 1);
    }

    /**
     * Cerca in che blocco della mappa il robot si trova, poi assegna i valori in navmap.xvals - navmap.yvals - navmap.idvals
     * 
     * @return unsigned int - blocco
     */
    unsigned int getCurrentBlock();

    /**
     * Cerca in che blocco della mappa le coordinate specificate si trovano, poi assegna i valori in navmap.xvals - navmap.yvals - navmap.idvals
     * 
     * @return unsigned int - blocco
     */
    unsigned int getCurrentBlockFakeXY(int x, int y);

    /**
     * Restituisce la distanza dal punto più vicino.
     * Include chiamata a getCurrentBlock()
     * 
     * @param point_type tipo di punto (0, 1, 2) - 200 == qualsiasi
     * @return std::tuple<unsigned int, unsigned int, unsigned int> - distanza dal punto, id punto, index punto
     */
    std::tuple<unsigned int, unsigned int, unsigned int> getClosestPointDst(unsigned int point_type = 200);

    /**
     * Restituisce la distanza dal punto più vicino, specificando X e Y attuali (non usa i valori reali).
     * Bisogna leggere il blocco in cui si vuole cercare prima di chiamare questa funzione
     * 
     * @param point_type tipo di punto (0, 1, 2) - 200 == qualsiasi
     * @return std::tuple<unsigned int, unsigned int, unsigned int> - distanza dal punto, id punto, index punto, index blocco
     */
    std::tuple<unsigned int, unsigned int, unsigned int, unsigned int> getClosestPointDstFakeXY(int x, int y, unsigned int point_type = 200);

    unsigned int getSDLastReadablePosition(const char* file);
    // non chiama getSDLastReadablePosition()
    unsigned int getMaxBlock(const char* map, bool fill_empty_block_space = false);

    /**
     * Ottiene il punto con valore X o Y più alto
     * 
     * @param axis X o Y
     * @return std::tuple<unsigned int, unsigned int, int> - idx blocco, idx punto, valore punto (solo X o Y)
     */
    std::tuple<unsigned int, unsigned int, int> getTopPoint(const char* map, unsigned int axis);
    /**
     * Ottiene il punto con valore X o Y più basso
     *
     * @param axis X o Y
     * @return std::tuple<unsigned int, unsigned int, int> - idx blocco, idx punto, valore punto (solo X o Y)
     */
    std::tuple<unsigned int, unsigned int, int> getBottomPoint(const char* map, unsigned int axis = X);
    /**
     * Ottiene il punto con valore X o Y più alto nel blocco specificato
     *
     * @param block
     * @param axis X o Y
     * @return std::tuple<unsigned int, unsigned int, int> - idx blocco, idx punto, valore punto (solo X o Y)
     */
    std::tuple<unsigned int, unsigned int, int> getTopPointOf(unsigned int block, unsigned int axis);
    /**
     * Ottiene il punto con valore X o Y più basso nel blocco specificato
     *
     * @param block
     * @param axis X o Y
     * @return std::tuple<unsigned int, unsigned int, int> - idx blocco, idx punto, valore punto (solo X o Y)
     */
    std::tuple<unsigned int, unsigned int, int> getBottomPointOf(unsigned int block, unsigned int axis);

    std::tuple<int, int> getPointXY(unsigned int block, unsigned int point_idx);

    /**
     * @return unsigned int distanza dal punto
     */
    unsigned int getPointDst(Point* p);
    /**
     * @return unsigned int distanza dal punto
     * @param x in cm
     * @param y in cm
     */
    unsigned int getPointDst(int x, int y);

    /**
     * @brief Gira il robot verso il punto
     * @return unsigned int heading 360
     */
    unsigned int setHeadingToPoint(int x, int y);
    /**
     * @brief Gira il robot verso il punto
     * @return unsigned int heading 360
     */
    unsigned int setHeadingToPoint(Point *p);

    /**
     * @brief Va automaticamente verso il punto
     * @param precedence se true i comandi avranno la precedenza rispetto agli altri già presenti
    */
    void goToPoint(int x, int y, bool precedence = true);
    /**
     * @brief Va automaticamente verso il punto
     * @param precedence se true i comandi avranno la precedenza rispetto agli altri già presenti
    */
    void goToPoint(Point *p, bool precedence = true);

    /**
     * @brief Unisce i blocchi della mappa. Es: max blocco 1: 2578; min blocco 2: 2772 - unisce i due blocchi renendo il max1: 2675; max2: 2676
     * 
     * @return std::vector<unsigned int> - blocchi uniti, due a due (1, 2, 2, 3, 5, 6, 6, 7)
     */
    unsigned int joinMapBlocks();

    int convertCharVecToInt(std::vector<char> a);

    // apre il mapfile
    bool checkMapCompletion();

    /**
     * Inverte l'heading in formato 180. Es: 135° -> 45°; 45° -> 135°
     * Restituisce sempre valori positivi
     */
    unsigned int invert180HDG(int heading);

    unsigned int getMeanDiff(unsigned int block, unsigned int axis);

    // se higher_priority è true, inserisce comandi e dati (nell'ordine dato) prima dei contenuti della coda, spostando quindi questi ultimi dietro ai nuovi comandi.
    // es: lista: CMD1, CMD2, CMD3 - da inserire: CMD4, CMD5 - lista finale: CMD4, CMD5, CMD1, CMD2, CMD3
    void addToCommandQueue(unsigned int cmd, std::vector<int> *input_data, CommandQueue *queue, bool higher_priority = false);
    // se higher_priority è true, inserisce comandi e dati (nell'ordine dato) prima dei contenuti della coda, spostando quindi questi ultimi dietro ai nuovi comandi.
    // es: lista: CMD1, CMD2, CMD3 - da inserire: CMD4, CMD5 - lista finale: CMD4, CMD5, CMD1, CMD2, CMD3
    void addToCommandQueue(std::vector<unsigned int> *cmds, std::vector<int> *input_data, CommandQueue *queue, bool higher_priority = false);
    // usa commands e command_data (globali) come input. dopo la chiamata questi vettori vengono svuotati
    void addToCommandQueue(CommandQueue *queue, bool higher_priority = false);
    /**
     * @brief Elimina il numero specificato di comandi (e i relativi dati) da una coda.
     * L'eliminazione parte dall'inizio del vettore (vector::begin())
    */
    void clearQueue(CommandQueue *queue, unsigned int items_to_delete = 2147750);

    void displayQueue(CommandQueue *queue);

    /**
     * Ottiene il punto con valore X o Y più alto con X o Y specificato.
     *  
     * @param coordinate
     * @param axis X o Y
     * @return std::tuple<unsigned int, unsigned int, int, int> - idx blocco, idx punto, x punto, y punto
     */
    std::tuple<unsigned int, unsigned int, int, int> getTopPointWith(int coordinate, unsigned int given_coordinate_axis);

    /**
     * Ottiene il punto con valore X o Y più basso con X o Y specificato.
     *  
     * @param coordinate
     * @param axis X o Y
     * @return std::tuple<unsigned int, unsigned int, int, int> - idx blocco, idx punto, x punto, y punto
     */
    std::tuple<unsigned int, unsigned int, int, int> getBottomPointWith(int coordinate, unsigned int given_coordinate_axis);

    unsigned int getBlockContaining(int coordinate, unsigned int axis);

    /**
     * @brief Riempe la mappa con punti di ID 1 se sono fuori dai bordi e ID 0 se sono dentro
    */
    void processMap(int minx, int miny, int maxx, int maxy);

    // non apre il mapfile!
    MapPoint searchPoint(Point);

    int charArrToInt(uint8_t* arr, unsigned int len);

    void resetArray(uint8_t* arr, unsigned int len);

    Point getPointFromArr(uint8_t* arr, unsigned int len);

    void createProcessedMap(const char* raw_map_name, const char* map_name, unsigned int fill_start, int width, int height);

    void getMapSquare(unsigned int idx);

    void getMapInfo();
};

#endif