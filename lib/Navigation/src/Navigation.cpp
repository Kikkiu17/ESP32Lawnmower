#include <vector>
#include <map>
#include <tuple>
#include <SD.h>
#include <Navigation.h>
#include <algorithm>
#include <Sensors.h>
#include <Motors.h>
#include <Mux.h>
#include <Core.h>
#include <SETTINGS.h>
#include <stdio.h>
#include "map.h"
#include "psramvector.h"

#define MOSI 42
#define MISO 41
#define SCLK 40
#define CS 39

// variabili globali così non si frammenta la memoria ogni volta che si chiama NAV::readBlock, risparmiando anche tempo
// vengono usati vettori e buffer allocato dinamicamente perché si allocano sull'heap ed è meglio se sono molto grandi
// buffer non può essere un vettore perché SD::read accetta solo array
std::vector<int32_t> map_arr_data((POINTS_PER_BLOCK) * 3);  // ci sono 768 dati in 256 punti (256 * 3 (x, y, ID)) o 2816 byte di punti
uint8_t* block_buffer = new uint8_t[MAP_BLOCK_SIZE];
std::vector<char> temp_data(256);                           // convertCharArrToInt accetta solo array da 256 elementi
uint8_t raw_point[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
Point generic_point = Point();

std::vector<uint32_t> commands;
std::vector<int32_t> command_data;

#define FILEREAD "r"
#define FILEWRITE "w"
#define FILEAPPEND "r+" // non si può usare quello della libreria SD.h, perché non funziona per scritture binarie

SPIClass *spi = new SPIClass(FSPI);
File mapfile;

Mux NAVMux;
Sensors NAVSensors;
Motors NAVMotors;
Core NAVCore;
uint32_t t3 = 0;

/*
ID BLOCCHI MAPPA
0: accessibile
1: non accessbile
2: bordo mappa
3: ignora punto
*/

bool is_paused = false;
bool autorun = false;
uint32_t distance_target = 0;
float distance_traveled = 0.00;
uint32_t heading_target = 0;
bool obstacle_detected = false;
bool going_forward = false;
bool gone_forward = false;
bool going_backwards = false;
bool gone_backwards = false;
bool distance_target_reached = false;
bool robot_moving_x_y = false;
bool forward_wait_for_rotation = false;
uint32_t forward_wait_for_rotation_val = 0;
int32_t forward_wait_for_rotation_hdg = 0;
uint32_t timer = millis();
bool bordermode = false;
uint32_t bordermode_start_time = 0;


class CurrentCommand
{
public:
    int32_t id = 0;
    bool* isbusy = nullptr;
};


class TaskUtil
{
    private:
        uint32_t time0 = 0;
    public:
        uint32_t timeout = 1000;    // ms per lo yield
        TaskUtil() { time0 = millis(); }
        // default: 1000 ms
        void yieldWhenNeeded()
        {
            if (millis() - time0 > timeout)
            {
                vTaskDelay(1);
                time0 = millis();
            }
        }
};


class Chrono
{
    private:
        uint32_t start_time = 0;
    public:
        void start() { start_time = millis(); }
        uint32_t getTime() { return millis() - start_time; }
        void printTime() { Serial.printf("Done in %d ms", this->getTime()); }
};


class DirControl
{
    public:
        uint32_t direction = 0;
        int32_t target_heading = 0;
        bool is_pivoting = false;       // true se sta girando con una ruota ferma (pivot)
        int32_t last_turn_dir = LEFT;
        /**
         * @brief ottiene la direzione in cui deve girare ora, contrario di last_turn_dir. Inverte anche last_turn_dir
        */
        uint32_t getDirectionToTurn()
        {
            if (last_turn_dir == LEFT)
            {
                last_turn_dir = RIGHT;
                return RIGHT;
            }
            else
            {
                last_turn_dir = LEFT;
                return LEFT;
            }
        };
};


struct
{
    uint32_t start_heading = 0;
    uint32_t diff_to_rotate = 0;
    bool rotating = false;
    bool rotated = false;
    bool pivot = false;
} Rotation;


class Position
{
    public:
        float xvector = 0;          // cm (1.23)
        float yvector = 0;          // cm (1.23)
        float last_dst = 0;
        int32_t last_xcm = 0;       // cm (1)
        int32_t current_xcm = 0;    // cm (1)
        int32_t last_ycm = 0;       // cm (1)
        int32_t current_ycm = 0;    // cm (1)

        int32_t intxvector() { return (int32_t)(xvector * 100); };                      // mm (123)
        int32_t intyvector() { return (int32_t)(yvector * 100); };                      // mm (123)
        int32_t abs_intxvector() { return (uint32_t)(abs((int32_t)xvector * 100)); };   // mm (123)
        int32_t abs_intyvector() { return (uint32_t)(abs((int32_t)yvector * 100)); };   // mm (123)
};


struct
{
    bool active = false;
    bool *var_ptr;
    char dir = false;
    bool condition = false;
    bool done = false;
} RotateUntil;


struct NavMap
{
    int32_t current_block = 0;
    std::vector<std::vector<Point>> square;
    Point* points = new Point[POINTS_PER_BLOCK];
    int32_t arrX[POINTS_PER_BLOCK] = {};
    int32_t arrY[POINTS_PER_BLOCK] = {};
    uint32_t arrID[POINTS_PER_BLOCK] = {};
    int32_t curr_arrX[POINTS_PER_BLOCK] = {};
    int32_t curr_arrY[POINTS_PER_BLOCK] = {};
    uint32_t curr_arrID[POINTS_PER_BLOCK] = {};
    struct max
    {
        uint32_t block = 0;
        int32_t value = -1000000000;
        uint32_t idx = 0;
    };
    struct min
    {
        uint32_t block = 0;
        int32_t value = 1000000000;
        uint32_t idx = 0;
    };
    max* MaxX = new max();
    min* MinX = new min();
    max* MaxY = new max();
    min* MinY = new min();

    // @param reset_all se true, resetta anche la variabile del blocco corrente
    void reset(bool reset_all = false)
    {
        if (reset_all)
            current_block = -1;
        for (uint32_t i = 0; i < POINTS_PER_BLOCK; i++)
        {
            arrX[i] = 0;
            arrY[i] = 0;
            arrID[i] = 0;
            curr_arrX[i] = 0;
            curr_arrY[i] = 0;
            curr_arrID[i] = 0;
        }
        MaxX->block = 0;
        MaxX->value = 0;
        MaxX->idx = 0;
        MaxY->block = 0;
        MaxY->value = 0;
        MaxY->idx = 0;

        MinX->block = 0;
        MinX->value = 1000000000;
        MinX->idx = 0;
        MinY->block = 0;
        MinY->value = 1000000000;
        MinY->idx = 0;
    }
};


struct NavMapUtil
{
    uint32_t last_readable_pos = 0;
    uint32_t last_full_block = 0;
    uint32_t last_incomplete_block = 0;
    bool map_not_full = false;
    bool map_completion_checked = false;
    bool log_active = false;
};


Position position;
NavMap *nav = new NavMap;
NavMapUtil navutil;
NAV::CommandQueue *nav_ptr, navqueue;      // navqueue
NAV::CommandQueue *avoid_ptr, avoidqueue;  // avoidqueue
DirControl dir;
Chrono chrono;
Chrono chrono1;
MapCreationInfo mapinfo;
// contiene il comando attuale in esecuzione (NAVIGAZIONE)
CurrentCommand currcmd;

int32_t current_hdg = 0;
int32_t diff = 0;
uint32_t spd = 0;
int32_t to_rotate = 0;
int32_t target_hdg = 0;
float dst = 0;
float rad = 0;
uint32_t pos_xvector = 0;
uint32_t pos_yvector = 0;

void NAV::update()
{
    uint64_t start_time = micros();
    if (forward_wait_for_rotation && !Rotation.rotating) // non serve più con la coda -- eliminare
    {
        forward_wait_for_rotation = false;
        goForward(forward_wait_for_rotation_val, forward_wait_for_rotation_hdg);
    }

    if (nav_ptr->commands.size() > 0 && !nav_ptr->busy)
    {
        currcmd.id = nav_ptr->commands[0];
        if (nav_ptr->commands[0] == GOFORWARD || nav_ptr->commands[0] == GOBACKWARDS)
        {
            if (nav_ptr->commands[0] == GOFORWARD)
                goForward(nav_ptr->data[0], nav_ptr->data[1]);                      // non si possono usare valori default, sono obbligatori valori noti
            else
                goBackwards(nav_ptr->data[0], nav_ptr->data[1]);
            nav_ptr->commands.erase(nav_ptr->commands.begin());                     // elimina il comando in esecuzione
            nav_ptr->data.erase(nav_ptr->data.begin(), nav_ptr->data.begin() + 2);  // elimina i dati del comando
            nav_ptr->busy = true;                                                   // la coda è ora occupata
        }
        else if (nav_ptr->commands[0] == ROTATETO)
        {
            rotateToDeg(nav_ptr->data[0], nav_ptr->data[1]);
            nav_ptr->commands.erase(nav_ptr->commands.begin());
            nav_ptr->data.erase(nav_ptr->data.begin(), nav_ptr->data.begin() + 2);
            nav_ptr->busy = true;
        }
        else if (nav_ptr->commands[0] == ROTATEFOR)
        {
            rotateForDeg(nav_ptr->data[0]);
            nav_ptr->commands.erase(nav_ptr->commands.begin());
            nav_ptr->data.erase(nav_ptr->data.begin());
            nav_ptr->busy = true;
        }
        else if (nav_ptr->commands[0] == GOTOPOINT)
        {
            uint32_t data0 = nav_ptr->data[0];
            uint32_t data1 = nav_ptr->data[1];
            // gli erase DEVONO essere prima della chiamata alla funzione perché goToPoint aggiunge in coda 2 comandi
            nav_ptr->commands.erase(nav_ptr->commands.begin());
            nav_ptr->data.erase(nav_ptr->data.begin(), nav_ptr->data.begin() + 2);
            goToPoint(data0, data1);
            // gotopoint chiama subito SETHDGTOPOINT quindi non serve nav_ptr->busy = true
            // l'ggiunta di gotopoint alla coda è ridondante, ma è comodo per quando si vogliono aggiungere in coda diversi
            // comandi per il movimento
        }
        else if (nav_ptr->commands[0] == SETHDGTOPOINT)
        {
            setHeadingToPoint((uint32_t)nav_ptr->data[0], (uint32_t)nav_ptr->data[1]);
            nav_ptr->commands.erase(nav_ptr->commands.begin());
            nav_ptr->data.erase(nav_ptr->data.begin(), nav_ptr->data.begin() + 2);
            nav_ptr->busy = true;
        }
    }

    if (avoid_ptr->commands.size() > 0 && !avoid_ptr->busy)
    {
        if (avoid_ptr->commands[0] == GOFORWARD || avoid_ptr->commands[0] == GOBACKWARDS)
        {
            if (avoid_ptr->commands[0] == GOFORWARD)
                goForward(avoid_ptr->data[0], avoid_ptr->data[1]); // non si possono usare valori default, sono obbligatori valori noti
            else
                goBackwards(avoid_ptr->data[0], avoid_ptr->data[1]);
            avoid_ptr->commands.erase(avoid_ptr->commands.begin()); // elimina il comando in esecuzione
            avoid_ptr->data.erase(avoid_ptr->data.begin(), avoid_ptr->data.begin() + 2);
            avoid_ptr->busy = true;
        }
        else if (avoid_ptr->commands[0] == ROTATETO)
        {
            rotateToDeg(avoid_ptr->data[0], avoid_ptr->data[1]);
            avoid_ptr->commands.erase(avoid_ptr->commands.begin());
            avoid_ptr->data.erase(avoid_ptr->data.begin(), avoid_ptr->data.begin() + 2);
            avoid_ptr->busy = true;
        }
        else if (avoid_ptr->commands[0] == ROTATEFOR)
        {
            rotateForDeg(avoid_ptr->data[0]);
            avoid_ptr->commands.erase(avoid_ptr->commands.begin());
            avoid_ptr->data.erase(avoid_ptr->data.begin());
            avoid_ptr->busy = true;
        }
        else if (avoid_ptr->commands[0] == ROTATEFORPIVOT)
        {
            rotateForPivot(avoid_ptr->data[0]);
            avoid_ptr->commands.erase(avoid_ptr->commands.begin());
            avoid_ptr->data.erase(avoid_ptr->data.begin());
            avoid_ptr->busy = true;
        }
    }

    if (going_forward)
    {
        distance_traveled = NAVSensors.getTraveledDistance();

        if (abs(distance_traveled) >= distance_target && distance_traveled != 0 && distance_target != 0)
        {
            stop(false);
            nav_ptr->busy = false;
            avoid_ptr->busy = false;
            going_forward = false;
            gone_forward = true;
            distance_target_reached = true;
            distance_target = 0;
        }
    }

    if (going_backwards)
    {
        distance_traveled = NAVSensors.getTraveledDistance();

        if (abs(distance_traveled) >= distance_target && distance_traveled != 0 && distance_target != 0)
        {
            stop(false);
            nav_ptr->busy = false;
            avoid_ptr->busy = false;
            going_backwards = false;
            gone_backwards = true;
            distance_target_reached = true;
            distance_target = 0;
        }
    }

    if (obstacle_detected)
    {
        NAVCore.println(F("(Navigation.cpp) OBSTACLE DETECTED"));
        obstacle_detected = false;
        going_forward = false;

        stop();
        current_hdg = getHeading360();
        if (USE_SD)
        {
            if (!bordermode && LOG_OBSTACLES_TO_MAP && navutil.log_active)
            {
                int32_t xvec, yvec;
                std::tie(xvec, yvec) = addToVectors(5, current_hdg);
                mapfile = SD.open("/map.bin", "r+");
                mapfile.print(xvec);
                mapfile.print(F(","));
                mapfile.print(yvec);
                mapfile.print(F(","));
                mapfile.print(1);
                mapfile.print(F(","));
                navutil.last_readable_pos += mapfile.position();
                mapfile.close();
            }
        }

        if (autorun)
        {
            // queste azioni vengono fatte solo se il robot è in modalità AUTO
            // altrimenti, sono attivi solo i sensori (il robot si ferma a un ostacolo)
            
            if (!dir.is_pivoting)
            {
                clearQueue(avoid_ptr);  // se stava già andando avanti, elimina tutti i comandi
                int32_t hdg_to_maintain = NAVSensors.getHeading();
                std::vector<uint32_t> commands = {GOBACKWARDS, ROTATEFORPIVOT, GOFORWARD};
                int32_t heading_displacement = 0;
                heading_displacement = (dir.getDirectionToTurn() == LEFT) ? -180 : 180;
                // gobackwards, gobackwards, rotateforpivot, goforward, goforward
                std::vector<int32_t> data = {10, hdg_to_maintain, heading_displacement, MAX_CM, AUTO};
                addToCommandQueue(&commands, &data, avoid_ptr);
            }
            else
            {
                // stava già ruotando per arrivare all'heading target (rotateForPivot)
                // robot già fermo
                // usa dir.last_turn_dir perché deve continuare a ruotare nella stessa direzione in cui lo stava già facendo
                std::vector<uint32_t> commands = {ROTATETO, GOFORWARD};
                // rotateto, rotateto, goforward, goforward
                std::vector<int32_t> data = {dir.target_heading, dir.last_turn_dir, MAX_CM, AUTO};
                addToCommandQueue(&commands, &data, avoid_ptr);
            }
        }
        else
            externalStop();
    }

    if (RotateUntil.active)
    {
        if (RotateUntil.dir == 'r')
            NAVMotors.right();
        else
            NAVMotors.left();

        if (*RotateUntil.var_ptr == RotateUntil.condition)
        {
            stop();
            RotateUntil.done = true;
            RotateUntil.active = false;
        }
    }

    if (Rotation.rotating && ENABLE_ROTATION_SENSING)
    {
        current_hdg = getHeading360();
        if (current_hdg > heading_target - 75 && current_hdg < heading_target + 75)
        {
            stop(false);
            nav_ptr->busy = false;
            avoid_ptr->busy = false;
            NAVSensors.resetMovementVars();
            Rotation.rotating = false;
            Rotation.rotated = true;
            dir.is_pivoting = false;
            timer = millis();
        }
        // disabilitato perché sull'erba la velocità diminuisce, quindi se usasse il rotation loop, i valori di velocità minimi
        // sarebbero troppo bassi e non si muoverebbe
        else if (ENABLE_ROTATION_LOOP)
        {
            current_hdg = convertHDGTo180(current_hdg); // c'è già prima current_hdg = getHeading360()
            target_hdg = convertHDGTo180(heading_target);
            diff = abs(getRealAngleDiff(current_hdg, target_hdg)) / 100;
            to_rotate = Rotation.diff_to_rotate;

            // https://www.desmos.com/calculator/3e9myz8cc7
            // sotto: funzione 5 (derivata della funzione 1)
            if (diff < 45)
            {
                spd = MOT_MIN_VAL + 45 * (((MOT_MAX_VAL - MOT_MIN_VAL) * (-2 * to_rotate * pow(diff, 2) + 2 * pow(to_rotate, 2) * diff)) /
                pow(2 * pow(diff, 2) - 2 * to_rotate * diff + pow(to_rotate, 2), 2));
                /*if (dir.is_pivoting)
                {
                    if (dir.direction == LEFT)
                        NAVMotors.setSpeed(spd, RIGHT);
                    else
                        NAVMotors.setSpeed(spd, LEFT);
                }
                else*/
                    NAVMotors.setSpeed(spd, BOTH);
            }
            else
                NAVMotors.setSpeed(MOT_MAX_VAL, BOTH);
        }
    }

    if (robot_moving_x_y || dir.is_pivoting)
    {
        /**
         * Heading in modailtà 360°
         * Il vettore X è positivo verso 90°
         * Il vettore Y è positivo verso 0°
         * |         0°         |
         * |     /       \      |
         * |  270°        90°   |
         * |     \       /      |
         * |        180°        |
         */

        dst = NAVSensors.getLastTraveledDistance();
        if (position.last_dst != dst && (dst > 0.03 || dst < 0.03))
        {
            uint32_t hdg_360 = getHeading360();
            position.last_dst = dst;

            // i vettori si alternano sin e cos perché il riferimento dell'heading cambia in base al quadrante

            if (hdg_360 >= 0 && hdg_360 <= 9000) // heading positivo per X e Y -- alto destra -- riferimento 0°
            {
                //if (going_backwards)
                //    hdg_360 = invertHDG(hdg_360);
                rad = radians(hdg_360 / 100);
                position.xvector += dst * sin(rad); // positivo per X
                position.yvector += dst * cos(rad); // positivo per Y
            }
            else if (hdg_360 > 9000 && hdg_360 <= 18000) // heading positivo per X, negativo per Y -- basso destra -- riferimento 90°
            {
                //if (going_backwards)
                //    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertFromRelativeHDGToRef0(hdg_360, 90);
                rad = radians(hdg_360 / 100);
                position.xvector += dst * cos(rad); // positivo per X
                position.yvector -= dst * sin(rad); // negativo per Y
            }
            else if (hdg_360 > 18000 && hdg_360 <= 27000) // heading negativo per X e Y -- basso sinistra -- riferimento 180°
            {
                //if (going_backwards)
                //    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertFromRelativeHDGToRef0(hdg_360, 180);
                rad = radians(hdg_360 / 100);
                position.xvector -= dst * sin(rad); // negativo per X
                position.yvector -= dst * cos(rad); // negativo per Y
            }
            else // heading positivo per Y, negativo per X -- alto sinistra -- riferimento 270°
            {
                //if (going_backwards)
                //    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertFromRelativeHDGToRef0(hdg_360, 270);
                rad = radians(hdg_360 / 100);
                position.xvector -= dst * cos(rad); // negativo per X
                position.yvector += dst * sin(rad); // positivo per Y
            }

            position.current_xcm = (int32_t)position.xvector;
            position.current_ycm = (int32_t)position.yvector;
            if (USE_SD)
            {
                if (position.current_xcm != position.last_xcm || position.current_ycm != position.last_ycm)
                {
                    position.last_xcm = position.current_xcm;
                    position.last_ycm = position.current_ycm;

                    if ((LOG_MAP || bordermode || navutil.map_not_full) && navutil.log_active)
                    {
                        generic_point.x = position.current_xcm;
                        generic_point.y = position.current_ycm;

                        if (bordermode)
                        {
                            generic_point.id = 2;
                            mapfile.write((uint8_t*)&generic_point, MAP_POINT_SIZE);
                            //writePoint(&generic_point);
                            if (position.xvector < 5 && position.xvector > -5 && position.yvector < 5 && position.yvector > -5 && millis() - bordermode_start_time > 20000)
                            {
                                bordermode = false;
                                stop();
                                NAVCore.println(F("Bordermode COMPLETED"));
                                mapfile.close();
                                ESP.restart();
                            }
                        }
                        else
                        {
                            generic_point.id = 0;
                            mapfile.write((uint8_t*)&generic_point, MAP_POINT_SIZE);
                            //writePoint(&generic_point);
                        }
                        navutil.last_readable_pos += mapfile.position();

                        NAVCore.println("X", position.current_xcm);
                        NAVCore.println("Y", position.current_ycm);
                        mapfile.flush();
                    }

                }

/*
                if (!bordermode)
                {
                    if (position.abs_intxvector() > 200 && position.abs_intyvector() > 200) // 200 / 100
                    {
                        uint32_t pt_dst, pt_id, pt_idx;
                        std::tie(pt_dst, pt_id, pt_idx) = getClosestPointDst();

                        if (pt_id == BORDER)
                        {
                            if (pt_dst < MAP_POINT_PROXIMITY_DST)
                            {
                                stop();
                                obstacleDetectedWhileMoving(ULTRASONIC, FRONT);
                                NAVCore.println("BORDER CLOSER THAN", MAP_POINT_PROXIMITY_DST);
                                NAVCore.println("CLOSEST POINT X", nav->curr_arrX[pt_idx]);
                                NAVCore.println("CLOSEST POINT Y", nav->curr_arrY[pt_idx]);
                                NAVCore.println("DST", pt_dst);
                            }
                        }
                    }
                }
*/
            }
        }
    }

    t3 = micros() - start_time;
}

uint32_t NAV::getTime()
{
    return t3;
}

void NAV::goForward(uint32_t cm, int32_t hdg_to_maintain)
{
    if (!is_paused)
    {
        resetMovementVars();
        dir.direction = FWD;
        nav_ptr->busy = true;
        if (Rotation.rotating)
        {
            forward_wait_for_rotation_val = cm;
            forward_wait_for_rotation_hdg = hdg_to_maintain;
            forward_wait_for_rotation = true;
            return;
        }
        NAVSensors.resetTraveledDistance();
        distance_target = cm;
        going_forward = true;
        robot_moving_x_y = true;
    }
    NAVMotors.forward();
}

void NAV::goBackwards(uint32_t cm, int32_t hdg_to_maintain)
{
    if (!is_paused)
    {
        resetMovementVars();
        dir.direction = BCK;
        nav_ptr->busy = true;
        NAVSensors.resetTraveledDistance();
        distance_target = cm;
        going_backwards = true;
        robot_moving_x_y = true;
    }
    NAVMotors.backwards();
}

void NAV::rotateForDeg(int32_t degs)
{
    /*
            0°
        /       \
    270°           90°
        \       /
           180°
    */

    // dev'essere all'inizio per far usare lo yaw dell'MPU da getHeading in Sensors
    if (degs < 0)
    {
        NAVMotors.left();
        dir.direction = LEFT;
    }
    else
    {
        NAVMotors.right();
        dir.direction = RIGHT;
    }

    nav_ptr->busy = true;
    resetMovementVars();
    Rotation.diff_to_rotate = abs(degs);
    int32_t heading_to_reach = degs * 100;
    if (heading_to_reach < 0)
        heading_to_reach = heading_to_reach + 36000; // 36000
    Rotation.start_heading = getHeading360();
    int32_t tgt = (int32_t(getHeading360()) + (degs * 100)) % 36000;
    if (tgt < 0)
        tgt += 36000;
    heading_target = tgt;
    NAVCore.println("cur", getHeading360());
    NAVCore.println("target", tgt);
    Rotation.rotating = true;

}

void NAV::rotateToDeg(uint32_t heading, uint32_t direction)
{
    // dev'essere all'inizio per far usare lo yaw dell'MPU da getHeading in Sensors
    if (direction == LEFT)
    {
        dir.direction = LEFT;
        NAVMotors.left();
    }
    else
    {
        dir.direction = RIGHT;
        NAVMotors.right();
    }

    nav_ptr->busy = true;
    resetMovementVars();
    Rotation.rotating = true;
    heading_target = heading;
    Rotation.start_heading = getHeading360();
    target_hdg = convertHDGTo180(heading_target);
    uint32_t current_hdg = Rotation.start_heading;
    if (current_hdg > 0 && target_hdg > 0) // entrambi a destra
        diff = current_hdg - target_hdg;
    else if (current_hdg < 0 && target_hdg < 0) // entrambi a sinistra
        diff = current_hdg - target_hdg;
    else
    {
        if (current_hdg < 0)
            current_hdg = invert180HDG(current_hdg);
        else if (target_hdg < 0)
            target_hdg = invert180HDG(target_hdg);
        diff = current_hdg - target_hdg;
    }
    Rotation.diff_to_rotate = abs(diff) / 100;

    NAVCore.println("Rotating to", target_hdg);
}

void NAV::obstacleDetectedWhileMoving(uint32_t sensor_type, uint32_t sensor_direction)
{
    obstacle_detected = true;
}

void NAV::externalStop()
{
    NAVCore.print("x", position.current_xcm);
    NAVCore.println("y", position.current_ycm);
    stop(true);
    resetMovementVars();
    dir = {};
    autorun = false;
    NAVSensors.resetMovementVars();
    robot_moving_x_y = false;
}

uint32_t NAV::getHeading360()
{
    int32_t hdg = NAVSensors.getHeading();
    if (hdg < 0)
        hdg += 36000;

    return hdg;
}

void NAV::autoRun()
{
    autorun = true;
    goForward();
}

void NAV::resetMovementVars()
{
    going_forward = false;
    gone_forward = false;
    going_backwards = false;
    gone_backwards = false;
    Rotation.rotating = false;
    Rotation.rotated = false;
    obstacle_detected = false;
    distance_target_reached = false;
    distance_target = 0;
    distance_traveled = 0;
}

float NAV::getVirtualHDG(int32_t heading)
{
    int32_t heading_to_reach = heading * 100;
    if (heading_to_reach < 0)
        heading_to_reach += 36000; // 36000 + 900 (900 = errore)
    //else
        //heading_to_reach -= 650;
    int32_t start_heading_loc = NAVSensors.getHeading();
    if (start_heading_loc < 0)
        start_heading_loc += 36000;
    int32_t over_start_heading = start_heading_loc + heading_to_reach;
    if (over_start_heading - 36000 < 0)
        heading_target = over_start_heading;
    else
        heading_target = over_start_heading - 36000;

    return heading_target;
}

uint32_t NAV::getRotationDirection(int32_t current_hdg, int32_t target_heading)
{
    diff = getRealAngleDiff(current_hdg, target_heading);
    if (diff < 0) return RIGHT;
    else return LEFT;
}

void NAV::rotateUntil(char dir, bool *var, bool condition)
{
    RotateUntil.condition = condition;
    RotateUntil.dir = dir;
    RotateUntil.var_ptr = var;
    RotateUntil.active = true;
}

void NAV::obstacleDetectedBeforeMoving(uint32_t sensor_type, uint32_t sensor_direction)
{
    stop();
}

int32_t NAV::convertHDGTo180(uint32_t heading, bool always_positive)
{
    if (heading > 18000)
        if (always_positive && heading - 36000 < 0)
            return (heading - 36000) * -1;
        else
            return heading - 36000;
    else
        return heading;
}

void NAV::stop(bool clear_command_queues)
{
    nav_ptr->busy = false;
    avoid_ptr->busy = false;
    if (clear_command_queues)
    {
        clearQueue(nav_ptr);
        clearQueue(avoid_ptr);
    }
    NAVMotors.stop();
    robot_moving_x_y = false;
    going_forward = false;
    going_backwards = false;
    Rotation.rotating = false;
    distance_traveled = 0;
}

uint32_t NAV::convertFromRelativeHDGToRef0(uint32_t hdg, uint32_t reference)
{
    uint32_t return_hdg = 36000 - reference * 100 + hdg;
    if (return_hdg - 36000 > 0)
        return return_hdg - 36000;
    else
        return return_hdg;
}

uint32_t NAV::invertHDG(uint32_t hdg)
{
    uint32_t return_hdg = hdg + 18000;
    if (return_hdg - 36000 > 0)
        return return_hdg - 36000;
    else
        return return_hdg;
}

void NAV::eraseSD(const char *path)
{
    if (!USE_SD) return;
    mapfile = SD.open(path, "w+");
    mapfile.print(F(""));
    mapfile.close();
}

bool NAV::readBlock(uint32_t block, bool open)
{
    if (!USE_SD || nav->current_block == block) return false;

    uint32_t start_position = MAP_BLOCK_SIZE * (block - 1)/* - 1*/;

    if (start_position + MAP_BLOCK_SIZE > navutil.last_readable_pos)
    {
        log_e("SD Target read position greater than last readable position (%d > %d)", start_position + MAP_BLOCK_SIZE, navutil.last_readable_pos);
        return false;
    }

    nav->reset();
    nav->current_block = block;
    uint32_t pos = 0;
    uint32_t data_type = 0;
    uint32_t read_data = 0;
    // reset degli array
    for (uint32_t i = 0; i < (POINTS_PER_BLOCK) * 3; i++)
        map_arr_data[i] = 0;
    uint32_t temp_idx = 0;
    uint32_t data_idx = 0;

    NAVCore.println("Reading", block);
    if (open)
        mapfile = SD.open("/map.bin");
    mapfile.seek(start_position);
    mapfile.read(block_buffer, MAP_BLOCK_SIZE);
    if (open)
        mapfile.close();

    uint32_t point_idx = 0;

    for (int32_t i = 0; i < MAP_BLOCK_SIZE; i++)
    {
        if (data_type < 2)                                          // coordinate
        {
            if (block_buffer[i] == 45)                                    // ASCII trattino
                temp_data[temp_idx] = '-';
            else
                temp_data[temp_idx] = block_buffer[i];
            if (temp_idx == 2 - 1)
            {
                map_arr_data[data_idx] = convertCharVecToInt(temp_data);

                if (data_type == 0)
                {
                    if (map_arr_data[data_idx] > nav->MaxX->value)
                    {
                        nav->MaxX->value = map_arr_data[data_idx];
                        nav->MaxX->block = block;
                        nav->MaxX->idx = read_data;
                    }
                    if (map_arr_data[data_idx] < nav->MinX->value)
                    {
                        nav->MinX->value = map_arr_data[data_idx];
                        nav->MinX->block = block;
                        nav->MinX->idx = read_data;
                    }
                    nav->arrX[read_data] = map_arr_data[data_idx];
                }
                else if (data_type == 1)
                {
                    if (map_arr_data[data_idx] > nav->MaxY->value)
                    {
                        nav->MaxY->value = map_arr_data[data_idx];
                        nav->MaxY->block = block;
                        nav->MaxY->idx = read_data;
                    }
                    if (map_arr_data[data_idx] < nav->MinY->value)
                    {
                        nav->MinY->value = map_arr_data[data_idx];
                        nav->MinY->block = block;
                        nav->MinY->idx = read_data;
                    }
                    nav->arrY[read_data] = map_arr_data[data_idx];
                }

                data_idx++;
                data_type++;
                temp_idx = 0;
                continue;
            }
            temp_idx++;
        }
        if (data_type == 2)                                         // ID punto
        {
            map_arr_data[data_idx] = block_buffer[i] - '0';
            nav->arrID[read_data] = map_arr_data[data_idx];
            data_idx++;
            data_type = 0;
        }
    }

    return true;
}

uint32_t NAV::getCurrentBlock()
{
    bool xblock_found = false;
    bool yblock_found = false;
    uint32_t xblock_idx = 0;
    uint32_t yblock_idx = 0;
    uint32_t counter = 1;
    uint32_t last_block = 0;
    uint32_t read_blocks = 0;
    int32_t int_xvector = position.intxvector();
    int32_t int_yvector = position.intyvector();

    // se la posizione del robot è già compresa nei valori del blocco attuali
    if (nav->MinX->value < int_xvector && nav->MaxX->value > int_xvector && nav->MinY->value < int_yvector &&nav->MaxY->value > int_yvector)
    {
        for (int32_t i = 0; i < POINTS_PER_BLOCK; i++)
            nav->curr_arrX[i] = nav->arrX[i];
        for (int32_t i = 0; i < POINTS_PER_BLOCK; i++)
            nav->curr_arrY[i] = nav->arrY[i];
        for (int32_t i = 0; i < POINTS_PER_BLOCK; i++)
            nav->curr_arrID[i] = nav->arrID[i];
        return nav->current_block;
    }

    mapfile = SD.open("/map.bin");
    while (!xblock_found || !yblock_found)
    {
        if (counter != last_block)
        {
            //if (read_blocks == 2)
                //pause();
            last_block = counter;
            readBlock(counter, false);
            read_blocks++;
        }
        else
        {
            if (read_blocks >= 2)
                resume();
            return nav->current_block;
        }

        if (!xblock_found)
        {
            if (int_xvector > nav->MinX->value - 300)
            {
                if (int_xvector < nav->MaxX->value + 300)
                {
                    xblock_idx = nav->current_block;
                    for (int32_t i = 0; i < POINTS_PER_BLOCK; i++)
                        nav->curr_arrX[i] = nav->arrX[i];
                    xblock_found = true;
                }
                else
                {
                    if (counter < navutil.last_full_block)
                        counter++;
                    continue;
                }
            }
            else
            {
                if (counter > 1)
                    counter--;
                continue;
            }
        }
        else if (!yblock_found)
        {
            if (int_yvector > nav->MinY->value - 300)
            {
                if (int_yvector < nav->MaxY->value + 300)
                {
                    yblock_idx = nav->current_block;
                    for (int32_t i = 0; i < POINTS_PER_BLOCK; i++)
                        nav->curr_arrY[i] = nav->arrY[i];
                    yblock_found = true;
                }
                else
                {
                    if (counter < navutil.last_full_block)
                        counter++;
                    continue;
                }
            }
            else
            {
                if (counter > 1)
                    counter--;
                continue;
            }
        }
    }
    mapfile.close();

    if (read_blocks >= 2)
        resume();

    if (xblock_idx != yblock_idx)
        return NOT_FOUND;
    else
    {
        for (int32_t i = 0; i < POINTS_PER_BLOCK; i++)
            nav->curr_arrID[i] = nav->arrID[i];
    }

    return nav->current_block;
}

uint32_t NAV::getCurrentBlockFakeXY(int32_t x, int32_t y)
{
    bool xblock_found = false;
    bool yblock_found = false;
    uint32_t xblock_idx = 0;
    uint32_t yblock_idx = 0;
    uint32_t counter = 1;
    uint32_t last_block = 0;
    uint32_t read_blocks = 0;
    int32_t int_xvector = x;
    int32_t int_yvector = y;

    // se la posizione del robot è già compresa nei valori del blocco attuali
    if (nav->MinX->value < int_xvector && nav->MaxX->value > int_xvector && nav->MinY->value < int_yvector && nav->MaxY->value > int_yvector)
    {
        for (int32_t i = 0; i < 256; i++)
            nav->curr_arrX[i] = nav->arrX[i];
        for (int32_t i = 0; i < 256; i++)
            nav->curr_arrY[i] = nav->arrY[i];
        for (int32_t i = 0; i < 256; i++)
            nav->curr_arrID[i] = nav->arrID[i];
        return nav->current_block;
    }

    mapfile = SD.open("/map.bin");
    while (!xblock_found || !yblock_found)
    {
        if (counter != last_block)
        {
            //if (read_blocks == 2)
                //pause();
            last_block = counter;
            readBlock(counter, false);
            read_blocks++;
        }
        else
        {
            if (read_blocks >= 2)
                resume();
            return nav->current_block;
        }

        if (!xblock_found)
        {
            if (int_xvector > nav->MinX->value - 300)
            {
                if (int_xvector < nav->MaxX->value + 300)
                {
                    xblock_idx = nav->current_block;
                    for (int32_t i = 0; i < 256; i++)
                        nav->curr_arrX[i] = nav->arrX[i];
                    xblock_found = true;
                }
                else
                {
                    if (counter < navutil.last_full_block)
                        counter++;
                    continue;
                }
            }
            else
            {
                if (counter > 1)
                    counter--;
                continue;
            }
        }
        else if (!yblock_found)
        {
            if (int_yvector > nav->MinY->value - 300)
            {
                if (int_yvector < nav->MaxY->value + 300)
                {
                    yblock_idx = nav->current_block;
                    for (int32_t i = 0; i < 256; i++)
                        nav->curr_arrY[i] = nav->arrY[i];
                    yblock_found = true;
                }
                else
                {
                    if (counter < navutil.last_full_block)
                        counter++;
                    continue;
                }
            }
            else
            {
                if (counter > 1)
                    counter--;
                continue;
            }
        }
    }
    mapfile.close();

    if (read_blocks >= 2)
        resume();

    if (xblock_idx != yblock_idx)
        return NOT_FOUND;
    else
    {
        for (int32_t i = 0; i < 256; i++)
            nav->curr_arrID[i] = nav->arrID[i];
    }

    return nav->current_block;
}

std::tuple<uint32_t, uint32_t, uint32_t> NAV::getClosestPointDst(uint32_t point_type)
{
    uint32_t current_block = getCurrentBlock();

    uint32_t pos_xvector = position.abs_intxvector();
    uint32_t pos_yvector = position.abs_intyvector();

    uint32_t dst_to_closest_point = 1000000;
    uint32_t closest_point_idx = 0;
    uint32_t second_closest_point_idx = 0;
    for (int32_t i = 0; i < 256; i++)
    {
        if (nav->curr_arrID[i] != 3)
        {
            // 200 == qualsiasi punto
            if ((nav->curr_arrID[i] == point_type && point_type != 200) || point_type == 200)
            {
                int32_t xdiff = 0;
                int32_t xpoint = nav->curr_arrX[i];
                xpoint *= (xpoint < 0) ? -1 : 1;
                xdiff = pos_xvector - xpoint;
                xdiff *= (xdiff < 0) ? -1 : 1;

                int32_t ydiff = 0;
                int32_t ypoint = nav->curr_arrY[i];
                ypoint *= (ypoint < 0) ? -1 : 1;
                ydiff = pos_yvector - ypoint;
                ydiff *= (ydiff < 0) ? -1 : 1;

                uint32_t real_diff = sqrt(pow(xdiff, 2) + pow(ydiff, 2)); // pitagora
                
                if (real_diff < dst_to_closest_point)
                {
                    dst_to_closest_point = real_diff;
                    closest_point_idx = i;
                }
            }
        }
    }

    return std::make_tuple(dst_to_closest_point, nav->curr_arrID[closest_point_idx], closest_point_idx);
}

std::tuple<uint32_t, uint32_t, uint32_t, uint32_t> NAV::getClosestPointDstFakeXY(int32_t x, int32_t y, uint32_t point_type)
{
    uint32_t pos_xvector = (uint32_t)abs(x);
    uint32_t pos_yvector = (uint32_t)abs(y);

    uint32_t dst_to_closest_point = 1000000;
    uint32_t closest_point_idx = 0;
    uint32_t second_closest_point_idx = 0;
    for (int32_t i = 0; i < 256; i++)
    {
        if (nav->arrID[i] != 3)
        {
            // 200 == qualsiasi punto
            if ((nav->arrID[i] == point_type && point_type != 200) || point_type == 200)
            {
                int32_t xdiff = 0;
                int32_t xpoint = nav->arrX[i];
                xpoint *= (xpoint < 0) ? -1 : 1;
                xdiff = pos_xvector - xpoint;
                xdiff *= (xdiff < 0) ? -1 : 1;

                int32_t ydiff = 0;
                int32_t ypoint = nav->arrY[i];
                ypoint *= (ypoint < 0) ? -1 : 1;
                ydiff = pos_yvector - ypoint;
                ydiff *= (ydiff < 0) ? -1 : 1;

                uint32_t real_diff = sqrt(pow(xdiff, 2) + pow(ydiff, 2)); // pitagora
                
                if (real_diff < dst_to_closest_point)
                {
                    dst_to_closest_point = real_diff;
                    closest_point_idx = i;
                }
            }
        }
    }

    return std::make_tuple(dst_to_closest_point, nav->arrID[closest_point_idx], closest_point_idx, 1);
}

void NAV::mapBorderMode(bool on_off)
{
    if (on_off)
        mapfile = SD.open("/map.bin", "r+");
    bordermode = on_off;
    bordermode_start_time = millis();
}

uint32_t NAV::getSDLastReadablePosition(const char* file)
{
    if (!USE_SD) return 0;
    uint32_t pos = 0;
    int32_t data = 0;
    mapfile = SD.open(file, "r");
    pos = mapfile.size();
    mapfile.close();
    return pos;
}

void NAV::sdspeedtest(uint32_t bytes)
{
    if (!USE_SD) return;
    uint8_t* buffer = new uint8_t[72900];
    uint32_t start_time = millis();
    mapfile = SD.open("/map.bin");
    //for (int32_t i = 0; i < bytes; i++)
    //    mapfile.parseInt();
    mapfile.close();
    Serial.print(F("TIME (72900  bytes): "));
    Serial.println(millis() - start_time);

    delete[] buffer;
}

std::tuple<int32_t, int32_t> NAV::addToVectors(int32_t val, uint32_t hdg)
{
    int32_t xvec = 0;
    int32_t yvec = 0;
    if (hdg > 0 && hdg < 9000) // heading positivo per X e Y -- alto destra -- riferimento 0°
    {
        if (going_backwards)
            hdg = invertHDG(hdg);
        float rad = radians(hdg / 100);
        xvec += val * sin(rad); // positivo per X
        yvec += val * cos(rad); // positivo per Y
    }
    else if (hdg > 9000 && hdg < 18000) // heading positivo per X, negativo per Y -- basso destra -- riferimento 90°
    {
        if (going_backwards)
            hdg = invertHDG(hdg);
        hdg = convertFromRelativeHDGToRef0(hdg, 90);
        float rad = radians(hdg / 100);
        xvec += val * cos(rad); // positivo per X
        yvec -= val * sin(rad); // negativo per Y
    }
    else if (hdg > 18000 && hdg < 27000) // heading negativo per X e Y -- basso sinistra -- riferimento 180°
    {
        if (going_backwards)
            hdg = invertHDG(hdg);
        hdg = convertFromRelativeHDGToRef0(hdg, 180);
        float rad = radians(hdg / 100);
        xvec -= val * sin(rad); // negativo per X
        yvec -= val * cos(rad); // negativo per Y
    }
    else // heading positivo per Y, negativo per X -- alto sinistra -- riferimento 270°
    {
        if (going_backwards)
            hdg = invertHDG(hdg);
        hdg = convertFromRelativeHDGToRef0(hdg, 270);
        float rad = radians(hdg / 100);
        xvec -= val * cos(rad); // negativo per X
        yvec += val * sin(rad); // positivo per Y
    }

    return std::make_tuple((int32_t)((xvec + position.xvector) * 100), (int32_t)((yvec + position.yvector) * 100));
}

std::tuple<int32_t, int32_t> NAV::getVectors(int32_t val, uint32_t hdg)
{
    int32_t xvec = 0;
    int32_t yvec = 0;
    if (hdg > 0 && hdg < 9000) // heading positivo per X e Y -- alto destra -- riferimento 0°
    {
        if (going_backwards)
            hdg = invertHDG(hdg);
        float rad = radians(hdg / 100);
        xvec += val * sin(rad); // positivo per X
        yvec += val * cos(rad); // positivo per Y
    }
    else if (hdg > 9000 && hdg < 18000) // heading positivo per X, negativo per Y -- basso destra -- riferimento 90°
    {
        if (going_backwards)
            hdg = invertHDG(hdg);
        hdg = convertFromRelativeHDGToRef0(hdg, 90);
        float rad = radians(hdg / 100);
        xvec += val * cos(rad); // positivo per X
        yvec -= val * sin(rad); // negativo per Y
    }
    else if (hdg > 18000 && hdg < 27000) // heading negativo per X e Y -- basso sinistra -- riferimento 180°
    {
        if (going_backwards)
            hdg = invertHDG(hdg);
        hdg = convertFromRelativeHDGToRef0(hdg, 180);
        float rad = radians(hdg / 100);
        xvec -= val * sin(rad); // negativo per X
        yvec -= val * cos(rad); // negativo per Y
    }
    else // heading positivo per Y, negativo per X -- alto sinistra -- riferimento 270°
    {
        if (going_backwards)
            hdg = invertHDG(hdg);
        hdg = convertFromRelativeHDGToRef0(hdg, 270);
        float rad = radians(hdg / 100);
        xvec -= val * cos(rad); // negativo per X
        yvec += val * sin(rad); // positivo per Y
    }

    return std::make_tuple((int32_t)(xvec * 100), (int32_t)(yvec * 100));
}

uint32_t NAV::getMaxBlock(const char* map, bool fill)
{
    if (!USE_SD) return 0;
    uint32_t last_full_block = 0;
    uint32_t last_incomplete_block = 0;
    uint32_t max_readable_pos = navutil.last_readable_pos;
    uint32_t remainder = max_readable_pos % MAP_BLOCK_SIZE;
    NAVCore.println(F("Max MAP position is"), max_readable_pos);
    if (fill) { NAVCore.print(F("(Navigation) MAP ")); };
    if (remainder == 0)
        last_full_block = max_readable_pos / MAP_BLOCK_SIZE;
    else
    {
        last_incomplete_block = std::ceil((float)max_readable_pos / (float)MAP_BLOCK_SIZE);
        last_full_block = last_incomplete_block - 1;
    }

    if (fill && last_incomplete_block != 0)
    {
        NAVCore.print(F("incomplete, filling... "));
        uint32_t target_position = MAP_BLOCK_SIZE * last_incomplete_block /*- 1; TO CHECK*/;
        uint32_t points_to_write = (last_incomplete_block * MAP_BLOCK_SIZE - max_readable_pos) / 6;

        generic_point.x = 0;
        generic_point.y = 0;
        generic_point.id = 3;
        mapfile = SD.open(map, FILEAPPEND);
        mapfile.seek(max_readable_pos);
        for (uint32_t i = 0; i < points_to_write; i++)
            mapfile.write((uint8_t*)&generic_point, MAP_POINT_SIZE);
        mapfile.close();

        NAVCore.print(F("DONE "));
    }

    navutil.last_full_block = last_full_block;
    navutil.last_incomplete_block = last_incomplete_block;
    if (fill) { NAVCore.println(F("OK")); };

    return last_full_block;
}

OneDimensionPoint NAV::getTopPoint(const char* map, uint32_t axis)
{
    OneDimensionPoint p;
    p.value = -1000000000;
    uint32_t max_block = getMaxBlock(map);
    uint32_t current_block = 0;
    mapfile = SD.open(map);
    while (current_block < max_block)
    {
        readPointBlock(current_block);
        
        if (axis == X)
        {
            if (p.value < nav->MaxX->value)
            {
                p.block = nav->MaxX->block;
                p.idx = nav->MaxX->idx;
                p.value = nav->MaxX->value;
            }
        }
        else
        {
            if (p.value < nav->MaxY->value)
            {
                p.block = nav->MaxY->block;
                p.idx = nav->MaxY->idx;
                p.value = nav->MaxY->value;
            }
        }

        current_block++;
    }
    mapfile.close();

    return p;
}

OneDimensionPoint NAV::getBottomPoint(const char* map, uint32_t axis)
{
    OneDimensionPoint p;
    p.value = 1000000000;
    uint32_t max_block = getMaxBlock(map);
    uint32_t current_block = 0;

    mapfile = SD.open(map);
    while (current_block < max_block)
    {
        readPointBlock(current_block);

        if (axis == X)
        {
            if (p.value > nav->MinX->value)
            {
                p.block = nav->MinX->block;
                p.idx = nav->MinX->idx;
                p.value = nav->MinX->value;
            }
        }
        else
        {
            if (p.value > nav->MinY->value)
            {
                p.block = nav->MinY->block;
                p.idx = nav->MinY->idx;
                p.value = nav->MinY->value;
            }
        }

        current_block++;
    }
    mapfile.close();

    return p;
}

OneDimensionPoint NAV::getTopPointOf(uint32_t block, uint32_t axis)
{
    OneDimensionPoint p;
    p.value = -1000000000;
    readPointBlock(block);
    if (axis == X)
    {
        if (p.value < nav->MaxX->value)
        {
            p.idx = nav->MaxX->idx;
            p.value = nav->MaxX->value;
        }
    }
    else
    {
        if (p.value < nav->MaxY->value)
        {
            p.idx = nav->MaxY->idx;
            p.value = nav->MaxY->value;
        }
    }

    return p;
}

OneDimensionPoint NAV::getBottomPointOf(uint32_t block, uint32_t axis)
{
    OneDimensionPoint p;
    p.value = 1000000000;
    readPointBlock(block);
    if (axis == X)
    {
        if (p.value > nav->MinX->value)
        {
            p.idx = nav->MinX->idx;
            p.value = nav->MinX->value;
        }
    }
    else
    {
        if (p.value > nav->MinY->value)
        {
            p.idx = nav->MinY->idx;
            p.value = nav->MinY->value;
        }
    }

    return p;
}

std::tuple<int32_t, int32_t> NAV::getPointXY(uint32_t block, uint32_t point_idx)
{
    int32_t x = 0;
    int32_t y = 0;
    if (!USE_SD) return std::make_tuple(x, y);
    uint32_t block_position = (block == 1) ? 0 : MAP_BLOCK_SIZE * (block - 1) - 1;
    uint32_t idx_position = point_idx * 22;
    uint32_t seek_position = block_position + idx_position;
    mapfile = SD.open("/map.bin");
    mapfile.seek(seek_position);
    uint8_t buf[7];
    mapfile.read(buf, 7);
    for (uint32_t i = 0; i < 7; i++)
    {
        x *= 10;
        x += buf[i] - '0';
    }
    mapfile.read(buf, 7);
    for (uint32_t i = 0; i < 7; i++)
    {
        y *= 10;
        y += buf[i] - '0';
    }
    mapfile.close();

    return std::make_tuple(x, y);
}

uint32_t NAV::convertFromRef0ToRelativeHDG(uint32_t hdg, uint32_t ref, bool add)
{
    if (add)
        return hdg + ref * 100;
    else
        return 9000 + ref * 100 - hdg;
}

uint32_t NAV::setHeadingToPoint(int32_t target_x, int32_t target_y)
{
    /**
     * Heading in modailtà 360°
     * Il vettore X è positivo verso 90°
     * Il vettore Y è positivo verso 0°
     * |         0°         |
     * |     /       \      |
     * |  270°        90°   |
     * |     \       /      |
     * |        180°        |
     */

    int32_t ydiff = 0;
    int32_t xdiff = 0;
    uint32_t target_heading = 0;
    uint32_t current_heading = getHeading360();
    int32_t int_xvector = position.intxvector();
    int32_t int_yvector = position.intyvector();
    nav_ptr->busy = true;

    xdiff = abs(position.abs_intxvector() - abs(target_x));
    ydiff = abs(position.abs_intyvector() - abs(target_y));

    float angle = degrees(atan((float)(xdiff) / (float)(ydiff)));
    if (target_x > int_xvector && target_y > int_yvector) // alto destra
        target_heading = angle * 100;
    else if (target_x > int_xvector && target_y < int_yvector) // basso destra
        target_heading = convertFromRef0ToRelativeHDG((uint32_t)(angle * 100), 90, false);
    else if (target_x < int_xvector && target_y < int_yvector) // basso sinistra
        target_heading = convertFromRef0ToRelativeHDG((uint32_t)(angle * 100), 180, true);
    else if (target_x < int_xvector && target_y > int_yvector) // alto sinistra
        target_heading = convertFromRef0ToRelativeHDG((uint32_t)(angle * 100), 270, false);

    int32_t target_heading_180 = convertHDGTo180(target_heading);

    rotateToDeg(target_heading, getRotationDirection(current_heading, target_heading_180));

    return target_heading;
}

uint32_t NAV::setHeadingToPoint(Point *p)
{
    return setHeadingToPoint(p->x, p->y);
}

uint32_t NAV::getPointDst(int32_t x, int32_t y)
{
    int32_t pos_xvector = abs(position.current_xcm);
    int32_t pos_yvector = abs(position.current_ycm);

    int32_t xdiff = 0;
    int32_t ydiff = 0;
    xdiff = pos_xvector - abs(x);
    ydiff = pos_yvector - abs(y);

    uint32_t dst = sqrt(pow(abs(xdiff), 2) + pow(abs(ydiff), 2)); // pitagora
    return dst;
}

uint32_t NAV::getPointDst(Point* p)
{
    return getPointDst(p->x, p->y);
}

void NAV::goToPoint(int32_t x, int32_t y, bool precedence)
{
    std::vector<uint32_t> commands = {SETHDGTOPOINT, GOFORWARD};
    std::vector<int32_t> data = {x, y, (int32_t)getPointDst(x, y), NAVSensors.getHeading()};
    addToCommandQueue(&commands, &data, nav_ptr, precedence);
}

void NAV::goToPoint(Point* p, bool precedence)
{
    goToPoint(p->x, p->y);
}

void NAV::scroll()
{
    updateMapInfo("/map.bin", false);

    char* chars = new char[80];
    sprintf(chars, "Bottom point: %d, %d", mapinfo.minx, mapinfo.miny);
    NAVCore.println(chars);
    for (int32_t i = 0; i < 80; i++)
        chars[i] = 0;
    sprintf(chars, "Top point: %d, %d", mapinfo.maxx, mapinfo.maxy);
    NAVCore.println(chars);

    delete[] chars;

    processMap();

    //for (int32_t i = 0; i < mapinfo.height_in_squares; i++)
    //{
    //    getMapSquare(i);    
    //}

    //xTaskCreatePinnedToCore(mapperfun, "mapper", 10000, NULL, 5, &mapper, 1);


    /*
    xTaskCreatePinnedToCore(ramsizetask, "ramsizetask", 10000, NULL, 15, &ramSizeTask, 0);
    void* stack = heap_caps_malloc(524288, MALLOC_CAP_SPIRAM);
    xTaskCreatePinnedToCore(astar, "ramsizetask", 524288, NULL, 24, &astartask, 0);
    /

    processMap(lowest_x, lowest_y, highest_x, highest_y);*/


    generic_point.x = 0;
    generic_point.y = 0;
    //nav_ptr->addGoToPoint(&generic_point);

    /*commands.push_back(GOTOPOINT);
    command_data.push_back(0);
    command_data.push_back(0);
    addToCommandQueue(nav_ptr);*/



    //rotateToDeg(0, getRotationDirection(convertHDGTo180(getHeading360()), 0));
    //displayQueue(nav_ptr);
    //removeDuplicatesFromMap(mapinfo.width, mapinfo.height, mapinfo.minx, mapinfo.miny, mapinfo.fill_start_pos);
}

void NAV::pause()
{
    NAVSensors.setMotorsStop(); // resetta gli encoder, altrimenti il tempo registrato farebbe sballare le misure
    NAVMotors.stop();
    is_paused = true;
}

void NAV::resume()
{
    // non serve specificare gli argomenti perché con la variabile is_paused, le funzioni sanno già che non devono resettare la logica
    if (currcmd.id == GOFORWARD)
        goForward(0, 0);
    else if (currcmd.id == GOBACKWARDS)
        goBackwards(0, 0);

    is_paused = false;
}

int32_t NAV::convertCharVecToInt(std::vector<char> a)
{
    uint32_t i = 0;
    int32_t num = 0;
    bool neg = false;
    if (a[0] == '-')
        neg = true;
    while (a[i] != '\0')
    {
        if (i == 0 && neg)
        {
            i++;
            continue;
        }
        num = (a[i] - '0') + (num * 10);
        i++;
    }
    if (neg)
        return num * -1;
    else
        return num;
}

void NAV::motorStall()
{
    //mots.stall = true;
}

bool NAV::checkMapCompletion()
{
    if (!navutil.map_completion_checked)
        navutil.map_completion_checked = true;
    else
        return navutil.map_not_full;

    uint32_t accessible_points_count = 0;
    mapfile = SD.open("/map.bin");
    for (uint32_t current_block = 0; current_block < navutil.last_full_block; current_block++)
    {
        readPointBlock(current_block);
        for (uint32_t i = 0; i < 256; i++)
        {
            if (nav->points[i].id == ACCESSIBLE)
                accessible_points_count++;
        }
    }
    mapfile.close();

    if (accessible_points_count < 50)
    {
        NAVCore.println(F("(Navigation) MAP has few accessible points!"), accessible_points_count);
        navutil.map_not_full = true;
        return true;
    }
    else
        return false;
}

void NAV::mapLogging(bool on_off)
{
    navutil.log_active = on_off;
}

uint32_t NAV::invert180HDG(int32_t hdg)
{
    return 180 - abs(hdg);
}

uint32_t NAV::getMeanDiff(uint32_t block, uint32_t axis)
{
    readBlock(block);
    uint32_t total_diff = 0;
    for (uint32_t i = 0; i < 255; i++) // fino a 254 perché c'è nav->arrX[i + 1] quindi al ciclo 254 sarebbe 255 (max)
    {
        if (axis == X)
            diff = abs(nav->arrX[i + 1] - nav->arrX[i]);
        else
            diff = abs(nav->arrY[i + 1] - nav->arrY[i]);
        total_diff += diff;
    }
    uint32_t mean_diff = (uint32_t)((float)total_diff / 255);
    return mean_diff;
}

void NAV::addToCommandQueue(uint32_t cmd, std::vector<int32_t> *input_data, NAV::CommandQueue *queue, bool higher_priority)
{
    if (higher_priority)
    {
        queue->commands.emplace(queue->commands.begin(), cmd);
        for (uint32_t i = input_data->size() - 1; i >= 0; i--)
            queue->data.emplace(queue->data.begin(), (*input_data)[i]);
    }
    else
    {
        queue->commands.push_back(cmd);
        for (uint32_t i = 0; i < input_data->size(); i++)
            queue->data.push_back((*input_data)[i]);
    }
}

void NAV::addToCommandQueue(std::vector<uint32_t> *input_cmds, std::vector<int32_t> *input_data, NAV::CommandQueue *queue, bool higher_priority)
{
    if (higher_priority)
    {
        for (uint32_t i = input_cmds->size() - 1; i >= 0; i--)
        {
            queue->commands.emplace(queue->commands.begin(), (*input_cmds)[i]);
            if (i == 0)
                break;
        }
        for (uint32_t i = input_data->size() - 1; i >= 0; i--)
        {
            queue->data.emplace(queue->data.begin(), (*input_data)[i]);
            if (i == 0)
                break;
        }
    }
    else
    {
        for (uint32_t i = 0; i < input_cmds->size(); i++)
            queue->commands.push_back((*input_cmds)[i]);
        for (uint32_t i = 0; i < input_data->size(); i++)
            queue->data.push_back((*input_data)[i]);
    }
}

void NAV::addToCommandQueue(NAV::CommandQueue *queue, bool higher_priority)
{
    if (higher_priority)
    {
        for (uint32_t i = commands.size() - 1; i >= 0; i--)
        {
            queue->commands.emplace(queue->commands.begin(), commands[i]);
            if (i == 0)
                break;
        }
        for (uint32_t i = command_data.size() - 1; i >= 0; i--)
        {
            queue->data.emplace(queue->data.begin(), command_data[i]);
            if (i == 0)
                break;
        }
    }
    else
    {
        for (uint32_t i = 0; i < commands.size(); i++)
            queue->commands.push_back(commands[i]);
        for (uint32_t i = 0; i < command_data.size(); i++)
            queue->data.push_back(command_data[i]);
    }

    commands.clear();
    command_data.clear();
}

void NAV::displayQueue(CommandQueue *queue)
{
    NAVCore.print(F("Command queue contains:"));
    for (uint32_t i = 0; i < queue->commands.size(); i++)
        NAVCore.print(" ", queue->commands[i]);
    NAVCore.println("");
    NAVCore.print(F("Data queue contains:"));
    for (uint32_t i = 0; i < queue->data.size(); i++)
        NAVCore.print(" ", queue->data[i]);
    NAVCore.println("");
}

std::tuple<uint32_t, uint32_t, int32_t, int32_t> NAV::getTopPointWith(int32_t coordinate, uint32_t given_coordinate_axis)
{
    uint32_t current_block = getBlockContaining(coordinate, given_coordinate_axis);
    readBlock(current_block);
    int32_t current_x, current_y = 0;
    int32_t p_x, p_y, max_p_x, max_p_y = 0;
    uint32_t p_dst, last_p_dst, p_id, p_idx, p_block = 0;
    uint32_t max_p_idx, max_p_block = 0;

    bool border_found = false;
    bool top_found = false;

    if (given_coordinate_axis == X)
    {
        while (current_y <= nav->MaxY->value)
        {
            std::tie(p_dst, p_id, p_idx, p_block) = getClosestPointDstFakeXY(coordinate, current_y);
            if (p_dst <= MAP_POINT_PROXIMITY_DST && p_id == 2) // id 2 == bordo
            {
                std::tie(p_x, p_y) = getPointXY(p_block, p_idx);
                if (p_y > max_p_y)
                {
                    max_p_x = p_x;
                    max_p_y = p_y;
                    max_p_idx = p_idx;
                    max_p_block = p_block;
                    top_found = true;
                }

                if (last_p_dst == p_dst)
                    break;
            }

            last_p_dst = p_dst;
            current_y += 0.5 * ((int32_t)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
        }

        if (!top_found)
        {
            while (current_y >= nav->MinY->value)
            {
                std::tie(p_dst, p_id, p_idx, p_block) = getClosestPointDstFakeXY(coordinate, current_y);
                if (p_dst <= MAP_POINT_PROXIMITY_DST && p_id == 2) // id 2 == bordo
                {
                    std::tie(p_x, p_y) = getPointXY(p_block, p_idx);
                    if (p_y > max_p_y)
                    {
                        max_p_x = p_x;
                        max_p_y = p_y;
                        max_p_idx = p_idx;
                        max_p_block = p_block;
                        top_found = true;
                    }

                    if (last_p_dst == p_dst)
                        break;
                }

                last_p_dst = p_dst;
                current_y -= 0.5 * ((int32_t)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
            }
        }
    }
    else
    {
        while (current_x <= nav->MaxX->value)
        {
            std::tie(p_dst, p_id, p_idx, p_block) = getClosestPointDstFakeXY(current_x, coordinate);
            if (p_dst <= MAP_POINT_PROXIMITY_DST && p_id == 2) // id 2 == bordo
            {
                std::tie(p_x, p_y) = getPointXY(p_block, p_idx);
                if (p_x > max_p_x)
                {
                    max_p_x = p_x;
                    max_p_y = p_y;
                    max_p_idx = p_idx;
                    max_p_block = p_block;
                    top_found = true;
                }

                if (last_p_dst == p_dst)
                    break;
            }

            last_p_dst = p_dst;
            current_x += 0.5 * ((int32_t)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
        }

        if (!top_found)
        {
            while (current_x >= nav->MinX->value)
            {
                std::tie(p_dst, p_id, p_idx, p_block) = getClosestPointDstFakeXY(current_x, coordinate);
                if (p_dst <= MAP_POINT_PROXIMITY_DST && p_id == 2) // id 2 == bordo
                {
                    std::tie(p_x, p_y) = getPointXY(p_block, p_idx);
                    if (p_x > max_p_x)
                    {
                        max_p_x = p_x;
                        max_p_y = p_y;
                        max_p_idx = p_idx;
                        max_p_block = p_block;
                        top_found = true;
                    }

                    if (last_p_dst == p_dst)
                        break;
                }

                last_p_dst = p_dst;
                current_y -= 0.5 * ((int32_t)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
            }
        }
    }

    return std::make_tuple(max_p_block, max_p_idx, max_p_x, max_p_y);
}

std::tuple<uint32_t, uint32_t, int32_t, int32_t> NAV::getBottomPointWith(int32_t coordinate, uint32_t given_coordinate_axis)
{
    uint32_t current_block = getBlockContaining(coordinate, given_coordinate_axis);
    readBlock(current_block);
    int32_t current_x, current_y = 0;
    int32_t p_x, p_y = 0;
    int32_t max_p_x, max_p_y = 1000000000;
    uint32_t p_dst, last_p_dst, p_id, p_idx, p_block = 0;
    uint32_t max_p_idx, max_p_block = 0;

    bool border_found = false;
    bool top_found = false;

    if (given_coordinate_axis == X)
    {
        while (current_y <= nav->MaxY->value)
        {
            std::tie(p_dst, p_id, p_idx, p_block) = getClosestPointDstFakeXY(coordinate, current_y);
            if (p_dst <= MAP_POINT_PROXIMITY_DST && p_id == 2) // id 2 == bordo
            {
                std::tie(p_x, p_y) = getPointXY(p_block, p_idx);
                if (p_y < max_p_y)
                {
                    max_p_x = p_x;
                    max_p_y = p_y;
                    max_p_idx = p_idx;
                    max_p_block = p_block;
                    top_found = true;
                }

                if (last_p_dst == p_dst)
                    break;
            }

            last_p_dst = p_dst;
            current_y += 0.5 * ((int32_t)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
        }

        if (!top_found)
        {
            while (current_y >= nav->MinY->value)
            {
                std::tie(p_dst, p_id, p_idx, p_block) = getClosestPointDstFakeXY(coordinate, current_y);
                if (p_dst <= MAP_POINT_PROXIMITY_DST && p_id == 2) // id 2 == bordo
                {
                    std::tie(p_x, p_y) = getPointXY(p_block, p_idx);
                    if (p_y < max_p_y)
                    {
                        max_p_x = p_x;
                        max_p_y = p_y;
                        max_p_idx = p_idx;
                        max_p_block = p_block;
                        top_found = true;
                    }

                    if (last_p_dst == p_dst)
                        break;
                }

                last_p_dst = p_dst;
                current_y -= 0.5 * ((int32_t)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
            }
        }
    }
    else
    {
        while (current_x <= nav->MaxX->value)
        {
            std::tie(p_dst, p_id, p_idx, p_block) = getClosestPointDstFakeXY(current_x, coordinate);
            if (p_dst <= MAP_POINT_PROXIMITY_DST && p_id == 2) // id 2 == bordo
            {
                std::tie(p_x, p_y) = getPointXY(p_block, p_idx);
                if (p_x < max_p_x)
                {
                    max_p_x = p_x;
                    max_p_y = p_y;
                    max_p_idx = p_idx;
                    max_p_block = p_block;
                    top_found = true;
                }

                if (last_p_dst == p_dst)
                    break;
            }

            last_p_dst = p_dst;
            current_x += 0.5 * ((int32_t)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
        }

        if (!top_found)
        {
            while (current_x >= nav->MinX->value)
            {
                std::tie(p_dst, p_id, p_idx, p_block) = getClosestPointDstFakeXY(current_x, coordinate);
                if (p_dst <= MAP_POINT_PROXIMITY_DST && p_id == 2) // id 2 == bordo
                {
                    std::tie(p_x, p_y) = getPointXY(p_block, p_idx);
                    if (p_x < max_p_x)
                    {
                        max_p_x = p_x;
                        max_p_y = p_y;
                        max_p_idx = p_idx;
                        max_p_block = p_block;
                        top_found = true;
                    }

                    if (last_p_dst == p_dst)
                        break;
                }

                last_p_dst = p_dst;
                current_y -= 0.5 * ((int32_t)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
            }
        }
    }

    return std::make_tuple(max_p_block, max_p_idx, max_p_x, max_p_y);
}

uint32_t NAV::getBlockContaining(int32_t coordinate, uint32_t axis)
{
    uint32_t current_block = 1;
    bool block_found = false;
    mapfile = SD.open("/map.bin");
    while (current_block <= navutil.last_full_block)
    {
        readBlock(current_block, false);
        if (axis == X)
        {
            if (coordinate <= nav->MaxX->value && coordinate >= nav->MinX->value)
            {
                block_found = true;
                break;
            }
        }
        else
        {
            if (coordinate <= nav->MaxY->value && coordinate >= nav->MinY->value)
            {
                block_found = true;
                break;
            }
        }

        current_block++;
    }
    mapfile.close();

    if (block_found)
        return current_block;
    else
        return NOT_FOUND;
}

void NAV::rotateForPivot(int32_t degs)
{
    /*
            0°
        /       \
    270°           90°
        \       /
           180°
    */

    dir.is_pivoting = true;
    nav_ptr->busy = true;
    resetMovementVars();
    Rotation.diff_to_rotate = abs(degs);
    int32_t heading_to_reach = degs * 100;
    if (heading_to_reach < 0)
        heading_to_reach = heading_to_reach + 36000; // 36000
    Rotation.start_heading = getHeading360();
    int32_t over_start_heading = Rotation.start_heading + heading_to_reach;
    if (over_start_heading - 36000 < 0)
        heading_target = over_start_heading;
    else
        heading_target = over_start_heading - 36000;

    Rotation.rotating = true;

    if (degs < 0)
    {
        dir.direction = LEFT;
        NAVMotors.left(true);

    }
    else
    {
        dir.direction = RIGHT;
        NAVMotors.right(true);
    }

    dir.target_heading = heading_target;
    NAVSensors.startSensorPolling();    // deve controllare se ci sono ostacoli mentre ruota

}

void NAV::clearQueue(CommandQueue *queue, uint32_t items_to_delete)
{
    if (items_to_delete == ALL)
    {
        while (queue->commands.size() > 0)
            queue->commands.erase(queue->commands.begin());
        while (queue->data.size() > 0)
            queue->data.erase(queue->data.begin());
    }
    else
    {
        if (queue->commands.size() >= items_to_delete)  // dà errore se prova ad eliminare un elemento da un vettore vuoto
        {
            for (int32_t i = 0; i < items_to_delete; i++)
            {
                // elimina il numero richiesto di dati in base al comando
                if (queue->commands[0] == GOFORWARD || queue->commands[0] == GOBACKWARDS && queue->data.size() >= 2 )
                    { queue->data.erase(queue->data.begin(), queue->data.begin() + 2); }
                else if (queue->commands[0] == ROTATETO && queue->data.size() >= 2) { queue->data.erase(queue->data.begin(), queue->data.begin() + 2); }
                else if (queue->commands[0] == ROTATEFOR && queue->data.size() >= 1) { queue->data.erase(queue->data.begin()); }
                else if (queue->commands[0] == GOTOPOINT && queue->data.size() >= 2) { queue->data.erase(queue->data.begin(), queue->data.begin() + 2); }
                else if (queue->commands[0] == SETHDGTOPOINT && queue->data.size() >= 2) { queue->data.erase(queue->data.begin(), queue->data.begin() + 2); }
                else if (queue->commands[0] == ROTATEFORPIVOT && queue->data.size() >= 1) { queue->data.erase(queue->data.begin()); }
                queue->commands.erase(queue->commands.begin());
            }
        }
    }
}

int32_t NAV::getRealAngleDiff(int32_t angle1, int32_t angle2)
{
    diff = angle1 - angle2;
    while (diff <= -18000)
        diff += 36000;
    while (diff > 18000)
        diff -= 36000;
    return diff;
}

bool sortbysec(const Point& a, const Point& b)
{
    return (a.x == b.x && a.y < b.y);
}

void NAV::processMap()
{
    uint32_t available_arr_size = getMaxHeapArraySize(MAP_POINT_SIZE);  // usa il 90% della PSRAM disponibile
    uint32_t map_size = 0;

    mapfile = SD.open("/map.bin", FILEREAD);
    map_size = mapfile.size() / 6;
    mapfile.close();

    uint32_t needed_read_blocks = 0;

    if (map_size > available_arr_size)
    {
        log_e("Errore: la mappa è troppo grande! Ci sono %d punti, quando il massimo è %d.\n", map_size, available_arr_size);
        return;
    }

    // inizializzazione del vettore con punti (0,0)
    std::vector<Point> pts;
    pts.resize(map_size);

    generic_point.x = 0;
    generic_point.y = 0;
    generic_point.id = 0;
    for (int32_t i = 0; i < map_size; i++)
        pts[i] = generic_point;

    chrono.start();
    generic_point.x = mapinfo.minx;
    generic_point.y = mapinfo.miny;

    // prepara la mappa, riempiendo i buchi
    readMapForSize("/map.bin", pts, map_size);
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());  // elimina i doppioni - sono rari, ma ci sono

    // copia la mappa originale in un altro file, come backup
    mapfile = SD.open("/raw_map.bin", FILEWRITE);
    for (uint32_t i = 0; i < pts.size(); i++)
        mapfile.write((uint8_t*)&(pts[i]), MAP_POINT_SIZE);
    mapfile.close();

    fillEmptyPointsInLine(pts, map_size);   // questa funzione deve trovarsi i punti uno dopo l'altro, non riordinati

    // sort per fare in modo che i prossimi for possano controllare i punti in modo progressivo, senza dover loopare tutto l'array
    std::sort(pts.begin(), pts.end());
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());

    uint32_t current_position = 0;
    mapfile = SD.open("/map.bin", FILEWRITE);
    for (uint32_t col = 0; col < mapinfo.width; col++)
    {
        for (uint32_t row = 0; row < mapinfo.height; row++)
        {
            generic_point.id = ACCESSIBLE;
            if (generic_point == pts[current_position])
            {
                generic_point.id = BORDER;
                current_position++;
            }

            mapfile.write((uint8_t*)&generic_point, MAP_POINT_SIZE);
            generic_point.y++;
        }

        NAVCore.print("Processed column", col);
        NAVCore.println("of", mapinfo.width);

        generic_point.x++;
        generic_point.y = mapinfo.miny;
    }
    mapfile.close();
    chrono.printTime();

    updateMapInfo("/map.bin", true, true);

    SD.end();
}

int32_t NAV::charArrToInt(uint8_t* arr, uint32_t len)
{
    uint32_t power = 1;
    int32_t res = 0;

    for (int32_t i = len - 1; i >= 0; i--)
    {
        if (arr[i] == 45)
        {
            res *= -1;
            break;
        }
        res += (arr[i] - '0') * power;
        power *= 10;
    }

    return res;
}

void NAV::resetArray(uint8_t* arr, uint32_t len)
{
    for (int32_t i = 0; i < len; i++)
        arr[i] = 0;
}

bool NAV::readPointBlock(uint32_t block)
{
    uint32_t start_position = MAP_BLOCK_SIZE * block;
    if (!USE_SD || nav->current_block == block) return false;
    if (start_position + MAP_BLOCK_SIZE > navutil.last_readable_pos)
    {
        log_e("SD Target read position greater than last readable position (%d > %d)", start_position + MAP_BLOCK_SIZE, navutil.last_readable_pos);
        return false;
    }
    Serial.printf("Reading block %d\n", block);

    nav->current_block = block;
    nav->reset();

    mapfile.seek(start_position);
    for (int32_t i = 0; i < POINTS_PER_BLOCK; i++)
    {
        mapfile.read((uint8_t*)&(nav->points[i]), MAP_POINT_SIZE);
        if (nav->points[i].x > nav->MaxX->value)
            nav->MaxX->value = nav->points[i].x;
        if (nav->points[i].y > nav->MaxY->value)
            nav->MaxY->value = nav->points[i].y;
        if (nav->points[i].x < nav->MinX->value)
            nav->MinX->value = nav->points[i].x;
        if (nav->points[i].y < nav->MinY->value)
            nav->MinY->value = nav->points[i].y;
    }

    if (nav->MaxX->value != 0 || nav->MaxY->value != 0 || nav->MinX->value != 0 || nav->MinY->value != 0)
        return true;

    return false;
}

uint32_t NAV::getPointPositionInSD(Point p, bool is_fill)
{
    if (is_fill)
        return mapinfo.fill_start_pos + ((mapinfo.height * (abs(p.x - mapinfo.minx)) + abs(p.y - mapinfo.miny) + abs(p.x - mapinfo.minx)) * MAP_POINT_SIZE);
    else
        return (mapinfo.height * (abs(p.x - mapinfo.minx)) + abs(p.y - mapinfo.miny) + abs(p.x - mapinfo.minx)) * MAP_POINT_SIZE;
}

void NAV::getMapSquare(uint32_t idx)
{
    Point p = Point(0, 0, 0);
    if (idx < mapinfo.height_in_squares)
    {
        p.y = mapinfo.miny + idx * MAP_SQUARE_WIDTH;
        p.x = mapinfo.minx;
    }
    else
    {
        p.y = mapinfo.miny + (idx - mapinfo.height_in_squares * (idx / mapinfo.height_in_squares)) * MAP_SQUARE_WIDTH;
        p.x = mapinfo.minx + idx / mapinfo.height_in_squares * MAP_SQUARE_WIDTH;
    }

    //Serial.printf("Requesting block %d (max %d). X: %d, Y: %d\n", idx, mapinfo.height_in_squares, p.x, p.y);
    log_w("Requesting block %d (max %d). X: %d, Y: %d\n", idx, mapinfo.height_in_squares, p.x, p.y);

    uint32_t starting_point_position = getPointPositionInSD(p);
    uint32_t starting_x = p.x;
    uint32_t starting_y = p.y;
    uint32_t id = 0;

    psvec<Point> obstacles;

    generic_point.id = OBSTACLE;
    mapfile = SD.open("/map.bin", FILEREAD);
    for (uint32_t col = 0; col < MAP_SQUARE_WIDTH; col++)
    {
        p.x += 1;
        for (uint32_t row = 0; row < MAP_SQUARE_WIDTH; row++)
        {
            p.y += 1;
            id = readPointID(p);
            if (id == OBSTACLE || id == BORDER)
            {
                generic_point.x = abs(p.x - starting_x);
                generic_point.y = abs(p.y - starting_y);
                if (generic_point.y != 15 || generic_point.y != 14 || generic_point.y != 16)
                    obstacles.push_back(generic_point);
            }
        }
        p.y = starting_y;
    }
    mapfile.close();

    //Serial.printf("mapp:\n");

    //for (uint32_t i = 0; i < MAP_SQUARE_WIDTH; i++)
        //Serial.printf("(%d; %d)\n", obstacles[i].x, obstacles[i].y);

    //Serial.printf("1 Heap: %d --- PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());
    Point start = Point(31, 0);
    Point end = Point(0, 31);

    Map mp;
    psvec<Point> path = mp.a_star(&obstacles, start, end);

    int32_t sas[32][32] =
    {
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
        {48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48}
    };

    //for (uint32_t i = 0; i < obstacles.size(); i++)
    //    sus[obstacles[i].x][obstacles[i].y] = '1';

    //for (int32_t i = 0; i < 32; i++)
    //{
    //    for (int32_t j = 0; j < 32; j++)
    //    {
    //        Serial.printf("%c ", sus[j][i]);
    //    }
    //    Serial.printf("\n");
    //}
    //Serial.printf("\nBECOMES \\/\\/\\/\\/\\/\n");

    for (int32_t i = 0; i < path.size(); i++)
    {
        int32_t x = path[i].x;
        int32_t y = path[i].y;
        sas[x][y] = '.';
    }

    sas[start.x][start.y] = '+';
    sas[end.x][end.y] = '-';

    //for (int32_t i = 0; i < 32; i++)
    //{
    //    for (int32_t j = 0; j < 32; j++)
    //    {
    //        Serial.printf("%c ", sus[j][i]);
    //    }
    //    Serial.printf("\n");
    //}
    //Serial.printf("\n\n");

    log_w("2 Heap: %d --- PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());

    /*int32_t x = (idx % mapinfo.height_in_squares != 0);
    for (uint32_t i = 0; i < dim; i++)
    {
        for (uint32_t j = 0; j < dim; j++)
        {
            getPointPositionInSD(Point(dim * dim));
        }
    }*/
}

uint32_t NAV::readPointID(Point p)
{
    mapfile.seek(getPointPositionInSD(p));
    mapfile.read((uint8_t*)&p, MAP_POINT_SIZE);
    return p.id;
}

void NAV::getMapInfo()
{
    mapfile = SD.open("/mapinfo.bin", FILEREAD);
    mapfile.read((uint8_t*)&mapinfo, sizeof(MapCreationInfo));
    mapfile.close();
}

uint32_t NAV::getMaxHeapArraySize(uint32_t data_size)
{
    return uint32_t(std::floor((MAP_RAM_USAGE_PRCTG * (ESP.getFreePsram() - SD_RAM_USAGE_SAFETY_BUFFER) / 100) / (float)(data_size)));
}

void NAV::readMapForSize(const char* map, std::vector<Point>& vec, uint32_t size, uint32_t start_point_idx)
{
    mapfile = SD.open(map, FILEREAD);
    mapfile.seek(start_point_idx * 6);
    for (int32_t i = 0; i < size; i++)
        mapfile.read((uint8_t*)&(vec[i]), MAP_POINT_SIZE);
    mapfile.close();
}

void NAV::fillEmptyPointsInLine(std::vector<Point>& vec, uint32_t size)
{
    generic_point.id = BORDER;

    for (size_t i = 0; i < size - 1; i++)
    {
        const Point& p1 = vec[i];
        const Point& p2 = vec[i + 1];
        int x1 = p1.x, y1 = p1.y;
        int x2 = p2.x, y2 = p2.y;
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            generic_point.x = x1;
            generic_point.y = y1;
            vec.push_back(generic_point);

            if (x1 == x2 && y1 == y2)
                break;

            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y1 += sy;
            }
        }
    }
}

bool NAV::updateMapInfo(const char* map, bool write_to_file, bool write_only)
{
    if (!write_only)
    {
        OneDimensionPoint bot_x;
        OneDimensionPoint bot_y;
        OneDimensionPoint top_x;
        OneDimensionPoint top_y;

        int32_t minx = 10000000;
        int32_t miny = 10000000;
        int32_t maxx = -10000000;
        int32_t maxy = -10000000;

        mapfile = SD.open(map, FILEREAD);
        for (uint32_t i = 0; i < getMaxBlock(map); i++)
        {
            bot_x = getBottomPointOf(i, X);
            if (bot_x.value < minx)
                minx = bot_x.value;
            bot_y = getBottomPointOf(i, Y);
            if (bot_y.value < miny)
                miny = bot_y.value;
            top_x = getTopPointOf(i, X);
            if (top_x.value > maxx)
                maxx = top_x.value;
            top_y = getTopPointOf(i, Y);
            if (top_y.value > maxy)
                maxy = top_y.value;
        }
        mapfile.close();

        if (maxx == -10000000 || maxy == -10000000 || minx == 10000000 || miny == 10000000)
            return false;

        uint32_t width = abs(maxx - minx);
        uint32_t height = abs(maxy - miny);

        // adattamento dei limiti della mappa per fare in modo che tutto possa essere diviso in quadrati di lato MAP_SQUARE_WIDTH
        uint32_t w_compensation = (width != MAP_SQUARE_WIDTH) ? std::ceil((float)width / (float)MAP_SQUARE_WIDTH) * MAP_SQUARE_WIDTH - width : 0;
        uint32_t h_compensation = (height != MAP_SQUARE_WIDTH) ? std::ceil((float)height / (float)MAP_SQUARE_WIDTH) * MAP_SQUARE_WIDTH - height : 0;

        width += w_compensation;
        height += h_compensation;

        maxx += w_compensation;
        maxy += h_compensation;

        mapinfo.maxx = maxx;
        mapinfo.maxy = maxy;
        mapinfo.minx = minx;
        mapinfo.miny = miny;
        mapinfo.width = width;
        mapinfo.height = height;
        mapinfo.width_in_squares = width / MAP_SQUARE_WIDTH;
        mapinfo.height_in_squares = height / MAP_SQUARE_WIDTH;
    }

    if (write_to_file)
    {
        mapfile = SD.open("/mapinfo.bin", FILEWRITE);
        mapfile.write((uint8_t*)&mapinfo, sizeof(MapCreationInfo));
        mapfile.close();
    }

    return true;
}

uint32_t NAV::convertHDGTo360(int32_t heading)
{
    if (heading >= 0)
        return heading;
    else
        return heading + 36000;
}

void NAV::begin()
{
    nav_ptr = &navqueue;
    avoid_ptr = &avoidqueue;

    currcmd.isbusy = &navqueue.busy;

    if (!USE_SD) return;
    spi->begin(SCLK, MISO, MOSI, CS);
    //SPI.begin(SCLK, MISO, MOSI, CS);
    if (SD.begin(CS, *spi))
        NAVCore.println(F("(Navigation) SD Card OK"));
    else
    {
        NAVCore.println(F("(Navigation) SD Card not recognized, RESTART IN 5 SECONDS"));
        delay(5000);
        ESP.restart();
    }

    navutil.last_readable_pos = getSDLastReadablePosition("/map.bin");  // consente a getMaxBlock di funzionare la prima volta
    getMaxBlock("/map.bin", true);
    navutil.last_readable_pos = getSDLastReadablePosition("/map.bin");  // aggiorna la posizione nel caso getMaxBlock l'abbia aggiornata (solo con fill = true)
    mapfile = SD.open("/map.bin", FILEREAD);
    readPointBlock(1);
    mapfile.close();
    getMapInfo();
    nav->reset(true);
}
