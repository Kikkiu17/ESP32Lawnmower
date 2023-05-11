#include <vector>
#include <tuple>
#include <SD.h>
#include <Navigation.h>
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
std::vector<int> map_arr_data((POINTS_PER_BLOCK) * 3);  // ci sono 768 dati in 256 punti (256 * 3 (x, y, ID)) o 2816 byte di punti
uint8_t* block_buffer = new uint8_t[MAP_BLOCK_SIZE];        // messo nell'heap così è più veloce
std::vector<char> temp_data(256);                           // convertCharArrToInt accetta solo array da 256 elementi
uint8_t raw_point[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
Point generic_point = Point();

std::vector<unsigned int> commands;
std::vector<int> command_data;

#define GOFORWARD 1             // --- 2 argomenti ---
#define GOBACKWARDS 2           // --- 2 argomenti ---
#define ROTATETO 3              // --- 2 argomenti ---
#define ROTATEFOR 4             // --- 1 argomento ---
#define GOTOPOINT 5             // --- 2 argomenti - AGGIUNGE DUE COMANDI ALLA CODA ---
#define SETHDGTOPOINT 6         // --- 2 argomenti ---
#define ROTATEFORPIVOT 7        // --- 1 argomento ---

#define FILEREAD "r"
#define FILEWRITE "w"
#define FILEAPPEND "r+"

SPIClass *spi = new SPIClass(FSPI);
File mapfile;

Mux NAVMux;
Sensors NAVSensors;
Motors NAVMotors;
Core NAVCore;
unsigned int t3 = 0;

/*
ID BLOCCHI MAPPA
0: accessibile
1: non accessbile
2: bordo mappa
3: ignora punto
*/

bool is_paused = false;
bool autorun = false;
unsigned int distance_target = 0;
float distance_traveled = 0.00;
unsigned int heading_target = 0;
bool obstacle_detected = false;
bool going_forward = false;
bool gone_forward = false;
bool going_backwards = false;
bool gone_backwards = false;
bool distance_target_reached = false;
bool robot_moving_x_y = false;
bool forward_wait_for_rotation = false;
unsigned int forward_wait_for_rotation_val = 0;
int forward_wait_for_rotation_hdg = 0;
unsigned int timer = millis();
bool bordermode = false;
unsigned int bordermode_start_time = 0;


class CurrentCommand
{
public:
    int id = 0;
    bool* isbusy = nullptr;
};


class TaskUtil
{
    private:
        unsigned int time0 = 0;
    public:
        unsigned int timeout = 1000;    // ms per lo yield
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
        unsigned int start_time = 0;
    public:
        void start() { start_time = millis(); }
        unsigned int getTime() { return millis() - start_time; }
};


class DirControl
{
    public:
        unsigned int direction = 0;
        int target_heading = 0;
        bool is_pivoting = false;       // true se sta girando con una ruota ferma (pivot)
        int last_turn_dir = LEFT;
        /**
         * @brief ottiene la direzione in cui deve girare ora, contrario di last_turn_dir. Inverte anche last_turn_dir
        */
        unsigned int getDirectionToTurn()
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
    unsigned int start_heading = 0;
    unsigned int diff_to_rotate = 0;
    bool rotating = false;
    bool rotated = false;
    bool pivot = false;
} Rotation;


class Vectors
{
    public:
        float xvector = 0;          // cm (1.23)
        float yvector = 0;          // cm (1.23)
        float last_dst = 0;
        int last_xcm = 0;       // cm (1)
        int current_xcm = 0;    // cm (1)
        int last_ycm = 0;       // cm (1)
        int current_ycm = 0;    // (cm 1)

        int intxvector() { return (int)(xvector * 100); };                      // mm (123)
        int intyvector() { return (int)(yvector * 100); };                      // mm (123)
        int abs_intxvector() { return (unsigned int)(abs((int)xvector * 100)); };   // mm (123)
        int abs_intyvector() { return (unsigned int)(abs((int)yvector * 100)); };   // mm (123)
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
    unsigned int current_block = 0;
    std::vector<std::vector<Point>> square;
    Point* points = new Point[POINTS_PER_BLOCK];
    int arrX[POINTS_PER_BLOCK] = {};
    int arrY[POINTS_PER_BLOCK] = {};
    unsigned int arrID[POINTS_PER_BLOCK] = {};
    int curr_arrX[POINTS_PER_BLOCK] = {};
    int curr_arrY[POINTS_PER_BLOCK] = {};
    unsigned int curr_arrID[POINTS_PER_BLOCK] = {};
    struct max
    {
        unsigned int block = 0;
        int value = -1000000000;
        unsigned int idx = 0;
    };
    struct min
    {
        unsigned int block = 0;
        int value = 1000000000;
        unsigned int idx = 0;
    };
    max* MaxX = new max();
    min* MinX = new min();
    max* MaxY = new max();
    min* MinY = new min();

    void reset()
    {
        current_block = 0;
        for (unsigned int i = 0; i < POINTS_PER_BLOCK; i++)
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
    unsigned int last_readable_pos = 0;
    unsigned int last_full_block = 0;
    unsigned int last_incomplete_block = 0;
    bool map_not_full = false;
    bool map_completion_checked = false;
    bool log_active = false;
};


Vectors vectors;
NavMap *nav = new NavMap;
NavMapUtil navutil;
NAV::CommandQueue *nav_ptr, navqueue;      // navqueue
NAV::CommandQueue *avoid_ptr, avoidqueue;  // avoidqueue
DirControl dir;
Chrono chrono;
Chrono chrono1;
NAV::MapCreationInfo mapinfo;
CurrentCommand currcmd;

int current_hdg = 0;
int diff = 0;
unsigned int spd = 0;
int to_rotate = 0;
int target_hdg = 0;
float dst = 0;
float rad = 0;
unsigned int pos_xvector = 0;
unsigned int pos_yvector = 0;

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
        if (nav_ptr->commands[0] == GOFORWARD || nav_ptr->commands[0] == GOBACKWARDS)
        {
            currcmd.id = nav_ptr->commands[0];
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
            unsigned int data0 = nav_ptr->data[0];
            unsigned int data1 = nav_ptr->data[1];
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
            setHeadingToPoint((unsigned int)nav_ptr->data[0], (unsigned int)nav_ptr->data[1]);
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
                int xvec, yvec;
                std::tie(xvec, yvec) = addToVectors(5, current_hdg);
                mapfile = SD.open("/raw_map.bin", "r+");
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
                int hdg_to_maintain = NAVSensors.getHeading();
                std::vector<unsigned int> commands = {GOBACKWARDS, ROTATEFORPIVOT, GOFORWARD};
                int heading_displacement = 0;
                heading_displacement = (dir.getDirectionToTurn() == LEFT) ? -180 : 180;
                // gobackwards, gobackwards, rotateforpivot, goforward, goforward
                std::vector<int> data = {10, hdg_to_maintain, heading_displacement, MAX_CM, AUTO};
                addToCommandQueue(&commands, &data, avoid_ptr);
            }
            else
            {
                // stava già ruotando per arrivare all'heading target (rotateForPivot)
                // robot già fermo
                // usa dir.last_turn_dir perché deve continuare a ruotare nella stessa direzione in cui lo stava già facendo
                std::vector<unsigned int> commands = {ROTATETO, GOFORWARD};
                // rotateto, rotateto, goforward, goforward
                std::vector<int> data = {dir.target_heading, dir.last_turn_dir, MAX_CM, AUTO};
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
        if (vectors.last_dst != dst && (dst > 0.03 || dst < 0.03))
        {
            unsigned int hdg_360 = getHeading360();
            vectors.last_dst = dst;

            // i vettori si alternano sin e cos perché il riferimento dell'heading cambia in base al quadrante

            if (hdg_360 >= 0 && hdg_360 <= 9000) // heading positivo per X e Y -- alto destra -- riferimento 0°
            {
                //if (going_backwards)
                //    hdg_360 = invertHDG(hdg_360);
                rad = radians(hdg_360 / 100);
                vectors.xvector += dst * sin(rad); // positivo per X
                vectors.yvector += dst * cos(rad); // positivo per Y
            }
            else if (hdg_360 > 9000 && hdg_360 <= 18000) // heading positivo per X, negativo per Y -- basso destra -- riferimento 90°
            {
                //if (going_backwards)
                //    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertFromRelativeHDGToRef0(hdg_360, 90);
                rad = radians(hdg_360 / 100);
                vectors.xvector += dst * cos(rad); // positivo per X
                vectors.yvector -= dst * sin(rad); // negativo per Y
            }
            else if (hdg_360 > 18000 && hdg_360 <= 27000) // heading negativo per X e Y -- basso sinistra -- riferimento 180°
            {
                //if (going_backwards)
                //    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertFromRelativeHDGToRef0(hdg_360, 180);
                rad = radians(hdg_360 / 100);
                vectors.xvector -= dst * sin(rad); // negativo per X
                vectors.yvector -= dst * cos(rad); // negativo per Y
            }
            else // heading positivo per Y, negativo per X -- alto sinistra -- riferimento 270°
            {
                //if (going_backwards)
                //    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertFromRelativeHDGToRef0(hdg_360, 270);
                rad = radians(hdg_360 / 100);
                vectors.xvector -= dst * cos(rad); // negativo per X
                vectors.yvector += dst * sin(rad); // positivo per Y
            }

            vectors.current_xcm = (int)vectors.xvector;
            vectors.current_ycm = (int)vectors.yvector;
            if (USE_SD)
            {
                if (vectors.current_xcm != vectors.last_xcm || vectors.current_ycm != vectors.last_ycm)
                {
                    vectors.last_xcm = vectors.current_xcm;
                    vectors.last_ycm = vectors.current_ycm;

                    if ((LOG_MAP || bordermode || navutil.map_not_full) && navutil.log_active)
                    {
                        generic_point.x = vectors.current_xcm;
                        generic_point.y = vectors.current_ycm;

                        if (bordermode)
                        {
                            generic_point.id = 2;
                            mapfile.write((uint8_t*)&generic_point, MAP_POINT_SIZE);
                            //writePoint(&generic_point);
                            if (vectors.xvector < 5 && vectors.xvector > -5 && vectors.yvector < 5 && vectors.yvector > -5 && millis() - bordermode_start_time > 20000)
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

                        NAVCore.println("X", vectors.current_xcm);
                        NAVCore.println("Y", vectors.current_ycm);
                        mapfile.flush();
                    }

                }

/*
                if (!bordermode)
                {
                    if (vectors.abs_intxvector() > 200 && vectors.abs_intyvector() > 200) // 200 / 100
                    {
                        unsigned int pt_dst, pt_id, pt_idx;
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

unsigned int NAV::getTime()
{
    return t3;
}

void NAV::goForward(unsigned int cm, int hdg_to_maintain)
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

void NAV::goBackwards(unsigned int cm, int hdg_to_maintain)
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

void NAV::rotateForDeg(int degs)
{
    /*
            0°
        /       \
    270°           90°
        \       /
           180°
    */

    nav_ptr->busy = true;
    resetMovementVars();
    Rotation.diff_to_rotate = abs(degs);
    int heading_to_reach = degs * 100;
    if (heading_to_reach < 0)
        heading_to_reach = heading_to_reach + 36000; // 36000
    Rotation.start_heading = getHeading360();
    int over_start_heading = Rotation.start_heading + heading_to_reach;
    if (over_start_heading - 36000 < 0)
        heading_target = over_start_heading;
    else
        heading_target = over_start_heading - 36000;

    Rotation.rotating = true;

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
}

void NAV::rotateToDeg(unsigned int heading, unsigned int direction)
{
    nav_ptr->busy = true;
    resetMovementVars();
    Rotation.rotating = true;
    heading_target = heading;
    Rotation.start_heading = getHeading360();
    target_hdg = convertHDGTo180(heading_target);
    unsigned int current_hdg = Rotation.start_heading;
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

    NAVCore.println("Rotating to", target_hdg);
}

void NAV::obstacleDetectedWhileMoving(unsigned int sensor_type, unsigned int sensor_direction)
{
    obstacle_detected = true;
}

void NAV::externalStop()
{
    stop(true);
    resetMovementVars();
    dir = {};
    autorun = false;
    NAVSensors.resetMovementVars();
    robot_moving_x_y = false;
}

unsigned int NAV::getHeading360()
{
    float hdg = NAVSensors.getHeading();
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

float NAV::getVirtualHDG(int heading)
{
    int heading_to_reach = heading * 100;
    if (heading_to_reach < 0)
        heading_to_reach += 36000; // 36000 + 900 (900 = errore)
    //else
        //heading_to_reach -= 650;
    int start_heading_loc = NAVSensors.getHeading();
    if (start_heading_loc < 0)
        start_heading_loc += 36000;
    int over_start_heading = start_heading_loc + heading_to_reach;
    if (over_start_heading - 36000 < 0)
        heading_target = over_start_heading;
    else
        heading_target = over_start_heading - 36000;

    return heading_target;
}

unsigned int NAV::getRotationDirection(int current_hdg, int target_heading)
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

void NAV::obstacleDetectedBeforeMoving(unsigned int sensor_type, unsigned int sensor_direction)
{
    stop();
}

int NAV::convertHDGTo180(unsigned int heading, bool always_positive)
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

unsigned int NAV::convertFromRelativeHDGToRef0(unsigned int hdg, unsigned int reference)
{
    unsigned int return_hdg = 36000 - reference * 100 + hdg;
    if (return_hdg - 36000 > 0)
        return return_hdg - 36000;
    else
        return return_hdg;
}

unsigned int NAV::invertHDG(unsigned int hdg)
{
    unsigned int return_hdg = hdg + 18000;
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

bool NAV::readBlock(unsigned int block, bool open)
{
    if (!USE_SD || nav->current_block == block) return false;

    unsigned int start_position = MAP_BLOCK_SIZE * (block - 1)/* - 1*/;

    if (start_position + MAP_BLOCK_SIZE > navutil.last_readable_pos)
    {
        log_e("SD Target read position greater than last readable position (%d > %d)", start_position + MAP_BLOCK_SIZE, navutil.last_readable_pos);
        return false;
    }

    nav->reset();
    nav->current_block = block;
    unsigned int pos = 0;
    unsigned int data_type = 0;
    unsigned int read_data = 0;
    // reset degli array
    for (unsigned int i = 0; i < (POINTS_PER_BLOCK) * 3; i++)
        map_arr_data[i] = 0;
    unsigned int temp_idx = 0;
    unsigned int data_idx = 0;

    NAVCore.println("Reading", block);
    if (open)
        mapfile = SD.open("/raw_map.bin");
    mapfile.seek(start_position);
    mapfile.read(block_buffer, MAP_BLOCK_SIZE);
    if (open)
        mapfile.close();

    unsigned int point_idx = 0;

    for (int i = 0; i < MAP_BLOCK_SIZE; i++)
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

unsigned int NAV::getCurrentBlock()
{
    bool xblock_found = false;
    bool yblock_found = false;
    unsigned int xblock_idx = 0;
    unsigned int yblock_idx = 0;
    unsigned int counter = 1;
    unsigned int last_block = 0;
    unsigned int read_blocks = 0;
    int int_xvector = vectors.intxvector();
    int int_yvector = vectors.intyvector();

    // se la posizione del robot è già compresa nei valori del blocco attuali
    if (nav->MinX->value < int_xvector && nav->MaxX->value > int_xvector && nav->MinY->value < int_yvector &&nav->MaxY->value > int_yvector)
    {
        for (int i = 0; i < POINTS_PER_BLOCK; i++)
            nav->curr_arrX[i] = nav->arrX[i];
        for (int i = 0; i < POINTS_PER_BLOCK; i++)
            nav->curr_arrY[i] = nav->arrY[i];
        for (int i = 0; i < POINTS_PER_BLOCK; i++)
            nav->curr_arrID[i] = nav->arrID[i];
        return nav->current_block;
    }

    mapfile = SD.open("/raw_map.bin");
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
                    for (int i = 0; i < POINTS_PER_BLOCK; i++)
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
                    for (int i = 0; i < POINTS_PER_BLOCK; i++)
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
        for (int i = 0; i < POINTS_PER_BLOCK; i++)
            nav->curr_arrID[i] = nav->arrID[i];
    }

    return nav->current_block;
}

unsigned int NAV::getCurrentBlockFakeXY(int x, int y)
{
    bool xblock_found = false;
    bool yblock_found = false;
    unsigned int xblock_idx = 0;
    unsigned int yblock_idx = 0;
    unsigned int counter = 1;
    unsigned int last_block = 0;
    unsigned int read_blocks = 0;
    int int_xvector = x;
    int int_yvector = y;

    // se la posizione del robot è già compresa nei valori del blocco attuali
    if (nav->MinX->value < int_xvector && nav->MaxX->value > int_xvector && nav->MinY->value < int_yvector && nav->MaxY->value > int_yvector)
    {
        for (int i = 0; i < 256; i++)
            nav->curr_arrX[i] = nav->arrX[i];
        for (int i = 0; i < 256; i++)
            nav->curr_arrY[i] = nav->arrY[i];
        for (int i = 0; i < 256; i++)
            nav->curr_arrID[i] = nav->arrID[i];
        return nav->current_block;
    }

    mapfile = SD.open("/raw_map.bin");
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
                    for (int i = 0; i < 256; i++)
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
                    for (int i = 0; i < 256; i++)
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
        for (int i = 0; i < 256; i++)
            nav->curr_arrID[i] = nav->arrID[i];
    }

    return nav->current_block;
}

std::tuple<unsigned int, unsigned int, unsigned int> NAV::getClosestPointDst(unsigned int point_type)
{
    unsigned int current_block = getCurrentBlock();

    unsigned int pos_xvector = vectors.abs_intxvector();
    unsigned int pos_yvector = vectors.abs_intyvector();

    unsigned int dst_to_closest_point = 1000000;
    unsigned int closest_point_idx = 0;
    unsigned int second_closest_point_idx = 0;
    for (int i = 0; i < 256; i++)
    {
        if (nav->curr_arrID[i] != 3)
        {
            // 200 == qualsiasi punto
            if ((nav->curr_arrID[i] == point_type && point_type != 200) || point_type == 200)
            {
                int xdiff = 0;
                int xpoint = nav->curr_arrX[i];
                xpoint *= (xpoint < 0) ? -1 : 1;
                xdiff = pos_xvector - xpoint;
                xdiff *= (xdiff < 0) ? -1 : 1;

                int ydiff = 0;
                int ypoint = nav->curr_arrY[i];
                ypoint *= (ypoint < 0) ? -1 : 1;
                ydiff = pos_yvector - ypoint;
                ydiff *= (ydiff < 0) ? -1 : 1;

                unsigned int real_diff = sqrt(pow(xdiff, 2) + pow(ydiff, 2)); // pitagora
                
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

std::tuple<unsigned int, unsigned int, unsigned int, unsigned int> NAV::getClosestPointDstFakeXY(int x, int y, unsigned int point_type)
{
    unsigned int pos_xvector = (unsigned int)abs(x);
    unsigned int pos_yvector = (unsigned int)abs(y);

    unsigned int dst_to_closest_point = 1000000;
    unsigned int closest_point_idx = 0;
    unsigned int second_closest_point_idx = 0;
    for (int i = 0; i < 256; i++)
    {
        if (nav->arrID[i] != 3)
        {
            // 200 == qualsiasi punto
            if ((nav->arrID[i] == point_type && point_type != 200) || point_type == 200)
            {
                int xdiff = 0;
                int xpoint = nav->arrX[i];
                xpoint *= (xpoint < 0) ? -1 : 1;
                xdiff = pos_xvector - xpoint;
                xdiff *= (xdiff < 0) ? -1 : 1;

                int ydiff = 0;
                int ypoint = nav->arrY[i];
                ypoint *= (ypoint < 0) ? -1 : 1;
                ydiff = pos_yvector - ypoint;
                ydiff *= (ydiff < 0) ? -1 : 1;

                unsigned int real_diff = sqrt(pow(xdiff, 2) + pow(ydiff, 2)); // pitagora
                
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
        mapfile = SD.open("/raw_map.bin", "r+");
    bordermode = on_off;
    bordermode_start_time = millis();
}

unsigned int NAV::getSDLastReadablePosition(const char* file)
{
    if (!USE_SD) return 0;
    unsigned int pos = 0;
    int data = 0;
    mapfile = SD.open(file, "r");
    pos = mapfile.size();
    mapfile.close();
    return pos;
}

void NAV::sdspeedtest(unsigned int bytes)
{
    if (!USE_SD) return;
    uint8_t* buffer = new uint8_t[72900];
    unsigned int start_time = millis();
    mapfile = SD.open("/raw_map.bin");
    //for (int i = 0; i < bytes; i++)
    //    mapfile.parseInt();
    mapfile.close();
    Serial.print(F("TIME (72900  bytes): "));
    Serial.println(millis() - start_time);

    delete[] buffer;
}

std::tuple<int, int> NAV::addToVectors(int val, unsigned int hdg)
{
    int xvec = 0;
    int yvec = 0;
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

    return std::make_tuple((int)((xvec + vectors.xvector) * 100), (int)((yvec + vectors.yvector) * 100));
}

std::tuple<int, int> NAV::getVectors(int val, unsigned int hdg)
{
    int xvec = 0;
    int yvec = 0;
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

    return std::make_tuple((int)(xvec * 100), (int)(yvec * 100));
}

unsigned int NAV::getMaxBlock(const char* map, bool fill)
{
    if (!USE_SD) return 0;
    unsigned int last_full_block = 0;
    unsigned int last_incomplete_block = 0;
    unsigned int max_readable_pos = navutil.last_readable_pos;
    unsigned int remainder = max_readable_pos % MAP_BLOCK_SIZE;
    NAVCore.println(F("Max MAP pos"), max_readable_pos);
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
        unsigned int target_position = MAP_BLOCK_SIZE * last_incomplete_block /*- 1; TO CHECK*/;
        unsigned int cursor = max_readable_pos;
        mapfile = SD.open(map, "r+");
        mapfile.seek(cursor);
        generic_point.x = 0;
        generic_point.y = 0;
        generic_point.id = 3;
        while (cursor != target_position)
        {
            mapfile.write((uint8_t*)&generic_point, sizeof(Point));
            cursor = mapfile.position();
        }
        mapfile.close();
        NAVCore.print(F("DONE "));
    }

    navutil.last_full_block = last_full_block;
    navutil.last_incomplete_block = last_incomplete_block;
    if (fill) { NAVCore.println(F("OK")); };

    return last_full_block;
}

std::tuple<unsigned int, unsigned int, int> NAV::getTopPoint(const char* map, unsigned int axis)
{
    unsigned int max_block = getMaxBlock(map);
    unsigned int current_block = 0;
    unsigned int block_idx, p_idx = 0;
    int p_value = 0;
    mapfile = SD.open(map);
    while (current_block < max_block)
    {
        readPointBlock(current_block);
        
        if (axis == X)
        {
            if (p_value < nav->MaxX->value)
            {
                block_idx = nav->MaxX->block;
                p_idx = nav->MaxX->idx;
                p_value = nav->MaxX->value;
            }
        }
        else
        {
            if (p_value < nav->MaxY->value)
            {
                block_idx = nav->MaxY->block;
                p_idx = nav->MaxY->idx;
                p_value = nav->MaxY->value;
            }
        }

        current_block++;
    }
    mapfile.close();

    return std::make_tuple(block_idx, p_idx, p_value);
}

std::tuple<unsigned int, unsigned int, int> NAV::getBottomPoint(const char* map, unsigned int axis)
{
    unsigned int max_block = getMaxBlock(map);
    unsigned int current_block = 0;
    unsigned int block_idx, p_idx = 0;
    int p_value = 1000000000;

    mapfile = SD.open(map);
    while (current_block < max_block)
    {
        readPointBlock(current_block);

        if (axis == X)
        {
            if (p_value > nav->MinX->value)
            {
                block_idx = nav->MinX->block;
                p_idx = nav->MinX->idx;
                p_value = nav->MinX->value;
            }
        }
        else
        {
            if (p_value > nav->MinY->value)
            {
                block_idx = nav->MinY->block;
                p_idx = nav->MinY->idx;
                p_value = nav->MinY->value;
            }
        }

        current_block++;
    }
    mapfile.close();

    return std::make_tuple(block_idx, p_idx, p_value);
}

std::tuple<unsigned int, unsigned int, int> NAV::getTopPointOf(unsigned int block, unsigned int axis)
{
    unsigned int block_idx, p_idx = 0;
    int p_value = 0;
    readBlock(block);
    if (axis == X)
    {
        if (p_value < nav->MaxX->value)
        {
            p_idx = nav->MaxX->idx;
            p_value = nav->MaxX->value;
        }
    }
    else
    {
        if (p_value < nav->MaxY->value)
        {
            p_idx = nav->MaxY->idx;
            p_value = nav->MaxY->value;
        }
    }

    return std::make_tuple(block_idx, p_idx, p_value);
}

std::tuple<unsigned int, unsigned int, int> NAV::getBottomPointOf(unsigned int block, unsigned int axis)
{
    unsigned int block_idx, p_idx = 0;
    int p_value = 0;
    readBlock(block);
    if (axis == X)
    {
        if (p_value > nav->MinX->value)
        {
            p_idx = nav->MinX->idx;
            p_value = nav->MinX->value;
        }
    }
    else
    {
        if (p_value > nav->MinY->value)
        {
            p_idx = nav->MinY->idx;
            p_value = nav->MinY->value;
        }
    }

    return std::make_tuple(block_idx, p_idx, p_value);
}

std::tuple<int, int> NAV::getPointXY(unsigned int block, unsigned int point_idx)
{
    int x = 0;
    int y = 0;
    if (!USE_SD) return std::make_tuple(x, y);
    unsigned int block_position = (block == 1) ? 0 : MAP_BLOCK_SIZE * (block - 1) - 1;
    unsigned int idx_position = point_idx * 22;
    unsigned int seek_position = block_position + idx_position;
    mapfile = SD.open("/raw_map.bin");
    mapfile.seek(seek_position);
    uint8_t buf[7];
    mapfile.read(buf, 7);
    for (unsigned int i = 0; i < 7; i++)
    {
        x *= 10;
        x += buf[i] - '0';
    }
    mapfile.read(buf, 7);
    for (unsigned int i = 0; i < 7; i++)
    {
        y *= 10;
        y += buf[i] - '0';
    }
    mapfile.close();

    return std::make_tuple(x, y);
}

unsigned int NAV::convertFromRef0ToRelativeHDG(unsigned int hdg, unsigned int ref, bool add)
{
    if (add)
        return hdg + ref * 100;
    else
        return 9000 + ref * 100 - hdg;
}

unsigned int NAV::setHeadingToPoint(int target_x, int target_y)
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

    int ydiff = 0;
    int xdiff = 0;
    unsigned int target_heading = 0;
    unsigned int current_heading = getHeading360();
    int int_xvector = vectors.intxvector();
    int int_yvector = vectors.intyvector();
    nav_ptr->busy = true;

    xdiff = abs(vectors.abs_intxvector() - abs(target_x));
    ydiff = abs(vectors.abs_intyvector() - abs(target_y));

    float angle = degrees(atan((float)(xdiff) / (float)(ydiff)));
    if (target_x > int_xvector && target_y > int_yvector) // alto destra
        target_heading = angle * 100;
    else if (target_x > int_xvector && target_y < int_yvector) // basso destra
        target_heading = convertFromRef0ToRelativeHDG((unsigned int)(angle * 100), 90, false);
    else if (target_x < int_xvector && target_y < int_yvector) // basso sinistra
        target_heading = convertFromRef0ToRelativeHDG((unsigned int)(angle * 100), 180, true);
    else if (target_x < int_xvector && target_y > int_yvector) // alto sinistra
        target_heading = convertFromRef0ToRelativeHDG((unsigned int)(angle * 100), 270, false);

    int target_heading_180 = convertHDGTo180(target_heading);

    rotateToDeg(target_heading, getRotationDirection(current_heading, target_heading_180));

    return target_heading;
}

unsigned int NAV::setHeadingToPoint(Point *p)
{
    return setHeadingToPoint(p->x, p->y);
}

unsigned int NAV::getPointDst(int x, int y)
{
    int pos_xvector = abs(vectors.current_xcm);
    int pos_yvector = abs(vectors.current_ycm);

    int xdiff = 0;
    int ydiff = 0;
    xdiff = pos_xvector - abs(x);
    ydiff = pos_yvector - abs(y);

    unsigned int dst = sqrt(pow(abs(xdiff), 2) + pow(abs(ydiff), 2)); // pitagora
    return dst;
}

unsigned int NAV::getPointDst(Point* p)
{
    return getPointDst(p->x, p->y);
}

void NAV::goToPoint(int x, int y, bool precedence)
{
    std::vector<unsigned int> commands = {SETHDGTOPOINT, GOFORWARD};
    std::vector<int> data = {x, y, (int)getPointDst(x, y), NAVSensors.getHeading()};
    addToCommandQueue(&commands, &data, nav_ptr, precedence);
}

void NAV::goToPoint(Point* p, bool precedence)
{
    goToPoint(p->x, p->y);
}

Map mp;

void NAV::scroll()
{
    /*int int_xvector = vectors.intxvector();
    int int_yvector = vectors.intyvector();
    int bot_blk_x, bot_idx_x, bot_pval_x = 0;
    int bot_blk_y, bot_idx_y, bot_pval_y = 0;
    int top_blk_x, top_idx_x, top_pval_x = 0;
    int top_blk_y, top_idx_y, top_pval_y = 0;
    int rightmost_point_blk_x, rightmost_point_idx_x, rightmost_point_val_x = 0;


    std::tie(bot_blk_x, bot_idx_x, bot_pval_x) = getBottomPoint("/raw_map.bin", X); // punto più a sinstra, punto iniziale
    std::tie(bot_blk_y, bot_idx_y, bot_pval_y) = getBottomPoint("/raw_map.bin", Y); // punto più in basso, punto iniziale
    std::tie(top_blk_x, top_idx_x, top_pval_x ) = getTopPoint("/raw_map.bin", X);
    std::tie(top_blk_y, top_idx_y, top_pval_y) = getTopPoint("/raw_map.bin", Y);

    char* chars = new char[80];
    sprintf(chars, "Bottom point: %d, %d - map point info: blockX %d idxX %d blockY %d idxY %d", bot_pval_x, bot_pval_y, bot_blk_x, bot_idx_x, bot_blk_y, bot_idx_y);
    NAVCore.println(chars);
    for (int i = 0; i < 80; i++)
        chars[i] = 0;
    sprintf(chars, "Top point: %d, %d - map point info: blockX %d idxX %d blockY %d idxY %d", top_pval_x, top_pval_y, top_blk_x, top_idx_x, top_blk_y, top_idx_y);
    NAVCore.println(chars);

    delete[] chars;

    unsigned int width = abs(top_pval_x - bot_pval_x);
    unsigned int height = abs(top_pval_y - bot_pval_y);
    generic_point.x = bot_pval_x;
    generic_point.y = bot_pval_y;
    generic_point.id = 1;

    processMap(bot_pval_x, bot_pval_y, top_pval_x, top_pval_y);*/

    //for (int i = 0; i < mapinfo.height_in_squares; i++)
    //{
    //    getMapSquare(i);    
    //}

    //xTaskCreatePinnedToCore(mapperfun, "mapper", 10000, NULL, 5, &mapper, 1);


    /*
    xTaskCreatePinnedToCore(ramsizetask, "ramsizetask", 10000, NULL, 15, &ramSizeTask, 0);
    void* stack = heap_caps_malloc(524288, MALLOC_CAP_SPIRAM);
    xTaskCreatePinnedToCore(astar, "ramsizetask", 524288, NULL, 24, &astartask, 0);
    */

    //processMap(bot_pval_x, bot_pval_y, top_pval_x, top_pval_y);
    commands.push_back(GOTOPOINT);
    command_data.push_back(0);
    command_data.push_back(0);
    addToCommandQueue(nav_ptr);
    //rotateToDeg(0, getRotationDirection(convertHDGTo180(getHeading360()), 0));
    //displayQueue(nav_ptr);
    //removeDuplicatesFromMap(mapinfo.width, mapinfo.height, mapinfo.minx, mapinfo.miny, mapinfo.fill_start_pos);
}

TaskHandle_t pauser;
SemaphoreHandle_t pauseSemaphore;
void (*pauseEvent)(void);

void pauseFunction(void* param)
{
    while (true)
    {
        if (currcmd.id == GOFORWARD || currcmd.id == GOBACKWARDS || *currcmd.isbusy == false)
        {
            if (xSemaphoreTake(pauseSemaphore, portMAX_DELAY))
            {
                NAVSensors.setMotorsStop(); // resetta gli encoder, altrimenti il tempo registrato farebbe sballare le misure
                NAVMotors.stop();
                is_paused = true;
                pauseEvent();
            }
        }

        vTaskDelay(100);
    }
}

void NAV::pause(void (*fun_pause)(void))
{
    xSemaphoreGive(pauseSemaphore);
    pauseEvent = fun_pause;
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

int NAV::convertCharVecToInt(std::vector<char> a)
{
    unsigned int i = 0;
    int num = 0;
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

    unsigned int accessible_points_count = 0;
    mapfile = SD.open("/raw_map.bin");
    for (unsigned int current_block = 0; current_block < navutil.last_full_block; current_block++)
    {
        readPointBlock(current_block);
        for (unsigned int i = 0; i < 256; i++)
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

unsigned int NAV::invert180HDG(int hdg)
{
    return 180 - abs(hdg);
}

unsigned int NAV::getMeanDiff(unsigned int block, unsigned int axis)
{
    readBlock(block);
    unsigned int total_diff = 0;
    for (unsigned int i = 0; i < 255; i++) // fino a 254 perché c'è nav->arrX[i + 1] quindi al ciclo 254 sarebbe 255 (max)
    {
        if (axis == X)
            diff = abs(nav->arrX[i + 1] - nav->arrX[i]);
        else
            diff = abs(nav->arrY[i + 1] - nav->arrY[i]);
        total_diff += diff;
    }
    unsigned int mean_diff = (unsigned int)((float)total_diff / 255);
    return mean_diff;
}

void NAV::addToCommandQueue(unsigned int cmd, std::vector<int> *input_data, NAV::CommandQueue *queue, bool higher_priority)
{
    if (higher_priority)
    {
        queue->commands.emplace(queue->commands.begin(), cmd);
        for (unsigned int i = input_data->size() - 1; i >= 0; i--)
            queue->data.emplace(queue->data.begin(), (*input_data)[i]);
    }
    else
    {
        queue->commands.push_back(cmd);
        for (unsigned int i = 0; i < input_data->size(); i++)
            queue->data.push_back((*input_data)[i]);
    }
}

void NAV::addToCommandQueue(std::vector<unsigned int> *input_cmds, std::vector<int> *input_data, NAV::CommandQueue *queue, bool higher_priority)
{
    if (higher_priority)
    {
        for (unsigned int i = input_cmds->size() - 1; i >= 0; i--)
        {
            queue->commands.emplace(queue->commands.begin(), (*input_cmds)[i]);
            if (i == 0)
                break;
        }
        for (unsigned int i = input_data->size() - 1; i >= 0; i--)
        {
            queue->data.emplace(queue->data.begin(), (*input_data)[i]);
            if (i == 0)
                break;
        }
    }
    else
    {
        for (unsigned int i = 0; i < input_cmds->size(); i++)
            queue->commands.push_back((*input_cmds)[i]);
        for (unsigned int i = 0; i < input_data->size(); i++)
            queue->data.push_back((*input_data)[i]);
    }
}

void NAV::addToCommandQueue(NAV::CommandQueue *queue, bool higher_priority)
{
    if (higher_priority)
    {
        for (unsigned int i = commands.size() - 1; i >= 0; i--)
        {
            queue->commands.emplace(queue->commands.begin(), commands[i]);
            if (i == 0)
                break;
        }
        for (unsigned int i = command_data.size() - 1; i >= 0; i--)
        {
            queue->data.emplace(queue->data.begin(), command_data[i]);
            if (i == 0)
                break;
        }
    }
    else
    {
        for (unsigned int i = 0; i < commands.size(); i++)
            queue->commands.push_back(commands[i]);
        for (unsigned int i = 0; i < command_data.size(); i++)
            queue->data.push_back(command_data[i]);
    }

    commands.clear();
    command_data.clear();
}

void NAV::displayQueue(CommandQueue *queue)
{
    NAVCore.print(F("Command queue contains:"));
    for (unsigned int i = 0; i < queue->commands.size(); i++)
        NAVCore.print(" ", queue->commands[i]);
    NAVCore.println("");
    NAVCore.print(F("Data queue contains:"));
    for (unsigned int i = 0; i < queue->data.size(); i++)
        NAVCore.print(" ", queue->data[i]);
    NAVCore.println("");
}

std::tuple<unsigned int, unsigned int, int, int> NAV::getTopPointWith(int coordinate, unsigned int given_coordinate_axis)
{
    unsigned int current_block = getBlockContaining(coordinate, given_coordinate_axis);
    readBlock(current_block);
    int current_x, current_y = 0;
    int p_x, p_y, max_p_x, max_p_y = 0;
    unsigned int p_dst, last_p_dst, p_id, p_idx, p_block = 0;
    unsigned int max_p_idx, max_p_block = 0;

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
            current_y += 0.5 * ((int)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
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
                current_y -= 0.5 * ((int)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
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
            current_x += 0.5 * ((int)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
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
                current_y -= 0.5 * ((int)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
            }
        }
    }

    return std::make_tuple(max_p_block, max_p_idx, max_p_x, max_p_y);
}

std::tuple<unsigned int, unsigned int, int, int> NAV::getBottomPointWith(int coordinate, unsigned int given_coordinate_axis)
{
    unsigned int current_block = getBlockContaining(coordinate, given_coordinate_axis);
    readBlock(current_block);
    int current_x, current_y = 0;
    int p_x, p_y = 0;
    int max_p_x, max_p_y = 1000000000;
    unsigned int p_dst, last_p_dst, p_id, p_idx, p_block = 0;
    unsigned int max_p_idx, max_p_block = 0;

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
            current_y += 0.5 * ((int)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
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
                current_y -= 0.5 * ((int)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
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
            current_x += 0.5 * ((int)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
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
                current_y -= 0.5 * ((int)p_dst - MAP_POINT_PROXIMITY_DST) + 30;
            }
        }
    }

    return std::make_tuple(max_p_block, max_p_idx, max_p_x, max_p_y);
}

unsigned int NAV::getBlockContaining(int coordinate, unsigned int axis)
{
    unsigned int current_block = 1;
    bool block_found = false;
    mapfile = SD.open("/raw_map.bin");
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

void NAV::rotateForPivot(int degs)
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
    int heading_to_reach = degs * 100;
    if (heading_to_reach < 0)
        heading_to_reach = heading_to_reach + 36000; // 36000
    Rotation.start_heading = getHeading360();
    int over_start_heading = Rotation.start_heading + heading_to_reach;
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

void NAV::clearQueue(CommandQueue *queue, unsigned int items_to_delete)
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
            for (int i = 0; i < items_to_delete; i++)
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

int NAV::getRealAngleDiff(int angle1, int angle2)
{
    diff = angle1 - angle2;
    while (diff <= -18000)
        diff += 36000;
    while (diff > 18000)
        diff -= 36000;
    return diff;
}

void NAV::processMap(int minx, int miny, int maxx, int maxy)
{
    unsigned int fill_start_position = 0;
    unsigned int width = abs(maxx - minx);
    unsigned int height = abs(maxy - miny);

    // adattamento dei limiti della mappa per fare in modo che tutto possa essere diviso in quadrati di lato MAP_SQUARE_WIDTH
    unsigned int w_compensation = (width != MAP_SQUARE_WIDTH) ? std::ceil((float)width / (float)MAP_SQUARE_WIDTH) * MAP_SQUARE_WIDTH - width : 0;
    unsigned int h_compensation = (height != MAP_SQUARE_WIDTH) ? std::ceil((float)height / (float)MAP_SQUARE_WIDTH) * MAP_SQUARE_WIDTH - height : 0;

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

    mapfile = SD.open("/mapinfo.bin", FILEWRITE);
    mapfile.write((uint8_t*)&mapinfo, sizeof(MapCreationInfo));
    mapfile.close();

    Serial.printf("Dims are now %d x %d, compensation: %d, %d\n", width, height, w_compensation, h_compensation);

    generic_point.x = minx;
    generic_point.y = miny;
    generic_point.id = 0;

    // riempimento di tutta la mappa
    mapfile = SD.open("/raw_map.bin", "r+");
    fill_start_position = mapfile.size();
    mapinfo.fill_start_pos = fill_start_position;
    mapfile.seek(fill_start_position);
    char* chars = new char[80];
    sprintf(chars, "Creating map (%d x %d; min %d, %d; max %d, %d)...", width, height, minx, miny, maxx, maxy);
    NAVCore.print(chars);
    for (int i = 0; i < 80; i++)
        chars[i] = '\0';
    chrono1.start();
    for (unsigned int col = 0; col <= width; col++)
    {
        for (unsigned int row = 0; row <= height; row++)
        {
            if (generic_point.x > mapinfo.maxx || generic_point.y > mapinfo.maxy)
                generic_point.id = IGNORE;
            else
                generic_point.id = ACCESSIBLE;
            mapfile.write((uint8_t*)&generic_point, MAP_POINT_SIZE);
            generic_point.y++;
        }
        generic_point.x++;
        generic_point.y = miny;
    }
    mapfile.close();
    unsigned int time = chrono1.getTime();
    float speed = ((float)(width * height * MAP_POINT_SIZE) / ((float)time / 1000)) / 1000;
    sprintf(chars, "Done (%f KB/s, %d ms)", speed, time);
    NAVCore.println(chars);


    // copia dei punti del bordo nella nuova area
    const unsigned int POINT_BUFFER_SIZE = POINTS_PER_BLOCK * 20;   // 51200 B di RAM
    Point mappoint = Point(0, 0);
    Point last_borderpoint = Point(0, 0);
    Point aux_point = Point(0, 0);
    Point fill_p = Point(0, 0);
    unsigned int aux_position = 0;
    int dstx = 0;
    int dsty = 0;
    int climb_rate = 0;
    int climb_freq = 0;
    int xidx = 0;
    bool go_right = false;
    bool go_up = false;
    unsigned int fill_p_position = 0;

    unsigned int position_idx = 0;
    unsigned int* positions_to_overwrite = new unsigned int[POINT_BUFFER_SIZE];
    Point* pts = new Point[POINT_BUFFER_SIZE];
    for (int i = 0; i < POINT_BUFFER_SIZE; i++)
        positions_to_overwrite[i] = 0;

    NAVCore.print("Removing duplicate fill points... ");
    mapfile = SD.open("/raw_map.bin", "r");
    chrono.start();
    for (int block = 0; block < fill_start_position / MAP_BLOCK_SIZE; block++)
    {
        readPointBlock(block);
        for (int map_point_idx = 0; map_point_idx < POINTS_PER_BLOCK; map_point_idx++)  // il fill start indica la fine della mappa fatta dal robot
        {
            mappoint = nav->points[map_point_idx];
            if (mappoint.id == 3)  
                continue;

            fill_p_position = getPointPositionInSD(mappoint, true);

            mapfile.seek(fill_p_position);
            mapfile.read((uint8_t*)&fill_p, MAP_POINT_SIZE);

            if (mappoint.id == BORDER)  // riempimento buchi nel bordo
            {
                dstx = abs(mappoint.x - last_borderpoint.x);
                dsty = abs(mappoint.y - last_borderpoint.y);
                if (dstx > 1 || dsty > 1)
                {
                    xidx = 0;
                    aux_point.x = last_borderpoint.x;
                    aux_point.y = last_borderpoint.y;

                    if (mappoint.x > last_borderpoint.x)
                        go_right = true;
                    else
                        go_right = false;
                    if (mappoint.y > last_borderpoint.y)
                        go_up = true;
                    else
                        go_up = false;

                    if (dstx == 0)
                        dstx = 1;
                    if (dsty == 0)
                        dsty = 1;

                    climb_rate = dsty / dstx;
                    climb_freq = dstx / dsty;

                    if (climb_rate == 0)
                        climb_rate = 1;
                    if (climb_freq == 0)
                        climb_freq = 1;

                    for (int i = 0; i < dstx; i++)
                    {
                        if ((aux_point.x == mappoint.x) && (aux_point.y == mappoint.y))
                            break;
                        if (go_right)
                            aux_point.x++;
                        else
                            aux_point.x--;
                        xidx++;
                        if (xidx == climb_freq)
                        {
                            for (int j = 0; j < climb_rate; j++)
                            {
                                if ((aux_point.x == mappoint.x) && (aux_point.y == mappoint.y))
                                    break;

                                if (go_up)
                                    aux_point.y++;
                                else
                                    aux_point.y--;

                                aux_position = getPointPositionInSD(aux_point, true);
                                positions_to_overwrite[position_idx] = aux_position;
                                pts[position_idx].x = aux_point.x;
                                pts[position_idx].y = aux_point.y;
                                pts[position_idx].id = BORDER;
                                position_idx++;
                            }
                        }
                        else
                        {
                            aux_position = getPointPositionInSD(aux_point, true);
                            positions_to_overwrite[position_idx] = aux_position;
                            pts[position_idx].x = aux_point.x;
                            pts[position_idx].y = aux_point.y;
                            pts[position_idx].id = BORDER;
                            position_idx++;
                        }
                    }
                }

                last_borderpoint = mappoint;
            }

            if ((mappoint.x == fill_p.x) && (mappoint.y == fill_p.y))
            {
                positions_to_overwrite[position_idx] = fill_p_position;
                pts[position_idx].x = mappoint.x;
                pts[position_idx].y = mappoint.y;
                pts[position_idx].id = mappoint.id;
                position_idx++;
            }

            if (position_idx == POINT_BUFFER_SIZE)
            {
                // deve scrivere i dati sull'SD
                mapfile.close();
                mapfile = SD.open("/raw_map.bin", "r+");
                for (int i = 0; i < position_idx; i++)
                {
                    mapfile.seek(positions_to_overwrite[i]);
                    mapfile.write((uint8_t*)&pts[i], MAP_POINT_SIZE);
                }
                position_idx = 0;
                mapfile.close();
                mapfile = SD.open("/raw_map.bin", "r");
            }
        }
    }

    // se non ha già scritto i dati sull'SD (cioè che non c'erano oltre POINT_BUFFER_SIZE punti da scrivere), li scrive ora
    if (position_idx != 0)
    {
        if (position_idx == POINT_BUFFER_SIZE)
            position_idx -= 1;
        mapfile.close();
        mapfile = SD.open("/raw_map.bin", "r+");
        for (int i = 0; i < position_idx; i++)
        {
            mapfile.seek(positions_to_overwrite[i]);
            mapfile.write((uint8_t*)&pts[i], MAP_POINT_SIZE);
        }
        position_idx = 0;
    }
    
    mapfile.close();

    delete[] positions_to_overwrite;
    delete[] pts;

    for (int i = 0; i < 80; i++)
        chars[i] = '\0';

    sprintf(chars, "Done (%d ms)", chrono.getTime());
    NAVCore.println(chars);


    // copia della mappa pulita in un nuovo file
    Point* points_to_write = new Point[POINT_BUFFER_SIZE];
    unsigned int last_pos = getSDLastReadablePosition("/raw_map.bin");
    unsigned int points_to_read = (last_pos - fill_start_position) / MAP_POINT_SIZE;
    unsigned int points_in_arr = 0;
    unsigned int new_map_pos = 0;
    unsigned int seek_pos = 0;
    bool accessible_area = false;
    int area_x = 26121256;
    NAVCore.print("Writing processed map to file... ");
    mapfile = SD.open("/raw_map.bin", "r");
    chrono.start();
    for (int block = 0; block < int(std::ceil((float)points_to_read / POINT_BUFFER_SIZE)); block++)
    {
        for (int i = 0; i < POINT_BUFFER_SIZE; i++)
        {
            if (block * POINT_BUFFER_SIZE + i > points_to_read)
                break;

            seek_pos = fill_start_position + block * (POINT_BUFFER_SIZE * MAP_POINT_SIZE) + i * MAP_POINT_SIZE;
            mapfile.seek(seek_pos);
            mapfile.read((uint8_t*)&points_to_write[i], MAP_POINT_SIZE);
            points_in_arr = i;
        }

        generic_point.x = minx;
        generic_point.y = miny;

        mapfile.close();
        mapfile = SD.open("/map.bin", "r+");
        mapfile.seek(new_map_pos);
        for (unsigned int i = 0; i < points_in_arr + 1; i++)
            mapfile.write((uint8_t*)&points_to_write[i], MAP_POINT_SIZE);
        new_map_pos = mapfile.position();
        mapfile.close();
        mapfile = SD.open("/raw_map.bin", "r");
    }
    mapfile.close();

    delete[] points_to_write;

    for (int i = 0; i < 80; i++)
        chars[i] = '\0';
    
    sprintf(chars, "Done (%d ms)", chrono.getTime());
    NAVCore.println(chars);

    delete[] chars;

    //xTaskCreatePinnedToCore(mapperfun, "mapper", 10000, NULL, 5, &mapper, 1);

    //removeDuplicatesFromMap(width, height, minx, miny, fill_start_position);
}

int NAV::charArrToInt(uint8_t* arr, unsigned int len)
{
    unsigned int power = 1;
    int res = 0;

    for (int i = len - 1; i >= 0; i--)
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

void NAV::resetArray(uint8_t* arr, unsigned int len)
{
    for (int i = 0; i < len; i++)
        arr[i] = 0;
}

bool NAV::readPointBlock(unsigned int block)
{
    unsigned int start_position = MAP_BLOCK_SIZE * block;
    if (!USE_SD || nav->current_block == block) return false;
    if (start_position + MAP_BLOCK_SIZE > navutil.last_readable_pos)
    {
        log_e("SD Target read position greater than last readable position (%d > %d)", start_position + MAP_BLOCK_SIZE, navutil.last_readable_pos);
        return false;
    }

    nav->current_block = block;

    mapfile.seek(start_position);
    for (int i = 0; i < POINTS_PER_BLOCK; i++)
    {
        mapfile.read((uint8_t*)&nav->points[i], sizeof(Point));
        //readPoint(i * MAP_POINT_SIZE + start_position, &nav->points[i]);
        if (nav->points[i].x > nav->MaxX->value)
            nav->MaxX->value = nav->points[i].x;
        if (nav->points[i].y > nav->MaxY->value)
            nav->MaxY->value = nav->points[i].y;
        if (nav->points[i].x < nav->MinX->value)
            nav->MinX->value = nav->points[i].x;
        if (nav->points[i].y < nav->MinY->value)
            nav->MinY->value = nav->points[i].y;
    }

    return true;
}

unsigned int NAV::getPointPositionInSD(Point p, bool is_fill)
{
    if (is_fill)
        return mapinfo.fill_start_pos + ((mapinfo.height * (abs(p.x - mapinfo.minx)) + abs(p.y - mapinfo.miny) + abs(p.x - mapinfo.minx)) * MAP_POINT_SIZE);
    else
        return (mapinfo.height * (abs(p.x - mapinfo.minx)) + abs(p.y - mapinfo.miny) + abs(p.x - mapinfo.minx)) * MAP_POINT_SIZE;
}

void NAV::getMapSquare(unsigned int idx)
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

    unsigned int starting_point_position = getPointPositionInSD(p);
    unsigned int starting_x = p.x;
    unsigned int starting_y = p.y;
    unsigned int id = 0;

    // non uint perché GraphType ha int
    psvec<Point> obstacles;

    generic_point.id = OBSTACLE;
    mapfile = SD.open("/map.bin", FILEREAD);
    for (unsigned int col = 0; col < MAP_SQUARE_WIDTH; col++)
    {
        p.x += 1;
        for (unsigned int row = 0; row < MAP_SQUARE_WIDTH; row++)
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

    //for (unsigned int i = 0; i < MAP_SQUARE_WIDTH; i++)
        //Serial.printf("(%d; %d)\n", obstacles[i].x, obstacles[i].y);

    //Serial.printf("1 Heap: %d --- PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());
    Point start = Point(31, 0);
    Point end = Point(0, 31);

    Map mp;
    psvec<Point> path = mp.a_star(&obstacles, start, end);

    int sas[32][32] =
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

    //for (unsigned int i = 0; i < obstacles.size(); i++)
    //    sus[obstacles[i].x][obstacles[i].y] = '1';

    //for (int i = 0; i < 32; i++)
    //{
    //    for (int j = 0; j < 32; j++)
    //    {
    //        Serial.printf("%c ", sus[j][i]);
    //    }
    //    Serial.printf("\n");
    //}
    //Serial.printf("\nBECOMES \\/\\/\\/\\/\\/\n");

    for (int i = 0; i < path.size(); i++)
    {
        int x = path[i].x;
        int y = path[i].y;
        sas[x][y] = '.';
    }

    sas[start.x][start.y] = '+';
    sas[end.x][end.y] = '-';

    //for (int i = 0; i < 32; i++)
    //{
    //    for (int j = 0; j < 32; j++)
    //    {
    //        Serial.printf("%c ", sus[j][i]);
    //    }
    //    Serial.printf("\n");
    //}
    //Serial.printf("\n\n");

    log_w("2 Heap: %d --- PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());

    /*int x = (idx % mapinfo.height_in_squares != 0);
    for (unsigned int i = 0; i < dim; i++)
    {
        for (unsigned int j = 0; j < dim; j++)
        {
            getPointPositionInSD(Point(dim * dim));
        }
    }*/
}

unsigned int NAV::readPointID(Point p)
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

void NAV::begin()
{
    nav_ptr = &navqueue;
    avoid_ptr = &avoidqueue;

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

    //navutil.last_readable_pos = getSDLastReadablePosition("/raw_map.bin");
    //getMaxBlock("/raw_map.bin", true);

    navutil.last_readable_pos = getSDLastReadablePosition("/raw_map.bin");
    NAVCore.println("Last full block is", navutil.last_full_block);
    //checkMapCompletion();
    //readBlock(1);
    readPointBlock(0);
    getMapInfo();

    // parte della pausa del robot
    currcmd.isbusy = &(nav_ptr->busy);
    pauseSemaphore = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(pauseFunction, "pausefunction", 4096, NULL, 4, &pauser, 0);
}
