#include <vector>
#include <tuple>
#include <WebSerial.h>
#include <SD.h>
#include <Navigation.h>
#include <Sensors.h>
#include <Motors.h>
#include <Mux.h>
#include <Core.h>
#include <SETTINGS.h>

#define MOSI 42
#define MISO 41
#define SCLK 40
#define CS 39

#define MAP_DATA_SIZE 7                         // dimensione dati x y in byte
#define MAP_POINT_SIZE (MAP_DATA_SIZE * 2 + 1)  // 15
#define MAP_BLOCK_SIZE (MAP_POINT_SIZE * 256)     // (MAP_DATA_SIZE * 2 + 1) * 256, byte (struttura: 000000000000000 (sarebbe 0000000,0000000,0,))
#define POINTS_PER_BLOCK 256

// variabili globali così non si frammenta la memoria ogni volta che si chiama NAV::readBlock, risparmiando anche tempo
// vengono usati vettori e buffer allocato dinamicamente perché si allocano sull'heap ed è meglio se sono molto grandi
// buffer non può essere un vettore perché SD::read accetta solo array
std::vector<int32_t> map_arr_data((POINTS_PER_BLOCK) * 3);  // ci sono 768 dati in 256 punti (256 * 3 (x, y, ID)) o 2816 byte di punti
uint8_t* block_buffer = new uint8_t[MAP_BLOCK_SIZE];        // messo nell'heap così è più veloce
std::vector<char> temp_data(256);                           // convertCharArrToInt accetta solo array da 256 elementi
uint8_t raw_point[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
NAV::Point generic_point = NAV::Point();

#define GOFORWARD 1             // --- 2 argomenti ---
#define GOBACKWARDS 2           // --- 2 argomenti ---
#define ROTATETO 3              // --- 2 argomenti ---
#define ROTATEFOR 4             // --- 1 argomento ---
#define GOTOPOINT 5             // --- 2 argomenti - AGGIUNGE DUE COMANDI ALLA CODA ---
#define SETHDGTOPOINT 6         // --- 2 argomenti ---
#define ROTATEFORPIVOT 7        // --- 1 argomento ---

//SPIClass *spi = new SPIClass(FSPI);
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

const uint8_t zeroes[7] = {48, 48, 48, 48, 48, 48, 48};
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

class Chrono
{
    private:
        uint32_t start_time = 0;
    public:
        void start() { start_time = millis(); }
        uint32_t getTime() { return millis() - start_time; }
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

class Vectors
{
    public:
        float xvector = 0;
        float yvector = 0;
        float last_dst = 0;
        int32_t last_xcm = 0;
        int32_t current_xcm = 0;
        int32_t last_ycm = 0;
        int32_t current_ycm = 0;

        int32_t intxvector() { return (int32_t)(xvector * 100); };
        int32_t intyvector() { return (int32_t)(yvector * 100); };
        int32_t abs_intxvector() { return (uint32_t)(abs((int32_t)xvector * 100)); };
        int32_t abs_intyvector() { return (uint32_t)(abs((int32_t)yvector * 100)); };
};

struct
{
    bool active = false;
    bool *var_ptr;
    char dir = false;
    bool condition = false;
    bool done = false;
} RotateUntil;

struct
{
    bool was_going_forward = false;
    bool was_going_backwards = false;
} pausevars;

struct NavMap
{
    uint32_t current_block = 0;
    NAV::Point* points = new NAV::Point[POINTS_PER_BLOCK];
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

    void reset()
    {
        current_block = 0;
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

Vectors vectors;
NavMap *nav = new NavMap;
NavMapUtil navutil;
NAV::CommandQueue *nav_ptr, navqueue;      // navqueue
NAV::CommandQueue *avoid_ptr, avoidqueue;  // avoidqueue
DirControl dir;
Chrono chrono;

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
        if (nav_ptr->commands[0] == GOFORWARD || nav_ptr->commands[0] == GOBACKWARDS)
        {
            if (nav_ptr->commands[0] == GOFORWARD)
                goForward(nav_ptr->data[0], nav_ptr->data[1]); // non si possono usare valori default, sono obbligatori valori noti
            else
                goBackwards(nav_ptr->data[0], nav_ptr->data[1]);
            nav_ptr->commands.erase(nav_ptr->commands.begin()); // elimina il comando in esecuzione
            nav_ptr->data.erase(nav_ptr->data.begin(), nav_ptr->data.begin() + 2);
            nav_ptr->busy = true;
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
            pausevars.was_going_forward = false;
            gone_forward = true;
            distance_target_reached = true;
            distance_target = 0;
        }
    }

    if (going_backwards)
    {
        distance_traveled = NAVSensors.getTraveledDistance();
        NAVCore.println("trav dst", distance_traveled);

        if (abs(distance_traveled) >= distance_target && distance_traveled != 0 && distance_target != 0)
        {
            stop(false);
            nav_ptr->busy = false;
            avoid_ptr->busy = false;
            pausevars.was_going_backwards = false;
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
        pausevars.was_going_forward = false;

        stop();
        current_hdg = getHeading360();
        if (USE_SD)
        {
            if (!bordermode && LOG_OBSTACLES_TO_MAP && navutil.log_active)
            {
                int32_t xvec, yvec;
                std::tie(xvec, yvec) = addToVectors(5, current_hdg);
                mapfile = SD.open("/MAP.txt", FILE_APPEND);
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
        if (current_hdg > heading_target - 50 && current_hdg < heading_target + 50)
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
            spd = 100 + 45 * ((120 * (-2 * to_rotate * pow(diff, 2) + 2 * pow(to_rotate, 2) * diff)) /
            pow(2 * pow(diff, 2) - 2 * to_rotate * diff + pow(to_rotate, 2), 2));
            if (dir.is_pivoting)
            {
                if (dir.direction == LEFT)
                    NAVMotors.setSpeed(spd, RIGHT);
                else
                    NAVMotors.setSpeed(spd, LEFT);
            }
            else
                NAVMotors.setSpeed(spd, BOTH);
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
            uint32_t hdg_360 = getHeading360();
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

            if (USE_SD)
            {
                vectors.current_xcm = (int32_t)vectors.xvector;
                vectors.current_ycm = (int32_t)vectors.yvector;
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
                            writePoint(generic_point);
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
                            writePoint(generic_point);
                        }
                        navutil.last_readable_pos += mapfile.position();

                        NAVCore.println("X", vectors.current_xcm);
                        NAVCore.println("Y", vectors.current_ycm);
                        mapfile.flush();
                    }

                }

                if (!bordermode)
                {
                    if (vectors.abs_intxvector() > 200 && vectors.abs_intyvector() > 200) // 200 / 100
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
    pausevars.was_going_forward = true;
    NAVSensors.resetTraveledDistance();
    NAVMotors.forward();
    distance_target = cm;
    going_forward = true;
    robot_moving_x_y = true;
    NAVSensors.enablePositionEncoders();
}

void NAV::goBackwards(uint32_t cm, int32_t hdg_to_maintain)
{
    resetMovementVars();
    dir.direction = BCK;
    nav_ptr->busy = true;
    pausevars.was_going_forward = true;
    NAVSensors.resetTraveledDistance();
    NAVMotors.backwards();
    distance_target = cm;
    going_backwards = true;
    robot_moving_x_y = true;
    NAVSensors.enablePositionEncoders();
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
        NAVMotors.left();
        dir.direction = LEFT;
    }
    else
    {
        NAVMotors.right();
        dir.direction = RIGHT;
    }
}

void NAV::rotateToDeg(uint32_t heading, uint32_t direction)
{
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
}

void NAV::obstacleDetectedWhileMoving(uint32_t sensor_type, uint32_t sensor_direction)
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

uint32_t NAV::getHeading360()
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
    pausevars.was_going_backwards = false;
    pausevars.was_going_forward = false;
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

uint32_t NAV::getRotationDirection(uint32_t current_hdg, int32_t target_heading)
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
    pausevars.was_going_backwards = false;
    pausevars.was_going_forward = false;
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
    mapfile = SD.open(path, FILE_WRITE);
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
    for (uint32_t i = 0; i < MAP_DATA_SIZE; i++)
        temp_data[i] = 0;
    temp_data[MAP_DATA_SIZE] = '\0';
    uint32_t temp_idx = 0;
    uint32_t data_idx = 0;

    NAVCore.println("Reading", block);
    if (open)
        mapfile = SD.open("/raw_map.txt");
    mapfile.seek(start_position);
    mapfile.read(block_buffer, MAP_BLOCK_SIZE);
    if (open)
        mapfile.close();

    uint32_t point_idx = 0;

    for (int i = 0; i < MAP_BLOCK_SIZE; i++)
    {
        if (data_type < 2)                                          // coordinate
        {
            if (block_buffer[i] == 45)                                    // ASCII trattino
                temp_data[temp_idx] = '-';
            else
                temp_data[temp_idx] = block_buffer[i];
            if (temp_idx == MAP_DATA_SIZE - 1)
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
    int32_t int_xvector = vectors.intxvector();
    int32_t int_yvector = vectors.intyvector();

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

    mapfile = SD.open("/MAP.txt");
    while (!xblock_found || !yblock_found)
    {
        if (counter != last_block)
        {
            if (read_blocks == 2)
                pause();
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
        for (int i = 0; i < 256; i++)
            nav->curr_arrX[i] = nav->arrX[i];
        for (int i = 0; i < 256; i++)
            nav->curr_arrY[i] = nav->arrY[i];
        for (int i = 0; i < 256; i++)
            nav->curr_arrID[i] = nav->arrID[i];
        return nav->current_block;
    }

    mapfile = SD.open("/MAP.txt");
    while (!xblock_found || !yblock_found)
    {
        if (counter != last_block)
        {
            if (read_blocks == 2)
                pause();
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

std::tuple<uint32_t, uint32_t, uint32_t> NAV::getClosestPointDst(uint32_t point_type)
{
    uint32_t current_block = getCurrentBlock();

    uint32_t pos_xvector = vectors.abs_intxvector();
    uint32_t pos_yvector = vectors.abs_intyvector();

    uint32_t dst_to_closest_point = 1000000;
    uint32_t closest_point_idx = 0;
    uint32_t second_closest_point_idx = 0;
    for (int i = 0; i < 256; i++)
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
    for (int i = 0; i < 256; i++)
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
        mapfile = SD.open("/raw_map.txt", FILE_APPEND);
    bordermode = on_off;
    bordermode_start_time = millis();
}

uint32_t NAV::getSDLastReadablePosition(const char* file)
{
    if (!USE_SD) return 0;
    uint32_t pos = 0;
    int32_t data = 0;
    mapfile = SD.open(file);
    pos = mapfile.size();
    mapfile.close();

    return pos;
}

void NAV::sdspeedtest(uint32_t bytes)
{
    if (!USE_SD) return;
    uint8_t* buffer = new uint8_t[72900];
    uint32_t start_time = millis();
    mapfile = SD.open("/MAP.txt");
    //for (int i = 0; i < bytes; i++)
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

    return std::make_tuple((int32_t)((xvec + vectors.xvector) * 100), (int32_t)((yvec + vectors.yvector) * 100));
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
        uint32_t target_position = MAP_BLOCK_SIZE * last_incomplete_block /*- 1; TO CHECK*/;
        uint32_t cursor = max_readable_pos;
        mapfile = SD.open(map, FILE_APPEND);
        mapfile.seek(cursor);
        uint8_t null_point[15] {'0','0','0','0','0','0','0','0','0','0','0','0','0','0','3'};
        while (cursor != target_position)
        {
            mapfile.write(null_point, 15); // punto da ignorare (3)
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

std::tuple<uint32_t, uint32_t, int32_t> NAV::getTopPoint(const char* map, uint32_t axis)
{
    uint32_t max_block = getMaxBlock(map);
    uint32_t current_block = 1;
    uint32_t block_idx, p_idx = 0;
    int32_t p_value = 0;
    mapfile = SD.open(map);
    while (current_block <= max_block)
    {
        readBlock(current_block, false);
        
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

std::tuple<uint32_t, uint32_t, int32_t> NAV::getBottomPoint(const char* map, uint32_t axis)
{
    uint32_t max_block = getMaxBlock(map);
    uint32_t current_block = 1;
    uint32_t block_idx, p_idx = 0;
    int32_t p_value = 1000000000;

    mapfile = SD.open(map);
    while (current_block <= max_block)
    {
        readBlock(current_block, false);

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

std::tuple<uint32_t, uint32_t, int32_t> NAV::getTopPointOf(uint32_t block, uint32_t axis)
{
    uint32_t block_idx, p_idx = 0;
    int32_t p_value = 0;
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

std::tuple<uint32_t, uint32_t, int32_t> NAV::getBottomPointOf(uint32_t block, uint32_t axis)
{
    uint32_t block_idx, p_idx = 0;
    int32_t p_value = 0;
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

std::tuple<int32_t, int32_t> NAV::getPointXY(uint32_t block, uint32_t point_idx)
{
    int32_t x = 0;
    int32_t y = 0;
    if (!USE_SD) return std::make_tuple(x, y);
    uint32_t block_position = (block == 1) ? 0 : MAP_BLOCK_SIZE * (block - 1) - 1;
    uint32_t idx_position = point_idx * 22;
    uint32_t seek_position = block_position + idx_position;
    mapfile = SD.open("/MAP.txt");
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
    int32_t xpoint = target_x;
    int32_t ypoint = target_y;
    uint32_t target_heading = 0;
    uint32_t current_heading = getHeading360();
    uint32_t pos_xvector = vectors.abs_intxvector();
    uint32_t pos_yvector = vectors.abs_intyvector();
    int32_t int_xvector = vectors.intxvector();
    int32_t int_yvector = vectors.intyvector();
    nav_ptr->busy = true;

    xpoint = abs(xpoint);
    xdiff = pos_xvector - xpoint;
    xdiff = abs(xdiff);

    ypoint = abs(ypoint);
    ydiff = pos_yvector - ypoint;
    ydiff = abs(ydiff);

    float angle = degrees(atan((float)(xdiff) / (float)(ydiff)));
    if (target_x > int_xvector && target_y > int_yvector) // alto destra
        target_heading = angle;
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

uint32_t NAV::setHeadingToPoint(uint32_t block, uint32_t point_idx)
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

    if (!USE_SD) return 0;
    uint32_t seek_position = ((block == 1) ? 0 : MAP_BLOCK_SIZE * (block - 1) - 1) + point_idx * 22;
    int32_t px = 0;
    int32_t py = 0;
    int32_t xdiff = 0;
    int32_t ydiff = 0;
    uint32_t target_heading = 0;
    uint32_t current_heading = getHeading360();
    uint32_t pos_xvector = vectors.abs_intxvector();
    uint32_t pos_yvector = vectors.abs_intyvector();
    int32_t int_xvector = vectors.intxvector();
    int32_t int_yvector = vectors.intyvector();
    nav_ptr->busy = true;

    mapfile = SD.open("/MAP.txt");
    mapfile.seek(seek_position);
    uint8_t buf[7];
    mapfile.read(buf, 7);
    for (uint32_t i = 0; i < 7; i++)
    {
        px *= 10;
        px += buf[i] - '0';
    }
    mapfile.read(buf, 7);
    for (uint32_t i = 0; i < 7; i++)
    {
        py *= 10;
        py += buf[i] - '0';
    }
    mapfile.close();

    xdiff = pos_xvector - abs(px);
    ydiff = pos_yvector - abs(py);

    float angle = degrees(atan((float)(abs(xdiff)) / (float)(abs(ydiff))));
    if (px > int_xvector && py > int_yvector) // alto destra
        target_heading = angle;
    else if (px > int_xvector && py < int_yvector) // basso destra
        target_heading = convertFromRef0ToRelativeHDG((uint32_t)(angle * 100), 90, false);
    else if (px < int_xvector && py < int_yvector) // basso sinistra
        target_heading = convertFromRef0ToRelativeHDG((uint32_t)(angle * 100), 180, true);
    else if (px < int_xvector && py > int_yvector) // alto sinistra
        target_heading = convertFromRef0ToRelativeHDG((uint32_t)(angle * 100), 270, false);

    int32_t target_heading_180 = convertHDGTo180(target_heading);
    rotateToDeg(target_heading, getRotationDirection(current_heading, target_heading_180));

    return target_heading;
}

uint32_t NAV::getPointDst(uint32_t block, uint32_t point_idx)
{
    if (!USE_SD) return 0;
    uint32_t seek_position = ((block == 1) ? 0 : MAP_BLOCK_SIZE * (block - 1) - 1) + point_idx * 22;
    int32_t px = 0;
    int32_t py = 0;
    mapfile = SD.open("/MAP.txt");
    mapfile.seek(seek_position);
    uint8_t buf[7];
    mapfile.read(buf, 7);
    for (uint32_t i = 0; i < 7; i++)
    {
        px *= 10;
        px += buf[i] - '0';
    }
    mapfile.read(buf, 7);
    for (uint32_t i = 0; i < 7; i++)
    {
        py *= 10;
        py += buf[i] - '0';
    }
    mapfile.close();
    uint32_t pos_xvector = (uint32_t)abs(vectors.xvector * 100);
    uint32_t pos_yvector = (uint32_t)abs(vectors.yvector * 100);

    int32_t xdiff = 0;
    int32_t ydiff = 0;
    xdiff = (int32_t)pos_xvector - (int32_t)abs(px);
    ydiff = (int32_t)pos_yvector - (int32_t)abs(py);

    uint32_t dst = sqrt(pow(abs(xdiff), 2) + pow(abs(ydiff), 2)); // pitagora
    return dst;
}

uint32_t NAV::getPointDst(int32_t x, int32_t y)
{
    uint32_t pos_xvector = (uint32_t)abs(vectors.xvector * 100);
    uint32_t pos_yvector = (uint32_t)abs(vectors.yvector * 100);

    int32_t xdiff = 0;
    int32_t ydiff = 0;
    xdiff = (int32_t)pos_xvector - (int32_t)abs(x);
    ydiff = (int32_t)pos_yvector - (int32_t)abs(y);

    uint32_t dst = sqrt(pow(abs(xdiff), 2) + pow(abs(ydiff), 2)); // pitagora
    return dst;
}

void NAV::goToPoint(int32_t x, int32_t y)
{
    setHeadingToPoint(x, y);
    getPointDst(x, y);
}

void NAV::goToPoint(uint32_t block, uint32_t point_idx)
{
    std::vector<uint32_t> commands = {SETHDGTOPOINT, GOFORWARD};
    std::vector<int32_t> data = {(int32_t)block, (int32_t)point_idx, (int32_t)(getPointDst(block, point_idx) / 100), NAVSensors.getHeading()};
    addToCommandQueue(&commands, &data, nav_ptr, true);
}

void NAV::scroll()
{
    int32_t int_xvector = vectors.intxvector();
    int32_t int_yvector = vectors.intyvector();
    int32_t bot_blk_x, bot_idx_x, bot_pval_x = 0;
    int32_t bot_blk_y, bot_idx_y, bot_pval_y = 0;
    int32_t top_blk_x, top_idx_x, top_pval_x = 0;
    int32_t top_blk_y, top_idx_y, top_pval_y = 0;
    int32_t rightmost_point_blk_x, rightmost_point_idx_x, rightmost_point_val_x = 0;

    std::tie(bot_blk_x, bot_idx_x, bot_pval_x) = getBottomPoint("/raw_map.txt", X); // punto più a sinstra, punto iniziale
    std::tie(bot_blk_y, bot_idx_y, bot_pval_y) = getBottomPoint("/raw_map.txt", Y); // punto più in basso, punto iniziale
    std::tie(top_blk_x, top_idx_x, top_pval_x ) = getTopPoint("/raw_map.txt", X);
    std::tie(top_blk_y, top_idx_y, top_pval_y) = getTopPoint("/raw_map.txt", Y);

    Serial.printf("Bottom point: %d, %d - map point info: blockX %d idxX %d blockY %d idxY %d\n", bot_pval_x, bot_pval_y, bot_blk_x, bot_idx_x, bot_blk_y, bot_idx_y);
    Serial.printf("Top point: %d, %d - map point info: blockX %d idxX %d blockY %d idxY %d\n", top_pval_x, top_pval_y, top_blk_x, top_idx_x, top_blk_y, top_idx_y);

    fillMap(bot_pval_x, bot_pval_y, top_pval_x, top_pval_y);
}

void NAV::pause()
{
    if (going_forward)
        pausevars.was_going_forward = true;
    else if (going_backwards)
        pausevars.was_going_backwards = true;
    NAVMotors.stop();
}

void NAV::resume()
{
    if (pausevars.was_going_forward)
    {
        pausevars.was_going_forward = false;
        NAVMotors.forward();
    }
    else if (pausevars.was_going_backwards)
    {
        pausevars.was_going_backwards = false;
        NAVMotors.backwards();
    }
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
    mapfile = SD.open("/MAP.txt");
    for (uint32_t current_block = 1; current_block <= navutil.last_full_block; current_block++)
    {
        readBlock(current_block, false);
        for (uint32_t i = 0; i < 256; i++)
        {
            if (nav->arrID[i] == ACCESSIBLE)
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
    uint32_t mean_diff = uint32_t((float)total_diff / 255);
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

void NAV::displayQueue(CommandQueue *queue)
{
    WebSerial.print(F("Command queue contains:"));
    for (uint32_t i = 0; i < queue->commands.size(); i++)
    {
        WebSerial.print(F(" "));
        WebSerial.print(queue->commands[i]);
    }
    WebSerial.println();
    WebSerial.print(F("Data queue contains:"));
    for (uint32_t i = 0; i < queue->data.size(); i++)
    {
        WebSerial.print(F(" "));
        WebSerial.print(queue->data[i]);
    }
    WebSerial.println();
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
    mapfile = SD.open("/MAP.txt");
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
    NAVSensors.enablePositionEncoders();   // le coordinate X Y del robot saranno diverse da quelle di partenza, quindi bisogna tenerne traccia

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

int32_t NAV::getRealAngleDiff(int32_t angle1, int32_t angle2)
{
    diff = angle1 - angle2;
    while (diff <= -18000)
        diff += 36000;
    while (diff > 18000)
        diff -= 36000;
    return diff;
}

void NAV::fillXBytes(uint32_t bytes)
{
    if (!USE_SD) return;
    mapfile = SD.open("/MAP.txt", FILE_WRITE);
    uint8_t num[1] = {0};
    for (int i = 0; i < bytes; i++)
        mapfile.write(num, 1);
    mapfile.close();
}

void NAV::fillMap(int32_t minx, int32_t miny, int32_t maxx, int32_t maxy)
{
    uint32_t fill_start_position = getSDLastReadablePosition("/raw_map.txt");

    uint32_t width = abs(maxx - minx);
    uint32_t height = abs(maxy - miny);

    Point inac_point = Point(minx, miny, 1);
    mapfile = SD.open("/raw_map.txt", FILE_APPEND);

    Serial.printf("Creating map... ");
    chrono.start();
    for (uint32_t col = 0; col <= width; col++)
    {
        inac_point.x += col;
        for (uint32_t row = 0; row <= height; row++)
        {
            inac_point.y += row;
            writePoint(inac_point);
        }
    }
    mapfile.close();
    float speed = (float)(width * height * 15) / ((float)chrono.getTime() / 1000);
    Serial.printf("Done (%f KB/s)\n", speed);

    removeDuplicatesFromMap(width, height, minx, miny, fill_start_position);
}

NAV::MapPoint NAV::searchPoint(Point p)
{
    bool map_opened = false;
    uint32_t current_block = 1;

    while (current_block <= navutil.last_full_block)
    {
        // prima legge cosa c'è nel blocco attuale, altrimenti cerca negli altri (così è più veloce)
        if (p.x >= nav->MinX->value && p.x <= nav->MaxX->value && p.y >= nav->MinY->value && p.y <= nav->MaxY->value)
        {
            for (uint32_t i = 0; i < POINTS_PER_BLOCK; i++)
            {
                if (p.x == nav->arrX[i] && p.y == nav->arrY[i])
                {
                    if (map_opened)
                        mapfile.close();
                    return MapPoint(p.x, p.y, current_block, i, nav->arrID[i]);
                }
            }
        }

        if (!map_opened)
        {
            map_opened = true;
            mapfile = SD.open("/MAP.txt");

        }
        readBlock(current_block, false);    // false per non aprire il file visto che è già aperto
        current_block++;
    }

    if (map_opened)
        mapfile.close();
    return MapPoint(false);
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

void NAV::removeDuplicatesFromMap(int32_t width, int32_t height, int32_t minx, int32_t miny, uint32_t fill_start_position)
{
    // SI PUò RENDERE UN PROCESSO PER DUE CORE
    Point p = Point(0, 0);
    Point fill_p = Point(0, 0);
    uint32_t fill_p_position = 0;

    uint32_t position_idx = 0;
    uint32_t* positions_to_overwrite = new uint32_t[POINTS_PER_BLOCK * 4];
    uint32_t* ids = new uint32_t[POINTS_PER_BLOCK * 4];
    for (int32_t i = 0; i < POINTS_PER_BLOCK * 4; i++)
    {
        positions_to_overwrite[i] = 0;
        ids[i] = 0;
    }

    Serial.printf("Removing duplicate fill points... ", width, height);
    mapfile = SD.open("/raw_map.txt");
    for (int32_t block = 0; block < fill_start_position / MAP_BLOCK_SIZE; block++)
    {
        readPointBlock(block);
        for (int32_t map_point_idx = 0; map_point_idx < POINTS_PER_BLOCK; map_point_idx++)  // il fill start indica la fine della mappa fatta dal robot
        {
            p = nav->points[map_point_idx];
            if (p.id == 3)  
                continue;
            fill_p_position = fill_start_position + ((height * (abs(p.x - minx)) + abs(p.y - miny) + abs(p.x - minx)) * MAP_POINT_SIZE);
            readPoint(fill_p_position, &fill_p);

            if (p.x == fill_p.x && p.y == fill_p.y)
            {
                positions_to_overwrite[position_idx] = fill_p_position;
                ids[position_idx] = p.id;
                position_idx++;
            }

            if (position_idx == POINTS_PER_BLOCK * 4)
            {
                // deve scrivere i dati sull'SD
                mapfile.close();    // chiude il file che è in modalità read
                mapfile = SD.open("/raw_map.txt", FILE_WRITE);
                for (int32_t i = 0; i < position_idx; i++)
                {
                    mapfile.seek(positions_to_overwrite[i] + 14);   // deve sovrascrivere solo l'ID del punto
                    mapfile.write(ids[i] + '0');                    // scrive l'ID (sarà quasi sempre 2)
                }
                position_idx = 0;
                mapfile.close();
                mapfile = SD.open("/raw_map.txt");
            }
        }
        if (block == 1)
            break;
    }

    // se non ha già scritto i dati sull'SD (cioè che non c'erano oltre 1024 punti da scrivere), li scrive ora
    if (position_idx != 0)
    {
        if (position_idx == POINTS_PER_BLOCK * 4)
            position_idx -= 1;
        mapfile.close();
        mapfile = SD.open("/raw_map.txt", FILE_WRITE);
        for (int32_t i = 0; i <= position_idx; i++)
        {
            mapfile.seek(positions_to_overwrite[i] + 14);
            mapfile.write(ids[i] + '0');
        }
        position_idx = 0;
        mapfile.close();
    }
    else
    {
        readPoint(15, &p);
        mapfile.close();
    }

    delete[] positions_to_overwrite;
    delete[] ids;

    Serial.printf("Done\n");

    createProcessedMap("/raw_map.txt", "/map.txt", fill_start_position, width, height);
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

    for (int i = 0; i < POINTS_PER_BLOCK; i++)
    {
        readPoint(i * MAP_POINT_SIZE + start_position, &nav->points[i]);
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

NAV::Point NAV::getPointFromArr(uint8_t* arr, uint32_t len)
{
    int32_t power = 1;
    Point p = Point(0, 0);
    for (int32_t i = 13; i >= 7; i--)
    {
        if (arr[i] == '-')
        {
            if (i == 7)
                p.y *= -1;
            else
                power *= -1;
            continue;
        }
        p.y += (arr[i] - '0') * power;
        power *= 10;
    }
    power = 1;
    for (int32_t i = 6; i >= 0; i--)
    {
        if (arr[i] == '-')
        {
            if (i == 0)
                p.x *= -1;
            else
                power *= -1;
            continue;
        }
        p.x += (arr[i] - '0') * power;
        power *= 10;
    }

    return p;
}

void NAV::readPoint(uint32_t seek, Point* p)
{
    p->x = 0;
    p->y = 0;
    if (!mapfile.seek(seek))
        Serial.printf("seek fail %d", seek);
    mapfile.read(raw_point, 15);
    int32_t power = 1;
    for (int32_t i = 13; i >= 7; i--)
    {
        if (raw_point[i] == '-')
        {
            if (i == 7)
                p->y *= -1;
            else
                power *= -1;
            continue;
        }
        p->y += (raw_point[i] - '0') * power;
        power *= 10;
    }
    power = 1;
    for (int32_t i = 6; i >= 0; i--)
    {
        if (raw_point[i] == '-')
        {
            if (i == 0)
                p->x *= -1;
            else
                power *= -1;
            continue;
        }
        p->x += (raw_point[i] - '0') * power;
        power *= 10;
    }
    p->id = raw_point[14] - '0';
}

void NAV::createProcessedMap(const char* raw_map_name, const char* map_name, uint32_t fill_start, int32_t width, int32_t height)
{
    Point* points_to_write = new Point[1024];
    uint32_t last_pos = getSDLastReadablePosition(raw_map_name);
    uint32_t points_to_read = (last_pos - fill_start) / MAP_POINT_SIZE;
    uint32_t points_in_arr = 0;
    uint32_t seek_pos = 0;
    Serial.printf("Writing processed map to file... ", width, height);
    mapfile = SD.open(raw_map_name);
    for (int32_t block = 0; block < int32_t(std::ceil((float)points_to_read / 1024.0)); block++)
    {
        for (int32_t i = 0; i < 1024; i++)
        {
            if (block * 1024 + i >= points_to_read)
                break;

            seek_pos = fill_start + block * 15360 /* 1024 * 15 */ + i * 15;
            readPoint(seek_pos, &points_to_write[i]);
            points_in_arr = i;
        }

        mapfile.close();
        mapfile = SD.open(map_name, FILE_APPEND);
        for (int i = 0; i < points_in_arr + 1; i++)
            writePoint(points_to_write[i]);
        mapfile.close();
        mapfile = SD.open(raw_map_name);
    }
    mapfile.close();

    delete[] points_to_write;

    Serial.printf("Done\n");

    fillBorderHoles(map_name, width, height);
}

void NAV::writePoint(Point p)
{
    int32_t tempx = abs(p.x);
    int32_t tempy = abs(p.y);
    for (int32_t i = 0; i < 15; i++)
        raw_point[i] = 0;

    for (int32_t i = 6; i >= 0; i--)
    {
        if (p.x < 0 && i == 0)
        {
            raw_point[i] = '-';
            break;
        }
        raw_point[i] = tempx % 10 + '0';
        tempx /= 10;
    }
    for (int32_t i = 13; i >= 7; i--)
    {
        if (p.y < 0 && i == 7)
        {
            raw_point[i] = 45;
            break;
        }
        raw_point[i] = tempy % 10 + '0';
        tempy /= 10;
    }
    raw_point[14] = p.id + '0';
    mapfile.write(raw_point, 15);
}

void NAV::fillBorderHoles(const char* map_name, int32_t width, int32_t height)
{
    uint32_t seek = 0;
    Point p = Point();

    Serial.printf("%d x %d\n", width, height);
    mapfile = SD.open(map_name);
    for (int32_t row = 0; row <= width; row++)
    {
        for (int32_t col = 0; col <= height; col++)
        {
            seek = row * height * 15 + (col + row) * 15;
            readPoint(seek, &p);
        }
        if (row == 1)
            break;
    }
}

void NAV::begin()
{
    nav_ptr = &navqueue;
    avoid_ptr = &avoidqueue;

    if (!USE_SD) return;
    //spi->begin(SCLK, MISO, MOSI, CS);
    SPI.begin(SCLK, MISO, MOSI, CS);
    if (SD.begin(CS))
        NAVCore.println(F("(Navigation) SD Card OK"));
    else
    {
        NAVCore.println(F("(Navigation) SD Card not recognized, RESTART IN 5 SECONDS"));
        delay(5000);
        ESP.restart();
    }

    navutil.last_readable_pos = getSDLastReadablePosition("/raw_map.txt");
    getMaxBlock("/raw_map.txt", true);

    navutil.last_readable_pos = getSDLastReadablePosition("/raw_map.txt");
    NAVCore.println("Last full block is", navutil.last_full_block);
    checkMapCompletion();
    readBlock(1);
}
