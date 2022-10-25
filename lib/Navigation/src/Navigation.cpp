#include <vector>
#include <Navigation.h>
#include <Sensors.h>
#include <Motors.h>
#include <Core.h>
#include <BluetoothSerial.h>
#ifdef ENABLE_WEBSERIAL
#include <WebSerial.h>
#endif
#include <SETTINGS.h>
#include <tuple>
#include <SPI.h>
#include <SD.h>
#define MOSI 17
#define MISO 35
#define SCLK 15
#define CS 23
#define MAP_BLOCK_SIZE 5632   // byte
#define MAP_DATA_SIZE 9       // dimensione dati x y in byte
//#define MAP_CLUSTER_SIZE 22528
#define UP 0
#define DOWN 1

SPIClass *spi = NULL;

Sensors NAVSensors;
Motors NAVMotors;
Core NAVCore;
BluetoothSerial NAVSerial;
uint32_t t3 = 0;

File mapfile;

/*

CODICI BLOCCHI MAPPA
-1: ignora punto
0: accessibile
1: non accessbile
2: bordo mappa

*/

/// OBSOLETO ///
/*
    // I VALORI DEL GIROSCOPIO SONO MOLTIPLICATI TUTTI PER 100 (data type uint32_t)
    // TUTTI GLI ALTRI VALORI HANNO COME UNITA' DI MISURA cm (data type uint32_t)

   ----------------------------------
   | 0° = X, avanti                 |
   | 90° = Y, destra                |
   | 180° = x, indietro             |
   | 270° = y, sinistra             |
   |                                |
   |         0°                     |
   |     /       \                  |
   | 270°           90°             |
   |     \       /                  |
   |        180°                    |
   |                                |
   | 0 < x < 90: quadrant = 0;      |
   | 90 < x < 180: quadrant = 1;    |
   | 180 < x < 270: quadrant = 2;   |
   | 270 < x < 360: quadrant = 3    |
   |                                |
   | vector slope guarda sempre     |
   | il lato più basso (ad es, se   |
   | quadrant = 2, vector slope si  |
   | riferisce a 180°)              |
   |                                |
   | coarse dir                     |
   | 0: 0°                          |
   | 1: 90°                         |
   | 2: 180°                        |
   | 3: 270°                        |
   ----------------------------------
*/

bool autorun = false;
uint32_t distance_target = 0;
uint8_t distance_traveled_before_uturn_sudden_stop = 0;
float distance_traveled = 0.00;
uint16_t heading_target = 0;
uint8_t rotation_direction = RIGHT;
float start_heading = false;
bool rotating = false;
bool rotated = false;
bool obstacle_detected = false;
bool going_forward = false;
bool gone_forward = false;
bool going_backwards = false;
bool gone_backwards = false;
bool distance_target_reached = false;
bool sudden_stop = false;
bool check_second_sudden_stop = false;
char movement_direction;
bool obstacle_detected_before_moving = false;
bool robot_moving_x_y = false;
bool forward_wait_for_rotation = false;
uint32_t forward_wait_for_rotation_val = 0;
int32_t forward_wait_for_rotation_hdg = 0;
uint32_t timer = millis();
uint8_t obstacle_sensor_direction = FRONT;
uint8_t obstacle_sensor_type = INFRARED;
bool bordermode = false;
uint32_t bordermode_start_time = 0;

// stage di movimento
uint8_t stages = 0;

struct Vectors
{
    float xvector = 0;
    float yvector = 0;
    float last_dst = 0;
    float *xvec_ptr = &xvector;
    float *yvec_ptr = &yvector;
    int32_t intxvector = (int32_t)(*xvec_ptr * 100);
    int32_t intyvector = (int32_t)(*yvec_ptr * 100);
};

struct
{
    bool active = false;
    uint32_t timer = millis();
    char lastMove = 'L';
    bool isBlocked = false;
    /* data */
} UTurn;
struct
{
    bool avoid = false;
    uint8_t stages = 0;
    uint32_t timer = 0;
    bool whileUturn = false;
} SuddenStop;

struct
{
    bool active = false;
    uint8_t stages;
    uint16_t top_hdg_border;
    uint16_t bottom_hdg_border;
} RotationControl;

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
    int32_t arrX[256] = {};
    int32_t arrY[256] = {};
    uint8_t arrID[256] = {};
    int32_t curr_arrX[256] = {};
    int32_t curr_arrY[256] = {};
    uint8_t curr_arrID[256] = {};
    struct 
    {
        int32_t value = 0;
        uint32_t block = 0;
        uint32_t idx = 0;
    } MaxX;
    struct
    {
        int32_t value = 1000000000;
        uint32_t block = 0;
        uint32_t idx = 0;
    } MinX;
    struct
    {
        int32_t value = 0;
        uint32_t block = 0;
        uint32_t idx = 0;
    } MaxY;
    struct
    {
        int32_t value = 1000000000;
        uint32_t block = 0;
        uint32_t idx = 0;
    } MinY;
};

struct NavMapUtil
{
    uint32_t last_readable_pos = 0;
    uint32_t last_full_block = 0;
    uint32_t last_incomplete_block = 0;
};

struct MovementMotors
{
    bool stall = false;
    uint8_t antistall_step = 0;
    int32_t heading_to_maintain = 0;
    uint32_t time = 0;
    uint32_t motor_start_time = 0;
    bool backwards = false;
};

Vectors vectors;
NavMap navmap;
NavMapUtil navutil;
MovementMotors mots;
uint32_t test = 0;

void NAV::update()
{
    uint64_t start_time = micros();
    if (forward_wait_for_rotation && !rotating)
    {
        forward_wait_for_rotation = false;
        goForward(forward_wait_for_rotation_val, forward_wait_for_rotation_hdg);
    }

    if (going_forward)
    {
        distance_traveled = NAVSensors.getTraveledDistance();

        if (UTurn.active)
            distance_traveled_before_uturn_sudden_stop = distance_traveled;

        if (distance_traveled >= distance_target && distance_traveled != 0 && distance_target != 0)
        {
            stop();
            going_forward = false;
            pausevars.was_going_forward = false;
            gone_forward = true;
            distance_target_reached = true;
        }
    }

    if (going_backwards)
    {
        distance_traveled = NAVSensors.getTraveledDistance();

        if (distance_traveled >= distance_target && distance_traveled != 0 && distance_target != 0)
        {
            stop();
            pausevars.was_going_backwards = false;
            going_backwards = false;
            gone_backwards = true;
            distance_target_reached = true;
        }
    }

    if (obstacle_detected)
    {
        NAVCore.println(F("(Navigation.cpp) OBSTACLE DETECTED"));
        obstacle_detected = false;
        going_forward = false;
        pausevars.was_going_forward = false;

        stop();
        uint32_t hdg = getHeading360();
        if (!bordermode)
        {
            /*int32_t xvec, yvec;
            std::tie(xvec, yvec) = addToVectors(5, hdg);
            mapfile = SD.open(F("/MAP.txt"), FILE_APPEND);
            mapfile.print(xvec);
            mapfile.print(F(","));
            mapfile.print(yvec);
            mapfile.print(F(","));
            mapfile.print(1);
            mapfile.print(F(","));
            navmap.last_readable_pos += mapfile.position();
            mapfile.close();*/
        }

        if (autorun)
        {
            // queste azioni vengono fatte solo se il robot è in modalità AUTO
            // altrimenti, sono attivi solo i sensori (il robot si ferma a un ostacolo)
            if (!UTurn.active)
            {
                // attiva l'u-turn solo se questo non è in corso
                stop();
                enableUTurn();
            }
            else
            {
                // se l'u-turn è in corso, quindi il robot sta andando avanti durante questa fase, allora il robot deve saltare all'ultima fase dell'uturn
                // resetta l'u-turn poi continua
                disableUTurn();
                UTurn.isBlocked = true;
            }
        }
        else
            externalStop();
    }

    if (obstacle_detected_before_moving)
        obstacle_detected_before_moving = false;    

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

    if (sudden_stop)
    {
        sudden_stop = false;
        going_forward = false;
        pausevars.was_going_forward = false;
        resetMovementVars();

        if (autorun)
        {
            if (check_second_sudden_stop)
            {
                // se il robot si è fermato di colpo, è andato indietro e ora sta andando avanti;
                // se viene di nuovo fermato di colpo (entro 1.5 secondi dal primo blocco), allora cambia direzione (abilitando uturn)
                // questo solo se l'uturn non era già in corso; se lo era, riprende da dove era rimasto
                NAVSerial.println();
                NAVSerial.println("CHECK SECOND SUDDEN STOP");
                if (SuddenStop.whileUturn)
                {
                    NAVSerial.println("SUDDENSTOP WHILE UTURN");
                    if (distance_traveled > 2)
                    {
                        NAVSerial.println("DISTANCE TRAVELED OVER 2");
                        SuddenStop.whileUturn = false;
                        distance_traveled = distance_traveled_before_uturn_sudden_stop;
                        UTurn.active = true;
                        check_second_sudden_stop = false;
                    }
                    else if (millis() - SuddenStop.timer <= 3000)
                    {
                        NAVSerial.println("DISTANCE TRAVELED LESS THAN 2 AND SUDDENSTOP TIMER LESS THAN 3s");
                        SuddenStop.whileUturn = false;
                        check_second_sudden_stop = false;
                        enableUTurn();
                    }
                }
                else if (millis() - SuddenStop.timer <= 3000)
                {
                    check_second_sudden_stop = false;
                    enableUTurn();
                }
            }
            else
            {
                if (UTurn.active)
                {
                    NAVCore.println((char *)"(Navigation.cpp) SUDDENSTOP WHILE UTURN");
                    SuddenStop.whileUturn = true;
                    // disabilita uturn temporaneamente
                    UTurn.active = false;
                }
                enableSuddenStopAvoid();
                SuddenStop.stages = 0;
            }
        }
    }

    if (check_second_sudden_stop)
    {
        if (distance_traveled > 4)
            check_second_sudden_stop = false;
    }

    if (rotating && ENABLE_ROTATION_SENSING)
    {
        uint16_t currentHDG = getHeading360();
        NAVCore.println("currenthdg", currentHDG);

        if (currentHDG > heading_target - 50 && currentHDG < heading_target + 50)
        {
            stop();
            NAVSensors.resetMovementVars();
            rotating = false;
            rotated = true;
            RotationControl.active = true;
            RotationControl.top_hdg_border = heading_target + 50;
            RotationControl.bottom_hdg_border = heading_target - 50;
            timer = millis();
        }
        else if (ENABLE_ROTATION_LOOP)
        {
            // 𝑦=140+𝑎𝑥
            // 𝑎 = 5.3

            int32_t current_hdg = convertHDGTo180(currentHDG);
            int32_t target_hdg = convertHDGTo180(heading_target);

            int32_t diff = 0;

            if (current_hdg > 0 && target_hdg > 0) // entrambi a destra
                diff = current_hdg - target_hdg;
            else if (current_hdg < 0 && target_hdg < 0) // entrambi a sinistra
                diff = current_hdg - target_hdg;
            else if ((current_hdg > 0 && target_hdg < 0) || (current_hdg < 0 && target_hdg > 0)) // sono in metà diverse
            {
                if (current_hdg < 0)
                    current_hdg *= -1;
                if (target_hdg < 0)
                    target_hdg *= -1;
                
                if (current_hdg > 9000 && target_hdg > 9000) // se sono sotto
                    diff = 36000 - current_hdg - target_hdg;
                else if (current_hdg < 9000 && target_hdg < 9000) // se sono sopra
                    diff = current_hdg + target_hdg;
            }

            if (diff < 0)
                diff *= -1;

            uint8_t spd;
            if (diff < 2300)
            {
                spd = 140 + 5 * (diff / 100);
                NAVMotors.setSpeed(spd, BOTH);
            }
            else
                NAVMotors.setSpeed(MOT_NORM_VAL, BOTH);
        }
    }

    if (UTurn.active)
    {
        switch (stages)
        {
        case 0:
        {
            if (millis() - timer > GLOBAL_NAV_DELAY)
            {
                stages++;
                goBackwards(1);
            }
            break;
        }
        case 1:
        {
            if (gone_backwards)
            {
                if (millis() - timer > GLOBAL_NAV_DELAY)
                {
                    stages++;
                    gone_backwards = false;
                    if (UTurn.lastMove == 'L')
                    {
                        rotateForDeg(90);
                        UTurn.lastMove = 'R';
                    }
                    else
                    {
                        rotateForDeg(-90);
                        UTurn.lastMove = 'L';
                    }
                }
            }
            else
                timer = millis();
            break;
        }
        case 2:
        {
            if (rotated)
            {
                if (millis() - timer > GLOBAL_NAV_DELAY)
                {
                    stages++;
                    rotated = false;
                    goForward(10);
                }
            }
            else
                timer = millis();
            break;
        }
        case 3:
        {
            if (gone_forward)
            {
                if (millis() - timer > GLOBAL_NAV_DELAY)
                {
                    stages++;
                    gone_forward = false;
                    if (UTurn.lastMove == 'R')
                        rotateForDeg(90);
                    else
                        rotateForDeg(-90);
                }
            }
            else
                timer = millis();
            break;
        }
        case 4:
        {
            if (rotated)
            {
                if (millis() - timer > GLOBAL_NAV_DELAY)
                {
                    stages++;
                    rotated = false;
                    goForward();
                    /*
                    checkDirectionsDeg();
                    NAVSerial.print("QUADRANTE: ");
                    NAVSerial.println(quadrant);
                    NAVSerial.print("VECTOR SLOPE: ");
                    NAVSerial.println(vector_slope);
                    */
                }
            }
            else
                timer = millis();
            break;
        }
        case 5:
        {
            disableUTurn();
            break;
        }
        }
    }

    if (UTurn.isBlocked)
    {
        switch (stages)
        {
        case 0:
        {
            if (millis() - timer > GLOBAL_NAV_DELAY)
            {
                stages++;
                goBackwards(2);
            }
            break;
        }
        case 1:
        {
            if (gone_backwards)
            {
                if (millis() - timer > GLOBAL_NAV_DELAY)
                {
                    stages++;
                    gone_backwards = false;
                    if (UTurn.lastMove == 'R')
                    {
                        rotateForDeg(90);
                    }
                    else
                    {
                        rotateForDeg(-90);
                    }
                }
            }
            else
            {
                timer = millis();
            }
            break;
        }
        case 2:
        {
            if (rotated)
            {
                if (millis() - timer > GLOBAL_NAV_DELAY)
                {
                    stages++;
                    rotated = false;
                    goForward();
                }
            }
            else
            {
                timer = millis();
            }
            break;
        }
        case 3:
        {
            disableUTurn();
            break;
        }
        }
    }

    if (SuddenStop.avoid)
    {
        if (movement_direction == 'w')
        {
            switch (SuddenStop.stages)
            {
                case 0:
                {
                    if (millis() - timer > GLOBAL_NAV_DELAY)
                    {
                        SuddenStop.stages++;
                        goBackwards(2);
                    }
                    break;
                }
                case 1:
                {
                    if (gone_backwards)
                    {
                        if (millis() - timer > GLOBAL_NAV_DELAY)
                        {
                            SuddenStop.stages++;
                            gone_backwards = false;
                            goForward();
                        }
                    }
                    break;
                }
                case 2:
                {
                    disableSuddenStopAvoid();
                    check_second_sudden_stop = true;
                    SuddenStop.timer = millis();
                    SuddenStop.stages = 0;
                    break;
                }
            }
        }
    }

    if (robot_moving_x_y)
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

        float dst = NAVSensors.getLastTraveledDistance();
        if (vectors.last_dst != dst && dst > 0.05)
        {
            uint32_t hdg_360 = getHeading360();
            vectors.last_dst = dst;

            // i vettori si alternano sin e cos perché il riferimento dell'heading cambia in base al quadrante

            if (hdg_360 > 0 && hdg_360 < 9000) // heading positivo per X e Y -- alto destra -- riferimento 0°
            {
                if (going_backwards)
                    hdg_360 = invertHDG(hdg_360);
                float rad = radians(hdg_360 / 100);
                vectors.xvector += dst * sin(rad); // positivo per X
                vectors.yvector += dst * cos(rad); // positivo per Y
            }
            else if (hdg_360 > 9000 && hdg_360 < 18000) // heading positivo per X, negativo per Y -- basso destra -- riferimento 90°
            {
                if (going_backwards)
                    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertFromRelativeHDGToRef0(hdg_360, 90);
                float rad = radians(hdg_360 / 100);
                vectors.xvector += dst * cos(rad); // positivo per X
                vectors.yvector -= dst * sin(rad); // negativo per Y
            }
            else if (hdg_360 > 18000 && hdg_360 < 27000) // heading negativo per X e Y -- basso sinistra -- riferimento 180°
            {
                if (going_backwards)
                    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertFromRelativeHDGToRef0(hdg_360, 180);
                float rad = radians(hdg_360 / 100);
                vectors.xvector -= dst * sin(rad); // negativo per X
                vectors.yvector -= dst * cos(rad); // negativo per Y
            }
            else // heading positivo per Y, negativo per X -- alto sinistra -- riferimento 270°
            {
                if (going_backwards)
                    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertFromRelativeHDGToRef0(hdg_360, 270);
                float rad = radians(hdg_360 / 100);
                vectors.xvector -= dst * cos(rad); // negativo per X
                vectors.yvector += dst * sin(rad); // positivo per Y
            }

            if (LOG_MAP || bordermode)
            {
                mapfile = SD.open(F("/MAP.txt"), FILE_APPEND);
                if (vectors.xvector < 0)
                {
                    mapfile.print(F("-"));
                    for (int i = 0; i < MAP_DATA_SIZE - 1 - countDigits((uint32_t)(vectors.xvector * -100)); i++)
                        mapfile.print(F("0"));

                    mapfile.print((uint32_t)(vectors.xvector * -100));
                }
                else
                {
                    for (int i = 0; i < MAP_DATA_SIZE - countDigits((uint32_t)(vectors.xvector * 100)); i++)
                        mapfile.print(F("0"));

                    mapfile.print((uint32_t)(vectors.xvector * 100));
                }

                mapfile.print(F(","));

                if (vectors.yvector < 0)
                {
                    mapfile.print(F("-"));
                    for (int i = 0; i < MAP_DATA_SIZE - 1 - countDigits((uint32_t)(vectors.yvector * -100)); i++)
                        mapfile.print(F("0"));

                    mapfile.print((uint32_t)(vectors.yvector * -100));
                }
                else
                {
                    for (int i = 0; i < MAP_DATA_SIZE - countDigits((uint32_t)(vectors.yvector * 100)); i++)
                        mapfile.print(F("0"));

                    mapfile.print((uint32_t)(vectors.yvector * 100));
                }

                mapfile.print(F(","));

                if (bordermode)
                {
                    mapfile.print(2);
                    if ((vectors.xvector < 5 && vectors.xvector > -5) && (vectors.yvector < 5 && vectors.yvector > -5) && millis() - bordermode_start_time > 10000)
                    {
                        bordermode = false;
                        stop();
                        NAVCore.println("Bordermode COMPLETED");
                        ESP.restart();
                    }
                }
                else
                    mapfile.print(0);
                mapfile.print(F(","));
                navutil.last_readable_pos += mapfile.position();

                mapfile.close();
            }
            
            if (!bordermode)
            {
                uint32_t pos_xvector = (uint32_t)abs(vectors.xvector * 100);
                uint32_t pos_yvector = (uint32_t)abs(vectors.yvector * 100);
                if (pos_xvector > 200 && pos_yvector > 200)
                {
                    uint32_t pt_dst, pt_id, pt_idx;
                    std::tie(pt_dst, pt_id, pt_idx) = getClosestPointDst();

                    if (pt_id == BORDER)
                    {
                        if (pt_dst < 500)
                        {
                            stop();
                            NAVCore.println("BORDER CLOSER THAN 5CM");
                            NAVCore.println("CLOSEST POINT X", navmap.curr_arrX[pt_idx]);
                            NAVCore.println("CLOSEST POINT Y", navmap.curr_arrY[pt_idx]);
                            NAVCore.println("DST", pt_dst);
                        }
                    }
                }
            }

            NAVCore.println("X: ", vectors.xvector);
            NAVCore.println("Y: ", vectors.yvector);
        }
    }

    if (mots.stall)
    {
        if (millis() - mots.motor_start_time > 1000)
        {
            switch (mots.antistall_step)
            {
                case 0:
                {
                    if (mots.backwards)
                        goForward(5, mots.heading_to_maintain);
                    else
                        goBackwards(5, mots.heading_to_maintain);
                    mots.antistall_step++;
                    break;
                }
                case 1:
                {
                    if (mots.backwards)
                    {
                        if (gone_forward)
                        {
                            if (millis() - mots.time > 50)
                            {
                                goBackwards(4294967295, mots.heading_to_maintain);
                                mots.antistall_step++;
                                gone_forward = false;
                            }
                        }
                        else
                            mots.time = millis();
                    }
                    else
                    {
                        if (gone_backwards)
                        {
                            if (millis() - mots.time > 50)
                            {
                                goForward(4294967295, mots.heading_to_maintain);
                                mots.antistall_step++;
                                gone_backwards = false;
                            }
                        }
                        else
                            mots.time = millis();
                    }
                    break;
                }
                case 2:
                {
                    mots = {};
                    break;
                }
            }
        }
        else
            mots.stall = false;
    }

    t3 = micros() - start_time;
}

uint32_t NAV::getTime()
{
    return t3;
}

void NAV::goForward(uint32_t cm, int32_t hdg_to_maintain)
{
    if (rotating)
    {
        forward_wait_for_rotation_val = cm;
        forward_wait_for_rotation_hdg = hdg_to_maintain;
        forward_wait_for_rotation = true;
        return;
    }
    pausevars.was_going_forward = true;
    mots.motor_start_time = millis();
    NAVSensors.resetTraveledDistance();
    NAVMotors.forward();
    if (hdg_to_maintain == AUTO)
        mots.heading_to_maintain = NAVMotors.getHeadingToMaintain();
    else
        mots.heading_to_maintain = hdg_to_maintain;
    distance_target = cm;
    resetMovementVars();
    going_forward = true;
    robot_moving_x_y = true;
}

void NAV::goBackwards(uint32_t cm, int32_t hdg_to_maintain)
{
    pausevars.was_going_forward = true;
    mots.backwards = true;
    mots.motor_start_time = millis();
    NAVSensors.resetTraveledDistance();
    NAVMotors.backwards();
    if (hdg_to_maintain == AUTO)
        mots.heading_to_maintain = NAVMotors.getHeadingToMaintain();
    else
        mots.heading_to_maintain = hdg_to_maintain;
    distance_target = cm;
    resetMovementVars();
    going_backwards = true;
    robot_moving_x_y = true;
}

void NAV::rotateForDeg(int16_t degs)
{
    /*
            0°
        /       \
    270°           90°
        \       /
           180°
    */

    resetMovementVars();
    int16_t heading_to_reach = degs * 100;
    if (heading_to_reach < 0)
        heading_to_reach = heading_to_reach + 36000; // 36000
    start_heading = getHeading360();
    int32_t over_start_heading = start_heading + heading_to_reach;
    if (over_start_heading - 36000 < 0)
        heading_target = over_start_heading;
    else
        heading_target = over_start_heading - 36000;

    rotating = true;

    if (degs < 0)
    {
        rotation_direction = LEFT;
        NAVMotors.left();
    }
    else
    {
        rotation_direction = RIGHT;
        NAVMotors.right();
    }
}

void NAV::rotateToDeg(uint16_t heading, char direction)
{
    resetMovementVars();
    rotating = true;
    heading_target = heading;
    if (direction == 'L')
        NAVMotors.left();
    else
        NAVMotors.right();
}

void NAV::obstacleDetectedWhileMoving(uint8_t sensor_type, uint8_t sensor_direction)
{
    obstacle_detected = true;
    obstacle_sensor_type = sensor_type;
    obstacle_sensor_direction = sensor_direction;
}

void NAV::externalStop()
{
    stop();
    resetMovementVars();
    UTurn.active = false;
    autorun = false;
    NAVSensors.setAutoRun(false);
    NAVSensors.resetMovementVars();
    robot_moving_x_y = false;
}

uint16_t NAV::getHeading360()
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
    NAVSensors.setAutoRun(true);
}

void NAV::resetMovementVars()
{
    pausevars.was_going_backwards = false;
    pausevars.was_going_forward = false;
    going_forward = false;
    gone_forward = false;
    going_backwards = false;
    gone_backwards = false;
    rotating = false;
    rotated = false;
    obstacle_detected = false;
    distance_target_reached = false;
    distance_traveled = 0;
}

void NAV::suddenStop(char direction)
{
    sudden_stop = true;
    movement_direction = direction;
}

void NAV::enableUTurn()
{
    stages = 0;
    UTurn.active = true;
    gone_forward = false;
    rotated = false;
    timer = millis();
}

void NAV::disableUTurn()
{
    stages = 0;
    UTurn.active = false;
    rotated = false;
    obstacle_detected = false;
    gone_forward = false;
    gone_backwards = false;
    distance_target_reached = false;
    timer = millis();
    UTurn.isBlocked = false;
}

void NAV::enableSuddenStopAvoid()
{
    SuddenStop.avoid = true;
    SuddenStop.stages = 0;
}

void NAV::disableSuddenStopAvoid()
{
    SuddenStop.stages = 0;
    SuddenStop.avoid = false;
}

float NAV::getVirtualHDG(int16_t heading)
{
    int16_t heading_to_reach = heading * 100;
    if (heading_to_reach < 0)
        heading_to_reach += 36000; // 36000 + 900 (900 = errore)
    //else
        //heading_to_reach -= 650;
    start_heading = NAVSensors.getHeading();
    if (start_heading < 0)
        start_heading += 36000;
    int32_t over_start_heading = start_heading + heading_to_reach;
    if (over_start_heading - 36000 < 0)
        heading_target = over_start_heading;
    else
        heading_target = over_start_heading - 36000;

    return heading_target;
}

char NAV::getRotationDirection(uint32_t current_hdg, int32_t target_hdg)
{
    if (current_hdg - target_hdg < current_hdg + target_hdg)
        return 'L';
    else
        return 'R';
}

void NAV::rotateUntil(char dir, bool *var, bool condition)
{
    RotateUntil.condition = condition;
    RotateUntil.dir = dir;
    RotateUntil.var_ptr = var;
    RotateUntil.active = true;
}

void NAV::obstacleDetectedBeforeMoving(uint8_t sensor_type, uint8_t sensor_direction)
{
    stop();
    obstacle_sensor_type = sensor_type;
    obstacle_sensor_direction = sensor_direction;
    obstacle_detected_before_moving = true;
}

int32_t NAV::convertHDGTo180(uint16_t heading, bool always_positive)
{
    if (heading > 18000)
        if (always_positive && heading - 36000 < 0)
            return (heading - 36000) * -1;
        else
            return heading - 36000;
    else
        return heading;
}

void NAV::stop()
{
    NAVMotors.stop();
    robot_moving_x_y = false;
    pausevars.was_going_backwards = false;
    pausevars.was_going_forward = false;
    going_forward = false;
    going_backwards = false;
    rotating = false;
    distance_traveled = 0;
}

uint16_t NAV::convertFromRelativeHDGToRef0(uint16_t hdg, uint16_t reference)
{
    uint16_t return_hdg = 36000 - reference * 100 + hdg;
    if (return_hdg - 36000 > 0)
        return return_hdg - 36000;
    else
        return return_hdg;
}

uint16_t NAV::invertHDG(uint16_t hdg)
{
    uint16_t return_hdg = hdg + 18000;
    if (return_hdg - 36000 > 0)
        return return_hdg - 36000;
    else
        return return_hdg;
}

void NAV::eraseSD(const char *path)
{
    mapfile = SD.open(path, FILE_WRITE);
    mapfile.print("");
    mapfile.close();
}

void NAV::readBlock(uint32_t block)
{
    if (navmap.current_block == block)
        return;
    navmap = {};
    navmap.current_block = block;
    uint32_t pos = 0;
    uint32_t start_position = (block == 1) ? 0 : MAP_BLOCK_SIZE * (block - 1) - 1;
    uint32_t target_position = MAP_BLOCK_SIZE * block - 1;

    if (target_position > navutil.last_readable_pos)
        return;

    uint8_t buffer[5632] = {};
    mapfile = SD.open("/MAP.txt");
    mapfile.seek(start_position);
    mapfile.read(buffer, 5632);
    mapfile.close();

    uint32_t data_type = 0;
    uint32_t read_data = 0;
    int32_t data[768] = {};                 // ci sono 768 dati in 256 punti (256 * 3) o 5632 byte di punti
    std::array<char, 256> temp_data = {};   // convertCharArrToInt accetta solo array da 256
    temp_data[MAP_DATA_SIZE] = '\0';
    uint8_t temp_idx = 0;
    uint32_t data_idx = 0;
    for (int i = 0; i < 5632; i++)
    {
        if (buffer[i] == 44)
            continue;
        if (data_type < 2)
        {
            if (buffer[i] == 45)
                temp_data[temp_idx] = '-';
            else
                temp_data[temp_idx] = buffer[i];
            if (temp_idx == MAP_DATA_SIZE - 1)
            {
                data[data_idx] = convertCharArrToInt(temp_data);
                data_idx++;
                data_type++;
                temp_idx = 0;
                continue;
            }
            temp_idx++;
        }
        if (data_type == 2)
        {
            data[data_idx] = buffer[i] - '0';
            data_idx++;
            data_type = 0;
        }
    }

    for (int i = 0; i < 768; i++)
    {
        if (data_type == 0)
        {
            if (data[i] > navmap.MaxX.value)
            {
                navmap.MaxX.value = data[i];
                navmap.MaxX.block = block;
                navmap.MaxX.idx = read_data;
            }
            if (data[i] < navmap.MinX.value)
            {
                navmap.MinX.value = data[i];
                navmap.MinX.block = block;
                navmap.MinX.idx = read_data;
            }
            navmap.arrX[read_data] = data[i];
        }
        else if (data_type == 1)
        {
            if (data[i] > navmap.MaxY.value)
            {
                navmap.MaxY.value = data[i];
                navmap.MaxY.block = block;
                navmap.MaxY.idx = read_data;
            }
            if (data[i] < navmap.MinY.value)
            {
                navmap.MinY.value = data[i];
                navmap.MinY.block = block;
                navmap.MinY.idx = read_data;
            }
            navmap.arrY[read_data] = data[i];
        }
        else if (data_type == 2)
        {
            navmap.arrID[read_data] = data[i];
            data_type = 0;
            read_data++;
            pos = mapfile.position();
            if (pos == target_position)
                break;
            continue;
        }
        data_type++;
    }
}

uint32_t NAV::getCurrentBlock()
{
    bool xblock_found = false;
    bool yblock_found = false;
    uint32_t xblock_idx = 0;
    uint32_t yblock_idx = 0;
    uint32_t counter = 1;
    uint32_t last_block = 0;
    int32_t int_xvector = (int32_t)(vectors.xvector * 100);
    int32_t int_yvector = (int32_t)(vectors.yvector * 100);

    // FIXARE CRASH QUANDO PROVA A LEGGERE DA SD OLTRE QUANTO SI PUò LEGGERE (MAX 19K, TRIED >19K)

    if (navmap.MinX.value < int_xvector)
    {
        if (navmap.MaxX.value > int_xvector)
        {
            if (navmap.MinY.value < int_yvector)
            {
                if (navmap.MaxY.value > int_yvector)
                {
                    return navmap.current_block;
                }
            }
        }
    }

    pause();
    while (!xblock_found || !yblock_found)
    {
        if (counter != last_block)
        {
            last_block = counter;
            readBlock(counter);
        }
        else
            return navmap.current_block;

/*
        if (found > 2 || (found == 2 && !xblock_found && !yblock_found))
        {
            NAVCore.println("FOUND > 2");
            NAVCore.println("CURR BLOCK", counter);

            uint32_t curr_block = counter;
            uint32_t curr_arrXMAX = navmap.abs_maxArrX;
            uint32_t curr_arrXMIN = navmap.abs_minArrX;
            uint32_t curr_arrYMIN = navmap.abs_minArrY;
            uint32_t curr_arrYMAX = navmap.abs_maxArrY;

            uint32_t prev_block = read_blocks.size() - 2;
            readBlock(prev_block);
            uint32_t prev_arrXMIN = navmap.abs_minArrX;
            uint32_t prev_arrXMAX = navmap.abs_maxArrX;
            uint32_t prev_arrYMIN = navmap.abs_minArrY;
            uint32_t prev_arrYMAX = navmap.abs_maxArrY;
            NAVCore.println("PREV BLOCK", prev_block);

            NAVCore.println("pos_xvector", pos_xvector);
            NAVCore.println("pos_yvector", pos_yvector);
            NAVCore.println("curr_arrXMIN", curr_arrXMAX);
            NAVCore.println("curr_arrXMAX", curr_arrXMIN);
            NAVCore.println("curr_arrYMIN", curr_arrYMIN);
            NAVCore.println("curr_arrYMAX", curr_arrYMAX);
            NAVCore.println("prev_arrXMIN", prev_arrXMIN);
            NAVCore.println("prev_arrXMAX", prev_arrXMAX);
            NAVCore.println("prev_arrYMIN", prev_arrYMIN);
            NAVCore.println("prev_arrYMAX", prev_arrYMAX);
            NAVCore.println("");

            if (abs(curr_arrXMAX - prev_arrXMIN) < abs(curr_arrXMIN - prev_arrXMAX))
            {
                uint32_t diff = abs(curr_arrXMAX - prev_arrXMIN);
                uint32_t new_curr_arrXMAX = curr_arrXMAX + diff;
                uint32_t new_prev_arrXMIN = new_curr_arrXMAX + 1;
                NAVCore.println("NEW CURR ARR X MAX", new_curr_arrXMAX);
                NAVCore.println("NEW PREV ARR X MMIN", new_prev_arrXMIN);
            }
            else
            {
                uint32_t diff = abs(curr_arrXMAX - prev_arrXMIN);
                uint32_t new_prev_arrXMAX = prev_arrXMAX + diff;
                uint32_t new_curr_arrXMIN = new_prev_arrXMAX + 1;
                NAVCore.println("NEW PREV ARR X MAX", new_prev_arrXMAX);
                NAVCore.println("NEW CURR ARR X MMIN", new_curr_arrXMIN);
            }*/

            /*uint32_t lower_block = (counter < prev_block) ? counter : prev_block;
            uint32_t upper_block = (counter < prev_block) ? prev_block : counter;
            if (lower_block != counter)
                readBlock(lower_block);
            uint32_t lower_pos_arrXMAX = (navmap.arrX[255] < 0) ? navmap.arrX[255] * -1 : navmap.arrX[255];
            uint32_t lower_pos_arrYMAX = (navmap.arrY[255] < 0) ? navmap.arrY[255] * -1 : navmap.arrY[255];
            readBlock(upper_block);
            uint32_t upper_pos_arrXMIN = (navmap.arrX[0] < 0) ? navmap.arrX[0] * -1 : navmap.arrX[0];
            uint32_t upper_pos_arrYMIN = (navmap.arrY[0] < 0) ? navmap.arrY[0] * -1 : navmap.arrY[0];

            NAVCore.println("pos_xvector", pos_xvector);
            NAVCore.println("pos_yvector", pos_yvector);
            NAVCore.println("lower_pos_arrXMAX", lower_pos_arrXMAX);
            NAVCore.println("lower_pos_arrYMAX", lower_pos_arrYMAX);
            NAVCore.println("upper_pos_arrXMIN", upper_pos_arrXMIN);
            NAVCore.println("upper_pos_arrYMIN", upper_pos_arrYMIN);

            if (pos_xvector > lower_pos_arrXMAX && pos_xvector < upper_pos_arrXMIN)
            {
                // aggiungi a lower arr min
                uint32_t value_toadd = abs(upper_pos_arrXMIN - lower_pos_arrXMAX) / 2;
                uint32_t new_upper_pos_arrXMIN = lower_pos_arrXMAX + value_toadd + 1;

                NAVCore.println("VALUE TO ADD TO LOWER ARRX MIN", value_toadd);
                NAVCore.println("new_upper_pos_arrXMIN", new_upper_pos_arrXMIN);
            }

            if (pos_yvector > lower_pos_arrYMAX && pos_yvector < upper_pos_arrYMIN)
            {
                // aggiungi a lower arr min
                uint32_t value_toadd = abs(upper_pos_arrYMIN - lower_pos_arrYMAX) / 2;
                uint32_t new_upper_pos_arrYMIN = lower_pos_arrYMAX + value_toadd + 1;

                NAVCore.println("VALUE TO ADD TO LOWER ARRY MIN", value_toadd);
                NAVCore.println("new_upper_pos_arrYMIN", new_upper_pos_arrYMIN);
            }*/

            /*for (int i = 0; i < read_blocks.size(); i++)
            {
                NAVCore.println("READ BLOCKS", read_blocks[i]);
            }
            ESP.restart();
        }
*/
        // CURR POS: 2624
        // BLOCK 1 MAX: 2578
        // BLOCK 2 MIN: 2772
        // NON COMPRESO IN NESSUNO DEI DUE, BISOGNA PORTARE IL BLOCCO 1 A 2675 E IL 2 A 2676
        // NON COMPRESO IN NESSUNO DEI DUE, BISOGNA PORTARE IL BLOCCO 1 A 2675 E IL 2 A 2676
        NAVCore.println("COUNTER", counter);
        NAVCore.println("int_xvector", int_xvector);
        NAVCore.println("int_yvector", int_yvector);
        NAVCore.println("MAX X", navmap.MaxX.value);
        NAVCore.println("MIN X", navmap.MinX.value);
        NAVCore.println("MAX Y", navmap.MaxY.value);
        NAVCore.println("MIN Y", navmap.MinY.value);
        if (!xblock_found)
        {
            if (int_xvector > navmap.MinX.value - 300)
            {
                if (int_xvector < navmap.MaxX.value + 300)
                {
                    xblock_idx = navmap.current_block;
                    for (int i = 0; i < 256; i++)
                        navmap.curr_arrX[i] = navmap.arrX[i];
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
            if (int_yvector > navmap.MinY.value - 300)
            {
                if (int_yvector < navmap.MaxY.value + 300)
                {
                    yblock_idx = navmap.current_block;
                    for (int i = 0; i < 256; i++)
                        navmap.curr_arrY[i] = navmap.arrY[i];
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

    if (xblock_idx != yblock_idx)
    {
        NAVCore.println(F("IL ROBOT SI TROVA IN UNA POSIZIONE NON MAPPATA"));
        return NOT_FOUND;
    }
    else
    {
        for (int i = 0; i < 256; i++)
            navmap.curr_arrID[i] = navmap.arrID[i];
    }

    resume();
    return navmap.current_block;
}

std::tuple<uint32_t, uint32_t, uint32_t> NAV::getClosestPointDst(uint32_t point_type)
{
    getCurrentBlock();

    uint32_t pos_xvector = (uint32_t)abs(vectors.xvector * 100);
    uint32_t pos_yvector = (uint32_t)abs(vectors.yvector * 100);

    uint32_t dst_to_closest_point = 1000000;
    uint32_t closest_point_idx = 0;
    for (int i = 0; i < 256; i++)
    {
        if (navmap.curr_arrID[i] != 3)
        {
            if ((navmap.curr_arrID[i] == point_type && point_type == 200) || point_type != 200)
            {
                int32_t xdiff = 0;
                int32_t xpoint = navmap.curr_arrX[i];
                xpoint *= (xpoint < 0) ? -1 : 1;
                xdiff = pos_xvector - xpoint;
                xdiff *= (xdiff < 0) ? -1 : 1;

                int32_t ydiff = 0;
                int32_t ypoint = navmap.curr_arrY[i];
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

    return std::make_tuple(dst_to_closest_point, navmap.curr_arrID[closest_point_idx], closest_point_idx);
}

void NAV::mapBorderMode(bool on_off)
{
    bordermode = on_off;
    bordermode_start_time = millis();
}

uint32_t NAV::getSDLastReadablePosition()
{
    mapfile = SD.open(F("/MAP.txt"));
    uint32_t pos = 0;
    while (mapfile.available())
        mapfile.read();
    pos = mapfile.position();
    mapfile.close();

    return pos;
}

void NAV::sdspeedtest()
{
    char buffer[5632] = {};
    uint32_t start_time = millis();
    mapfile = SD.open("/MAP.txt");
    mapfile.readBytes(buffer, 5632);
    /*uint8_t bytes_read = 0;
    uint8_t data_type = 0;
    uint8_t n_data_read = 0;
    char buffer[256];
    while (mapfile.available())
    {
        buffer[bytes_read] = mapfile.read();
        if (buffer[bytes_read] == ',')
        {
            buffer[bytes_read] = '\0';
            int32_t data;
            sscanf(buffer, "%d", &data);
            if (data_type == 0)
                *(navmap->arrX + n_data_read) = data;
            else if (data_type == 1)
                *(navmap->arrY + n_data_read) = data;
            else if (data_type == 2)
            {
                *(navmap->arrID + n_data_read) = data;
                data_type = 0;
                n_data_read++;
                continue;
            }

            data_type++;

            bytes_read = 0;
        }
        else
            bytes_read++;
    }*/
    mapfile.close();
    Serial.println("TIME");
    Serial.println(millis() - start_time);
    for (int i = 0; i < 5632; i++)
        Serial.print(buffer[i]);
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

uint32_t NAV::getLastBlock(bool fill)
{
    uint32_t last_f_block = 0;
    uint32_t last_incmplete_block = 0;
    uint32_t max_readable_pos = navutil.last_readable_pos;
    uint32_t remainder = max_readable_pos % MAP_BLOCK_SIZE;
    if (remainder == 0)
        last_f_block = max_readable_pos / MAP_BLOCK_SIZE;
    else
    {
        last_incmplete_block = std::ceil(max_readable_pos / (float)MAP_BLOCK_SIZE);
        last_f_block = last_incmplete_block - 1;
    }

    if (fill && last_incmplete_block != 0)
    {
        uint32_t target_position = MAP_BLOCK_SIZE * last_incmplete_block - 1;
        uint32_t cursor = max_readable_pos;
        mapfile = SD.open(F("/MAP.txt"), FILE_APPEND);
        mapfile.seek(cursor);
        while (cursor <= target_position)
        {
            mapfile.print(F("000000000,000000000,3,"));
            cursor = mapfile.position();
        }
        mapfile.close();
    }

    navutil.last_full_block = last_f_block;
    navutil.last_incomplete_block = last_incmplete_block;

    return last_f_block;
}

std::tuple<uint32_t, uint32_t, int32_t> NAV::getTopPoint(uint8_t axis)
{
    uint32_t max_block = getLastBlock();
    uint32_t current_block = 1;
    uint32_t block_idx, p_idx = 0;
    int32_t p_value = 0;
    while (current_block <= max_block)
    {
        readBlock(current_block);
        
        if (axis == X)
        {
            if (p_value < navmap.MaxX.value)
            {
                block_idx = navmap.MaxX.block;
                p_idx = navmap.MaxX.idx;
                p_value = navmap.MaxX.value;
            }
        }
        else
        {
            if (p_value < navmap.MaxY.value)
            {
                block_idx = navmap.MaxY.block;
                p_idx = navmap.MaxY.idx;
                p_value = navmap.MaxY.value;
            }
        }

        current_block++;
    }

    return std::make_tuple(block_idx, p_idx, p_value);
}

std::tuple<uint32_t, uint32_t, int32_t> NAV::getBottomPoint(uint8_t axis)
{
    uint32_t max_block = getLastBlock();
    uint32_t current_block = 1;
    uint32_t block_idx, p_idx = 0;
    int32_t p_value = 1000000000;
    while (current_block <= max_block)
    {
        readBlock(current_block);

        if (axis == X)
        {
            if (p_value > navmap.MinX.value)
            {
                block_idx = navmap.MinX.block;
                p_idx = navmap.MinX.idx;
                p_value = navmap.MinX.value;
            }
        }
        else
        {
            if (p_value > navmap.MinY.value)
            {
                block_idx = navmap.MinY.block;
                p_idx = navmap.MinY.idx;
                p_value = navmap.MinY.value;
            }
        }

        current_block++;
    }

    return std::make_tuple(block_idx, p_idx, p_value);
}

std::tuple<uint32_t, uint32_t, int32_t> NAV::getTopPointOf(uint32_t block, uint8_t axis)
{
    uint32_t block_idx, p_idx = 0;
    int32_t p_value = 0;
    readBlock(block);
    if (axis == X)
    {
        if (p_value < navmap.MaxX.value)
        {
            p_idx = navmap.MaxX.idx;
            p_value = navmap.MaxX.value;
        }
    }
    else
    {
        if (p_value < navmap.MaxY.value)
        {
            p_idx = navmap.MaxY.idx;
            p_value = navmap.MaxY.value;
        }
    }

    return std::make_tuple(block_idx, p_idx, p_value);
}

std::tuple<uint32_t, uint32_t, int32_t> NAV::getBottomPointOf(uint32_t block, uint8_t axis)
{
    uint32_t block_idx, p_idx = 0;
    int32_t p_value = 0;
    readBlock(block);
    if (axis == X)
    {
        if (p_value > navmap.MinX.value)
        {
            p_idx = navmap.MinX.idx;
            p_value = navmap.MinX.value;
        }
    }
    else
    {
        if (p_value > navmap.MinY.value)
        {
            p_idx = navmap.MinY.idx;
            p_value = navmap.MinY.value;
        }
    }

    return std::make_tuple(block_idx, p_idx, p_value);
}

std::tuple<int32_t, int32_t> NAV::getPointXY(uint32_t block, uint32_t point_idx)
{
    uint32_t block_position = (block == 1) ? 0 : MAP_BLOCK_SIZE * (block - 1) - 1;
    uint32_t idx_position = point_idx * 22;
    uint32_t seek_position = block_position + idx_position;
    int32_t x = 0;
    int32_t y = 0;
    mapfile = SD.open("/MAP.txt");
    mapfile.seek(seek_position);
    for (int i = 0; i < 2; i++)
    {
        if (i == 0)
            x = mapfile.parseInt();
        else
            y = mapfile.parseInt();
    }
    mapfile.close();

    return std::make_tuple(x, y);
}

uint16_t NAV::convertFromRef0ToRelativeHDG(uint16_t hdg, uint16_t ref, bool add)
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

    uint32_t pos_xvector = (uint32_t)abs(vectors.xvector * 100);
    uint32_t pos_yvector = (uint32_t)abs(vectors.yvector * 100);
    int32_t int_xvector = (int32_t)(vectors.xvector * 100);
    int32_t int_yvector = (int32_t)(vectors.yvector * 100);

    int32_t xdiff = 0;
    int32_t xpoint = target_x;
    xpoint = abs(xpoint);
    xdiff = pos_xvector - xpoint;
    xdiff = abs(xdiff);

    int32_t ydiff = 0;
    int32_t ypoint = target_y;
    ypoint = abs(ypoint);
    ydiff = pos_yvector - ypoint;
    ydiff = abs(ydiff);

    float angle = degrees(atan((float)(xdiff) / (float)(ydiff)));
    uint16_t target_heading = 0;
    if (target_x > int_xvector && target_y > int_yvector) // alto destra
        target_heading = angle;
    else if (target_x > int_xvector && target_y < int_yvector) // basso destra
        target_heading = convertFromRef0ToRelativeHDG((uint16_t)(angle * 100), 90, false);
    else if (target_x < int_xvector && target_y < int_yvector) // basso sinistra
        target_heading = convertFromRef0ToRelativeHDG((uint16_t)(angle * 100), 180, true);
    else if (target_x < int_xvector && target_y > int_yvector) // alto sinistra
        target_heading = convertFromRef0ToRelativeHDG((uint16_t)(angle * 100), 270, false);

    NAVCore.println("angle", angle);

    uint32_t current_heading = getHeading360();
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

    uint32_t seek_position = ((block == 1) ? 0 : MAP_BLOCK_SIZE * (block - 1) - 1) + point_idx * 22;
    int32_t px = 0;
    int32_t py = 0;
    mapfile = SD.open("/MAP.txt");
    mapfile.seek(seek_position);
    px = mapfile.parseInt();
    py = mapfile.parseInt();
    mapfile.close();

    uint32_t pos_xvector = (uint32_t)abs(vectors.xvector * 100);
    uint32_t pos_yvector = (uint32_t)abs(vectors.yvector * 100);
    int32_t int_xvector = (int32_t)(vectors.xvector * 100);
    int32_t int_yvector = (int32_t)(vectors.yvector * 100);

    int32_t xdiff = 0;
    int32_t ydiff = 0;
    xdiff = pos_xvector - abs(px);
    ydiff = pos_yvector - abs(py);

    float angle = degrees(atan((float)(abs(xdiff)) / (float)(abs(ydiff))));
    uint16_t target_heading = 0;
    if (px > int_xvector && py > int_yvector) // alto sinistra
        target_heading = angle;
    else if (px > 0 && py < int_yvector) // basso destra
        target_heading = convertFromRef0ToRelativeHDG((uint16_t)(angle * 100), 90, false);
    else if (px < 0 && py < int_yvector) // basso sinistra
        target_heading = convertFromRef0ToRelativeHDG((uint16_t)(angle * 100), 180, false);
    else if (px < 0 && py > int_yvector) // alto sinistra
        target_heading = convertFromRef0ToRelativeHDG((uint16_t)(angle * 100), 270, false);

    uint32_t current_heading = getHeading360();
    int32_t target_heading_180 = convertHDGTo180(target_heading);

    rotateToDeg(target_heading, getRotationDirection(current_heading, target_heading_180));

    return target_heading;
}

uint32_t NAV::getPointDst(uint32_t block, uint32_t point_idx)
{
    uint32_t seek_position = ((block == 1) ? 0 : MAP_BLOCK_SIZE * (block - 1) - 1) + point_idx * 22;
    int32_t px = 0;
    int32_t py = 0;
    mapfile = SD.open("/MAP.txt");
    mapfile.seek(seek_position);
    px = mapfile.parseInt();
    py = mapfile.parseInt();
    mapfile.close();

    uint32_t pos_xvector = (uint32_t)abs(vectors.xvector * 100);
    uint32_t pos_yvector = (uint32_t)abs(vectors.yvector * 100);

    int32_t xdiff = 0;
    int32_t ydiff = 0;
    xdiff = pos_xvector - abs(px);
    ydiff = pos_yvector - abs(py);

    uint32_t dst = sqrt(pow(abs(xdiff), 2) + pow(abs(ydiff), 2)); // pitagora

    return dst;
}

uint32_t NAV::getPointDst(int32_t x, int32_t y)
{
    uint32_t pos_xvector = (uint32_t)abs(vectors.xvector * 100);
    uint32_t pos_yvector = (uint32_t)abs(vectors.yvector * 100);

    int32_t xdiff = 0;
    int32_t ydiff = 0;
    xdiff = pos_xvector - abs(x);
    ydiff = pos_yvector - abs(y);

    uint32_t dst = sqrt(pow(abs(xdiff), 2) + pow(abs(ydiff), 2)); // pitagora

    return dst;
}

void NAV::goToPoint(int32_t x, int32_t y)
{
    NAVCore.println("HEADING TO POINT TO", setHeadingToPoint(x, y));
    NAVCore.println("POINT DST", getPointDst(x, y));
}

void NAV::goToPoint(uint32_t block, uint32_t point_idx)
{
    setHeadingToPoint(block, point_idx);
    goForward(getPointDst(block, point_idx));
}

void NAV::scroll()
{
    int32_t top_blk_x, top_pidx_x, top_pval_x = 0;
    int32_t top_blk_y, top_pidx_y, top_pval_y = 0;
    int32_t bot_blk_x, bot_pidx_x, bot_pval_x = 0;

    pause();
    std::tie(top_blk_x, top_pidx_x, top_pval_x) = getTopPoint(X);
    std::tie(top_blk_y, top_pidx_y, top_pval_y) = getTopPoint(Y);
    std::tie(bot_blk_x, bot_pidx_x, bot_pval_x) = getBottomPoint(X);
    resume();

    int32_t starting_point_x = bot_pval_x;
    int32_t starting_point_y = top_pval_y;
    int32_t end_point_x = top_pval_x;
    int32_t end_point_y = top_pval_y;
    NAVCore.println("x vector", vectors.xvector);
    NAVCore.println("y vector", vectors.yvector);

    goToPoint(1, 1);
    /*int32_t top_x_x, top_x_y = 0;
    int32_t bot_x_x, bot_x_y = 0;
    int32_t top_y_x, top_y_y = 0;
    int32_t bot_y_x, bot_y_y = 0;
    std::tie(top_x_x, top_x_y) = getPointXY(top_blk_x, top_pidx_x);
    std::tie(bot_x_x, bot_x_y) = getPointXY(bot_blk_x, bot_pidx_x);
    std::tie(top_y_x, top_y_y) = getPointXY(top_blk_y, top_pidx_y);
    std::tie(bot_y_x, bot_y_y) = getPointXY(bot_blk_y, bot_pidx_y);
    NAVCore.println("top_x_x", top_x_x);
    NAVCore.println("top_x_y", top_x_y);
    NAVCore.println("bot_x_x", bot_x_x);
    NAVCore.println("bot_x_y", bot_x_y);
    NAVCore.println("top_y_x", top_y_x);
    NAVCore.println("top_y_y", top_y_y);
    NAVCore.println("bot_y_x", bot_y_x);
    NAVCore.println("bot_y_y", bot_y_y);*/
}

uint32_t NAV::joinMapBlocks()
{
    /*uint32_t last_block = navutil.last_full_block;
    uint32_t current_block = 1;
    struct
    {
        int32_t maxX = 0;
        int32_t maxX_idx = 0;
        int32_t maxY = 0;
        int32_t maxY_idx = 0;
        int32_t minX = 0;
        int32_t minX_idx = 0;
        int32_t minY = 0;
        int32_t minY_idx = 0;
    } current;

    struct
    {
        int32_t maxX = 0;
        int32_t maxX_idx = 0;
        int32_t maxY = 0;
        int32_t maxY_idx = 0;
        int32_t minX = 0;
        int32_t minX_idx = 0;
        int32_t minY = 0;
        int32_t minY_idx = 0;
    } previous;

    while (current_block <= last_block)
    {
        readBlock(current_block);
        NAVCore.println("MAX X", navmap.arrX[255]);
        NAVCore.println("MIN X", navmap.arrX[0]);
        NAVCore.println("MAX Y", navmap.arrY[255]);
        NAVCore.println("MIN Y", navmap.arrY[0]);
        previous.maxX = navmap.arrX[255];
        previous.maxY = navmap.arrY[255];
        previous.minX = navmap.arrX[0];
        previous.minY = navmap.arrY[0];
        readBlock(current_block+1);
        NAVCore.println("MAX X", navmap.arrX[255]);
        NAVCore.println("MIN X", navmap.arrX[0]);
        NAVCore.println("MAX Y", navmap.arrY[255]);
        NAVCore.println("MIN Y", navmap.arrY[0]);
        current.maxX = navmap.arrX[255];;
        current.maxY = navmap.arrY[255];;
        current.minX = navmap.arrX[0];;
        current.minY = navmap.arrY[0];;
        uint32_t x_diff = abs(abs(previous.maxX) - abs(current.minX));
        if (x_diff != 1)
        {
            if (previous.maxX > current.minX)
            {
                int32_t val_toadd = (previous.maxX - current.minX) / 2;
                NAVCore.println("REAL CURR MINX", current.minX + val_toadd);
                NAVCore.println("REAL PREV MAXX", current.minX + val_toadd + 1);
            }
            else
            {
                int32_t val_toadd = (current.minX - previous.maxX) / 2;
                NAVCore.println("REAL PREV MAXX", previous.maxX + val_toadd);
                NAVCore.println("REAL CURR MINX", previous.maxX + val_toadd+ 1);
            }
        }
        uint32_t y_diff = abs(abs(previous.maxY) - abs(current.minY));
        if (y_diff != 1)
        {
            if (previous.maxY > current.minY)
            {
                int32_t val_toadd = (previous.maxY - current.minY) / 2;
                NAVCore.println("REAL CURR MINY", current.minY + val_toadd);
                NAVCore.println("REAL PREV MAXY", current.minY + val_toadd + 1);
            }
            else
            {
                int32_t val_toadd = (current.minY - previous.maxY) / 2;
                NAVCore.println("REAL PREV MAXY", previous.maxY + val_toadd);
                NAVCore.println("REAL CURR MINY", previous.maxY + val_toadd + 1);
            }
        }

        current_block++;
    }*/
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

int32_t NAV::convertCharArrToInt(std::array<char, 256> a)
{
    uint8_t i = 0;
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
    mots.stall = true;
}

void NAV::begin()
{
    spi = new SPIClass(VSPI);
    spi->begin(SCLK, MISO, MOSI, CS);
    if (SD.begin(CS, *spi))
        NAVCore.println(F("--- Scheda SD OK ---"));
    else
    {
        NAVCore.println(F("--- Scheda SD NON RICONOSCIUTA, RESTART ---"));
        ESP.restart();
    }

    mapfile = SD.open("/MAP.txt");
    navutil.last_readable_pos = getSDLastReadablePosition();
    getLastBlock(true);
    navutil.last_readable_pos = getSDLastReadablePosition();
    NAVCore.println("LAST SD POSITION", navutil.last_readable_pos);
    joinMapBlocks();
    readBlock(1);
}
