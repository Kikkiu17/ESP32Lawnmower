#include <Navigation.h>
#include <Sensors.h>
#include <Motors.h>
#include <Core.h>
#include <BluetoothSerial.h>
#ifdef ENABLE_WEBSERIAL
#include <WebSerial.h>
#endif
#include <SETTINGS.h>

Sensors NAVSensors;
Motors NAVMotors;
Core NAVCore;
BluetoothSerial NAVSerial;
uint32_t t3 = 0;

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

int8_t quadrant = 0;
int8_t coarse_dir = 0;
float vector_slope = 0.00;

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
uint32_t timer = millis();
uint8_t obstacle_sensor_direction = FRONT;
uint8_t obstacle_sensor_type = INFRARED;

// stage di movimento
uint8_t stages = 0;

struct Vectors
{
    float xvector = 0;
    float yvector = 0;
    float last_dst = 0;
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
    bool active = false;
    bool completed = false;
    float relative_right_angle = 0.00;
    bool lateral_obstacle_detected = false;
    uint8_t stages = 0;
    bool check_straight_path = false;
    uint16_t last_lateral_distance = 0;
    bool obstacle_detected = false;
} BorderMode;

Vectors vectors;

void NAV::update()
{
    uint64_t start_time = micros();

    if (going_forward)
    {
        distance_traveled = NAVSensors.getTraveledDistance();

        if (UTurn.active)
            distance_traveled_before_uturn_sudden_stop = distance_traveled;

        if (distance_traveled >= distance_target && distance_traveled != 0 && distance_target != 0)
        {
            NAVMotors.stop();
            stop();
            going_forward = false;
            gone_forward = true;
            distance_target_reached = true;
        }
    }

    if (going_backwards)
    {
        distance_traveled = NAVSensors.getTraveledDistance();

        if (distance_traveled >= distance_target && distance_traveled != 0 && distance_target != 0)
        {
            NAVMotors.stop();
            stop();
            going_backwards = false;
            gone_backwards = true;
            distance_target_reached = true;
        }
    }

    if (obstacle_detected)
    {
        NAVCore.println((char *)"(Navigation.cpp) OBSTACLE DETECTED");
        obstacle_detected = false;
        going_forward = false;
        // non è necessario NAVMotors.stop(), il robot viene fermato automaticamente dalla sensoristica

        if (autorun)
        {
            // queste azioni vengono fatte solo se il robot è in modalità AUTO
            // altrimenti, sono attivi solo i sensori (il robot si ferma a un'ostacolo)
            if (!BorderMode.active && !BorderMode.completed && ENABLE_BORDERMODE)
            {
                BorderMode.active = true;
                BorderMode.stages = 0;
                timer = millis();
            }
            else if (BorderMode.active && ENABLE_BORDERMODE)
                BorderMode.obstacle_detected = true;

            if (BorderMode.completed && ENABLE_BORDERMODE)
            {
                if (!UTurn.active)
                {
                    // attiva l'u-turn solo se questo non è in corso
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
            else if (!ENABLE_BORDERMODE)
            {
                if (!UTurn.active)
                {
                    // attiva l'u-turn solo se questo non è in corso
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
        }
    }

    if (obstacle_detected_before_moving)
    {
        obstacle_detected_before_moving = false;

        if (BorderMode.active)
            BorderMode.obstacle_detected = true;
    }

    if (BorderMode.active)
    {
        switch (BorderMode.stages)
        {
            case 0:
            {
                if (millis() - timer > GLOBAL_NAV_DELAY)
                {
                    BorderMode.stages++;
                    gone_backwards = false;
                    rotateForDeg(10);
                }
                break;
            }
            case 1:
            {
                if (rotated)
                {
                    if (millis() - timer > GLOBAL_NAV_DELAY)
                    {
                        BorderMode.stages++;
                        rotated = false;
                        goForward();
                    }
                }
                else
                    timer = millis();
                break;
            }
            case 2:
            {
                if (BorderMode.obstacle_detected)
                {
                    if (millis() - timer > GLOBAL_NAV_DELAY)
                    {
                        BorderMode.stages = 0;
                        BorderMode.obstacle_detected = false;
                    }
                }
                else if (BorderMode.lateral_obstacle_detected)
                {
                    if (millis() - timer > GLOBAL_NAV_DELAY)
                    {
                        BorderMode.stages++;
                        rotateUntil('r', &BorderMode.lateral_obstacle_detected, false);
                    }
                }
                else
                    timer = millis();
                break;
            }
            case 3:
            {
                if (RotateUntil.done)
                {
                    if (millis() - timer > GLOBAL_NAV_DELAY)
                    {
                        BorderMode.stages++;
                        RotateUntil.done = false;
                        goForward();
                    }
                }
                else
                    timer = millis();
                break;
            }
            case 4:
            {
                if (BorderMode.lateral_obstacle_detected)
                {
                    NAVMotors.setSpeed(150, RIGHT);
                    NAVMotors.setSpeed(255, LEFT);
                }
                else
                {
                    NAVMotors.setSpeed(150, LEFT);
                    NAVMotors.setSpeed(255, RIGHT);
                }
                break;
            }
        }
    }

    if (RotateUntil.active)
    {
        if (RotateUntil.dir == 'r')
            NAVMotors.right();
        else
            NAVMotors.left();

        if (*RotateUntil.var_ptr == RotateUntil.condition)
        {
            NAVMotors.stop();
            stop();
            RotateUntil.done = true;
            RotateUntil.active = false;
        }
    }

    if (sudden_stop)
    {
        sudden_stop = false;
        going_forward = false;
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
        uint16_t currentHDG = getHDG();

        if (currentHDG > heading_target - 50 && currentHDG < heading_target + 50)
        {
            NAVMotors.stop();
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
            if (diff < 3558)
            {
                spd = 140 + 3.2 * (diff / 100);
                NAVMotors.setSpeed(spd, BOTH);
            }
            else
                NAVMotors.setSpeed(255, BOTH);
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
                    goForward(10);
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
            if (gone_forward)
            {
                if (millis() - timer > GLOBAL_NAV_DELAY)
                {
                    stages++;
                    gone_forward = false;
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
            {
                timer = millis();
            }
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
        uint32_t hdg_360 = getHDG();
        float dst = NAVSensors.getLastTraveledDistance();

        /**
         * Heading in modailtà 180°
         * Il vettore X è positivo verso 90°
         * Il vettore Y è positivo verso 0°
         * |         0°         |
         * |     /       \      |
         * |  270°        90°   |
         * |     \       /      |
         * |        180°        |
         */

        if (vectors.last_dst != dst)
        {
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
                hdg_360 = convertRelativeHDG(hdg_360, 90);
                float rad = radians(hdg_360 / 100);
                vectors.xvector += dst * cos(rad); // positivo per X
                vectors.yvector -= dst * sin(rad); // negativo per Y
            }
            else if (hdg_360 > 18000 && hdg_360 < 27000) // heading negativo per X e Y -- basso sinistra -- riferimento 180°
            {
                if (going_backwards)
                    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertRelativeHDG(hdg_360, 180);
                float rad = radians(hdg_360 / 100);
                vectors.xvector -= dst * sin(rad); // negativo per X
                vectors.yvector -= dst * cos(rad); // negativo per Y
            }
            else // heading positivo per Y, negativo per X -- alto sinistra -- riferimento 270°
            {
                if (going_backwards)
                    hdg_360 = invertHDG(hdg_360);
                hdg_360 = convertRelativeHDG(hdg_360, 270);
                float rad = radians(hdg_360 / 100);
                vectors.xvector -= dst * cos(rad); // negativo per X
                vectors.yvector += dst * sin(rad); // positivo per Y
            }
        }
    }

    t3 = micros() - start_time;
}

uint32_t NAV::getTime()
{
    return t3;
}

void NAV::goForward(uint32_t cm)
{
    NAVSensors.resetTraveledDistance();
    NAVMotors.forward();
    distance_target = cm;
    resetMovementVars();
    going_forward = true;
    robot_moving_x_y = true;
}

void NAV::goBackwards(uint32_t cm)
{
    NAVSensors.resetTraveledDistance();
    NAVMotors.backwards();
    distance_target = cm;
    resetMovementVars();
    going_backwards = true;
    robot_moving_x_y = true;
}

void NAV::rotateForDeg(int16_t heading)
{
    /*
            0°
        /       \
    270°           90°
        \       /
           180°
    */

    resetMovementVars();

    if (heading == 90)
    {
        if (coarse_dir + 1 <= 3)
            coarse_dir++;
        else
            coarse_dir = 0;
    }
    else
    {
        if (coarse_dir - 1 >= 0)
            coarse_dir--;
        else
            coarse_dir = 3;
    }

    int16_t heading_to_reach = heading * 100;
    if (heading_to_reach < 0)
        heading_to_reach = heading_to_reach + 36000; // 36000
    start_heading = getHDG();
    int32_t over_start_heading = start_heading + heading_to_reach;
    if (over_start_heading - 36000 < 0)
        heading_target = over_start_heading;
    else
        heading_target = over_start_heading - 36000;

    rotating = true;

    if (heading < 0)
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

void NAV::rotateToDeg(uint16_t HEADING, char DIRECTION)
{
    resetMovementVars();
    rotating = true;
    heading_target = HEADING;
    if (DIRECTION == 'L')
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
    BorderMode.active = false;
    BorderMode.check_straight_path = false;
    BorderMode.completed = false;
    BorderMode.lateral_obstacle_detected = false;
    stop();
    NAVMotors.stop();
    resetMovementVars();
    UTurn.active = false;
    autorun = false;
    NAVSensors.setAutoRun(false);
    NAVSensors.resetMovementVars();
    robot_moving_x_y = false;
}

void NAV::checkDirectionsDeg()
{
    float current_heading = getHDG();

    /*
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
            ----------------------------------
    */

    if (current_heading >= 0 && current_heading < 90)
    {
        // 0 < x < 90: quadrant = 0;
        vector_slope = 90 - current_heading;
        quadrant = 0;
    }
    else if (current_heading >= 90 && current_heading < 180)
    {
        // 90 < x < 180: quadrant = 1;
        vector_slope = current_heading - 90;
        quadrant = 1;
    }
    else if (current_heading >= 180 && current_heading < 270)
    {
        // 180 < x < 270: quadrant = 2;
        vector_slope = current_heading - 180;
        quadrant = 2;
    }
    else if (current_heading >= 270 && current_heading < 360)
    {
        // 270 < x < 360: quadrant = 3
        vector_slope = current_heading - 270;
        quadrant = 3;
    }
}

uint16_t NAV::getHDG()
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
    gone_forward = false;
    rotated = false;
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

char NAV::getRotationDirection(uint16_t current_hdg, uint16_t target_hdg)
{
    // imposta l'heading corrente come 0 e cambia il target heading relativamente al nuovo 0

    /*
    esempio: current_hdg = 9000, target_hdg = 27000
    relative_target_hdg = 27000 - 9000 = 18000

    se l'heading relativo è positivo, la direzione è DESTRA (R);
    altrimenti, la direzione è SINISTRA (L)
    */

    int32_t relative_target_hdg = target_hdg - current_hdg;

    if(relative_target_hdg > 0)
        return 'R';
    else
        return 'L';
}

void NAV::rotateUntil(char dir, bool *var, bool condition)
{
    RotateUntil.condition = condition;
    RotateUntil.dir = dir;
    RotateUntil.var_ptr = var;
    RotateUntil.active = true;
}

void NAV::setLateralObstacle(bool state)
{
    BorderMode.lateral_obstacle_detected = state;
}

void NAV::obstacleDetectedBeforeMoving(uint8_t sensor_type, uint8_t sensor_direction)
{
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
    robot_moving_x_y = false;
    going_forward = false;
    going_backwards = false;
    rotating = false;
    distance_traveled = 0;
}

uint16_t NAV::convertRelativeHDG(uint16_t hdg, uint16_t reference)
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
