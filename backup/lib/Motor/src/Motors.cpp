#include <Motors.h>
#include <Sensors.h>
#include <Status.h>
#include <BluetoothSerial.h>
#include <Core.h>
#include <Navigation.h>
#include <Mux.h>

Sensors motorsensor;
Status motorstatus;
Core motorscore;
uint32_t t1 = 0;

struct MainMotor
{
    struct
    {
        bool active = false;
        uint8_t note_n = 0;
        uint32_t time = 0;
    } playsound;

    struct
    {
        bool active = false;
        uint32_t time = 0;
        uint16_t speed = 0;
        uint16_t speed_target = 0;
    } startup;

    bool active = false;
};

MainMotor mainmotor;

const float wheel_circumference = WHEEL_DIAMETER * 3.14;
const float wheel_encoder_ratio = WHEEL_DIAMETER / ENCODER_DIAMETER;

uint8_t left_spd = MOT_MAX_VAL;
uint8_t right_spd = MOT_MAX_VAL;

bool robot_moving = false;
bool move = false;
int32_t heading_to_maintain;
bool maintain_heading = false;
char direction;
bool reverse_heading = false;

uint32_t time_1 = millis();
uint32_t time_2 = millis();
uint32_t time_wait = millis();

uint32_t Motors::getTime()
{
    return t1;
}

void Motors::forward(int32_t new_hdg)
{
    stop();
    //motorsensor.checkFrontObstacle(MOTORS); // manda subito una richiesta per controllare se c'è un ostacolo
    motorscore.println((char *)"(Motors.cpp) MOTORS FORWARD");
    motorstatus.setRunning(true);

    if (!maintain_heading)
    {
        left_spd = MOT_MAX_VAL;
        right_spd = MOT_MAX_VAL;
        digitalWrite(MOTOR_RIGHT_DIR, FWD);
        digitalWrite(MOTOR_LEFT_DIR, FWD);
        ledcWrite(CHANNEL_RIGHT, MOT_MAX_VAL);
        ledcWrite(CHANNEL_LEFT, MOT_MAX_VAL);
        if (new_hdg != AUTO)
            heading_to_maintain = new_hdg;
        else
            heading_to_maintain = motorsensor.getHeading();
        maintain_heading = true;
        robot_moving = true;
        reverse_heading = false;
    }

    direction = 'w';
}

void Motors::backwards(int32_t new_hdg)
{
    stop();
    motorscore.println((char *)"(Motors.cpp) MOTORS BACKWARDS");
    motorstatus.setRunning(true);

    if (!maintain_heading)
    {
        left_spd = MOT_MAX_VAL;
        right_spd = MOT_MAX_VAL;
        digitalWrite(MOTOR_RIGHT_DIR, BCK);
        digitalWrite(MOTOR_LEFT_DIR, BCK);
        ledcWrite(CHANNEL_RIGHT, MOT_MAX_VAL);
        ledcWrite(CHANNEL_LEFT, MOT_MAX_VAL);
        if (new_hdg != AUTO)
            heading_to_maintain = new_hdg;
        else
            heading_to_maintain = motorsensor.getHeading();
        maintain_heading = true;
        robot_moving = true;
        reverse_heading = true;
        time_wait = millis();
    }

    direction = 's';
}

void Motors::right(bool pivot)
{
    stop();
    digitalWrite(MOTOR_RIGHT_DIR, BCK);
    digitalWrite(MOTOR_LEFT_DIR, FWD);
    if (!pivot)
        ledcWrite(CHANNEL_RIGHT, MOT_NORM_VAL);
    else
        ledcWrite(CHANNEL_RIGHT, 0);
    ledcWrite(CHANNEL_LEFT, MOT_NORM_VAL);
    motorstatus.setRunning(true);
    direction = 'd';
    robot_moving = true;
    maintain_heading = false;
    time_wait = millis();
}

void Motors::left(bool pivot)
{
    stop();
    digitalWrite(MOTOR_RIGHT_DIR, FWD);
    digitalWrite(MOTOR_LEFT_DIR, BCK);
    ledcWrite(CHANNEL_RIGHT, MOT_NORM_VAL);
    if (!pivot)
        ledcWrite(CHANNEL_LEFT, MOT_NORM_VAL);
    else
        ledcWrite(CHANNEL_LEFT, 0);
    motorstatus.setRunning(true);
    direction = 'a';
    robot_moving = true;
    maintain_heading = false;
}

void Motors::stop()
{
    motorstatus.setReady(true);
    ledcWrite(CHANNEL_RIGHT, 0);
    ledcWrite(CHANNEL_LEFT, 0);
    direction = 't';
    robot_moving = false;
    maintain_heading = false;
    motorsensor.resetMovementVars();
    motorsensor.setMotorsStop();
}

char Motors::getDirection()
{
    return direction;
}

void Motors::maintainHeading()
{
    // DESTRA: DIFF NEGATIVO; SINISTRA: DIFF POSITIVO

    int32_t current_heading = motorsensor.getHeading();
    int32_t diff = heading_to_maintain - current_heading;

    if (reverse_heading)
        diff *= -1; // inverte destra e sinistra se il robot sta andando indietro

    if (diff < 0)
    {
        // DESTRA

        diff *= -1;

        /*
        decadimento esponenziale
        funzione: 𝑦=𝑐+𝑎(1−𝑏)^𝑥
        dove:
        y = motor_value
        c = valore minimo motore (150, MOT_MIN_VAL)
        a = valore massimo motore (200 - 150 = 105; MOT_MAX_VAL - MOT_MIN_VAL = c)
        b = costante (rateo del decay, 0.642223)
        x = diff (floatdiff)
        */

        float floatdiff = (float)diff / 100;
        uint8_t motor_value = MOT_MIN_VAL + ((MOT_MAX_VAL - MOT_MIN_VAL) * pow((0.35777), floatdiff));
        //if (motor_value > MOT_MIN_VAL)
        //{
            left_spd = motor_value;
            right_spd = MOT_MAX_VAL;
        //}
    }
    else
    {
        // SINISTRA

        float floatdiff = (float)diff / 100;
        uint8_t motor_value = MOT_MIN_VAL + ((MOT_MAX_VAL - MOT_MIN_VAL) * pow((0.35777), floatdiff));
        //if (motor_value > MOT_MIN_VAL)
        //{
            right_spd = motor_value;
            left_spd = MOT_MAX_VAL;
        //}
    }


    ledcWrite(CHANNEL_RIGHT, right_spd);
    ledcWrite(CHANNEL_LEFT, left_spd);
}

void Motors::setSpeed(uint8_t spd, uint8_t motor)
{
    if (motor == RIGHT)
        right_spd = spd;
    else if (motor == LEFT)
        left_spd = spd;
    else if (motor == BOTH)
    {
        right_spd = spd;
        left_spd = spd;
    }
    else if (motor == MAIN)
    {
        if (spd > 20)
        {
            for (int i = 20; i < spd; i++)
            {
                delay(25);
                ledcWrite(CHANNEL_MAIN, i);
            }
        }
        else
            ledcWrite(CHANNEL_MAIN, spd);

        return;
    }

    ledcWrite(CHANNEL_RIGHT, right_spd);
    ledcWrite(CHANNEL_LEFT, left_spd);
}

void Motors::playStartSound()
{
    mainmotor.playsound.active = true;
}

bool Motors::toggleMainMotor(uint8_t spd, uint8_t stat)
{
    if (stat == TOGGLE)
    {
        if (mainmotor.active == false)
        {
            mainmotor.active = true;
            mainmotor.startup.active = true;
            mainmotor.startup.speed_target = spd;
            motorstatus.mainMotorStarting();
        }
        else
        {
            mainmotor.active = false;
            mainmotor.startup.active = false;
            mainmotor.startup.speed = 0;
            ledcWrite(CHANNEL_MAIN, 0);
        }
    }
    else if (stat == RUNNING)
    {
        mainmotor.active = true;
        mainmotor.startup.active = true;
        mainmotor.startup.speed_target = spd;
        motorstatus.mainMotorStarting();
    }
    else
    {
        mainmotor.active = false;
        mainmotor.startup.active = false;
        mainmotor.startup.speed = 0;
        ledcWrite(CHANNEL_MAIN, 0);
    }
    
    return mainmotor.active;
}

int32_t Motors::getHeadingToMaintain()
{
    return heading_to_maintain;
}

int32_t Motors::addToHeadingToMaintain(int32_t degs)
{
    heading_to_maintain += degs;
    if (heading_to_maintain > 18000)
        heading_to_maintain = heading_to_maintain - 36000;
    else if (heading_to_maintain < -18000)
        heading_to_maintain = heading_to_maintain + 36000;

    return heading_to_maintain;
}

void Motors::begin()
{
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);
    ledcAttachPin(MOT_R_SPD, CHANNEL_RIGHT);
    ledcAttachPin(MOT_L_SPD, CHANNEL_LEFT);
    ledcSetup(CHANNEL_RIGHT, MOVEMENT_MOT_FREQ, 8);
    ledcSetup(CHANNEL_LEFT, MOVEMENT_MOT_FREQ, 8);
    pinMode(23, INPUT);
    pinMode(MOT_MAIN, OUTPUT);
    ledcAttachPin(MOT_MAIN, CHANNEL_MAIN);
    playStartSound();
}

void Motors::update()
{
    uint64_t start_time = micros();

    if (ENABLE_ROTATION_SENSING)
    {
        if (maintain_heading)
        {
            if (millis() - time_wait > 500)
            {
                time_2 = millis();

                if (time_2 - time_1 > 150)
                {
                    time_1 = millis();
                    maintainHeading();
                }
            }
        }
    }

    if (mainmotor.playsound.active)
    {
        switch (mainmotor.playsound.note_n)
        {
        case 0:
        {
            ledcSetup(CHANNEL_MAIN, 496, 8);
            ledcWrite(CHANNEL_MAIN, 7);
            mainmotor.playsound.time = millis();
            mainmotor.playsound.note_n++;
            break;
        }
        case 1:
        {
            if (millis() - mainmotor.playsound.time > 200)
            {
                ledcSetup(CHANNEL_MAIN, 624, 8);
                ledcWrite(CHANNEL_MAIN, 7);
                mainmotor.playsound.time = millis();
                mainmotor.playsound.note_n++;
            }
            break;
        }
        case 2:
        {
            if (millis() - mainmotor.playsound.time > 200)
            {
                ledcSetup(CHANNEL_MAIN, 990, 8);
                ledcWrite(CHANNEL_MAIN, 7);
                mainmotor.playsound.time = millis();
                mainmotor.playsound.note_n++;
            }
            break;
        }
        case 3:
        {
            if (millis() - mainmotor.playsound.time > 200)
            {
                ledcSetup(CHANNEL_MAIN, 936, 8);
                ledcWrite(CHANNEL_MAIN, 7);
                mainmotor.playsound.time = millis();
                mainmotor.playsound.note_n++;
            }
            break;
        }
        case 4:
        {
            if (millis() - mainmotor.playsound.time > 250)
            {
                ledcWrite(CHANNEL_MAIN, 0);
                mainmotor.playsound.time = millis();
                mainmotor.playsound.note_n++;
            }
            break;
        }
        case 5:
        {
            if (millis() - mainmotor.playsound.time > 150)
            {
                ledcSetup(CHANNEL_MAIN, 744, 8);
                ledcWrite(CHANNEL_MAIN, 7);
                mainmotor.playsound.time = millis();
                mainmotor.playsound.note_n++;
            }
            break;
        }
        case 6:
        {
            if (millis() - mainmotor.playsound.time > 250)
            {
                ledcSetup(CHANNEL_MAIN, MAIN_MOT_FREQ, 8);
                ledcWrite(CHANNEL_MAIN, 0);
                mainmotor.playsound.active = false;
                mainmotor.playsound.note_n = 0;
            }
            break;
        }
        }
    }

    if (mainmotor.startup.active)
    {
        if (millis() - mainmotor.startup.time > 25)
        {
            ledcWrite(CHANNEL_MAIN, mainmotor.startup.speed);
            mainmotor.startup.speed++;
            mainmotor.startup.time = millis();
        }

        if (mainmotor.startup.speed > mainmotor.startup.speed_target)
        {
            mainmotor.startup.active = false;
            mainmotor.startup.speed = 0;
            mainmotor.startup.speed_target = 0;
            mainmotor.startup.time = 0;
        }
    }

    t1 = micros() - start_time;
}
