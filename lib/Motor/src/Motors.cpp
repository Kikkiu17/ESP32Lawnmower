#include <Motors.h>
#include <Sensors.h>
#include <Status.h>
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
        uint32_t time = 0;
        uint32_t speed = 600;
        uint32_t speed_target = 0;
    } startup;

    bool active = false;
};

class Note
{
    public:
        uint32_t freq = 0;
        uint32_t duration = 0;
        uint32_t intensity = 0;   // valore standard: 1815
        Note(uint32_t infreq, uint32_t induration, uint32_t inintensity = 800)
        {
            freq = infreq;
            duration = induration;
            intensity = inintensity;
        }
};

class SoundPlayer
{
    public:
        std::vector<Note>* notes = new std::vector<Note>();
        uint32_t time1 = 0;
        uint32_t time2 = 0;
        bool busy = false;
};

MainMotor mainmotor;
SoundPlayer* sp = new SoundPlayer();

const float wheel_circumference = WHEEL_DIAMETER * 3.14;
const float wheel_encoder_ratio = WHEEL_DIAMETER / ENCODER_DIAMETER;

uint32_t left_spd = MOT_MAX_VAL;
uint32_t right_spd = MOT_MAX_VAL;

bool robot_moving = false;
bool move = false;
int32_t heading_to_maintain;
bool maintain_heading = false;
char direction;
bool reverse_heading = false;

unsigned int time_1 = millis();
unsigned int time_2 = millis();
unsigned int time_wait = millis();
unsigned int direc = FWD;
bool ispivoting = false;

uint32_t Motors::getTime()
{
    return t1;
}

void Motors::forward(int32_t new_hdg)
{
    stop();
    motorscore.println(F("(Motors.cpp) MOTORS FORWARD"));
    motorstatus.setRunning(true);

    if (!maintain_heading)
    {
        left_spd = MOT_MAX_VAL;
        right_spd = MOT_MAX_VAL;
        ledcWrite(CH_R1, MOT_MAX_VAL);
        ledcWrite(CH_R2, 0);
        ledcWrite(CH_L1, MOT_MAX_VAL);
        ledcWrite(CH_L2, 0);
        if (new_hdg != AUTO)
            heading_to_maintain = new_hdg;
        else
            heading_to_maintain = motorsensor.getHeading();
        maintain_heading = true;
        robot_moving = true;
        reverse_heading = false;
    }

    ispivoting = false;
    direc = FWD;
    motorsensor.enablePositionEncoders();
    direction = 'w';
}

void Motors::backwards(int32_t new_hdg)
{
    stop();
    motorscore.println(F("(Motors.cpp) MOTORS BACKWARDS"));
    motorstatus.setRunning(true);

    if (!maintain_heading)
    {
        left_spd = MOT_MAX_VAL;
        right_spd = MOT_MAX_VAL;
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, MOT_MAX_VAL);
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, MOT_MAX_VAL);
        if (new_hdg != AUTO)
            heading_to_maintain = new_hdg;
        else
            heading_to_maintain = motorsensor.getHeading();
        maintain_heading = true;
        robot_moving = true;
        reverse_heading = true;
        time_wait = millis();
    }

    ispivoting = false;
    direc = BCK;
    motorsensor.enablePositionEncoders();
    direction = 's';
}

void Motors::right(bool pivot)
{
    stop();
    ledcWrite(CH_L1, MOT_MAX_VAL);
    ledcWrite(CH_L2, 0);
    if (!pivot)
    {
        motorscore.println(F("(Motors.cpp) MOTORS RIGHT"));
        right_spd = MOT_MAX_VAL;
        left_spd = MOT_MAX_VAL;
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, right_spd);
        ispivoting = false;
    }
    else
    {
        motorscore.println(F("(Motors.cpp) MOTORS RIGHT (PIVOT)"));
        right_spd = 0;
        left_spd = MOT_MAX_VAL;
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, right_spd);
        ispivoting = true;
    }
    motorstatus.setRunning(true);
    direction = 'd';
    robot_moving = true;
    maintain_heading = false;
    time_wait = millis();

    direc = RIGHT;
    motorsensor.enablePositionEncoders();
}

void Motors::left(bool pivot)
{
    stop();
    ledcWrite(CH_R1, MOT_MAX_VAL);
    ledcWrite(CH_R2, 0);
    if (!pivot)
    {
        motorscore.println(F("(Motors.cpp) MOTORS LEFT"));
        right_spd = MOT_MAX_VAL;
        left_spd = MOT_MAX_VAL;
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, left_spd);
        ispivoting = false;
    }
    else
    {
        motorscore.println(F("(Motors.cpp) MOTORS LEFT (PIVOT)"));
        right_spd = MOT_MAX_VAL;
        left_spd = 0;
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, left_spd);
        ispivoting = true;
    }

    motorstatus.setRunning(true);
    direction = 'a';
    robot_moving = true;
    maintain_heading = false;

    direc = LEFT;
    motorsensor.enablePositionEncoders();
}

void Motors::stop()
{
    motorstatus.setReady(true);
    ledcWrite(CH_R1, 0);
    ledcWrite(CH_R2, 0);
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, 0);
    //brake();
    direction = 't';
    robot_moving = false;
    maintain_heading = false;
    motorsensor.resetMovementVars();
    motorsensor.setMotorsStop();
    direc = STOP;
}

unsigned int Motors::getDirection()
{
    return direc;
}

void Motors::maintainHeading()
{
    // DESTRA: DIFF NEGATIVO; SINISTRA: DIFF POSITIVO

    int32_t current_heading = motorsensor.getHeading();
    int32_t diff = current_heading - heading_to_maintain;

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
        uint32_t motor_value = MOT_MIN_VAL + ((MOT_MAX_VAL - MOT_MIN_VAL) * pow((0.35777), floatdiff));
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
        uint32_t motor_value = MOT_MIN_VAL + ((MOT_MAX_VAL - MOT_MIN_VAL) * pow((0.35777), floatdiff));
        //if (motor_value > MOT_MIN_VAL)
        //{
            right_spd = motor_value;
            left_spd = MOT_MAX_VAL;
        //}
    }

    if (direc == FWD)
    {
        ledcWrite(CH_R1, right_spd);
        ledcWrite(CH_R2, 0);
        ledcWrite(CH_L1, left_spd);
        ledcWrite(CH_L2, 0);
    }
    else if (direc == BCK)
    {
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, right_spd);
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, left_spd);
    }
    else if (direc == RIGHT)
    {
        ledcWrite(CH_L1, left_spd);
        ledcWrite(CH_L2, 0);
        if (!ispivoting)
        {
            ledcWrite(CH_R1, 0);
            ledcWrite(CH_R2, right_spd);
        }
        else
        {
            ledcWrite(CH_R1, 0);
            ledcWrite(CH_R2, 0);
        }
    }
    else
    {
        ledcWrite(CH_R1, right_spd);
        ledcWrite(CH_R2, 0);
        if (!ispivoting)
        {
            ledcWrite(CH_L1, 0);
            ledcWrite(CH_L2, left_spd);
        }
        else
        {
            ledcWrite(CH_L1, 0);
            ledcWrite(CH_L2, 0);
        }
    }
}

void Motors::setSpeed(uint32_t spd, uint32_t motor)
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

    if (direc == FWD)
    {
        ledcWrite(CH_R1, right_spd);
        ledcWrite(CH_R2, 0);
        ledcWrite(CH_L1, left_spd);
        ledcWrite(CH_L2, 0);
    }
    else if (direc == BCK)
    {
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, right_spd);
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, left_spd);
    }
    else if (direc == RIGHT)
    {
        ledcWrite(CH_L1, left_spd);
        ledcWrite(CH_L2, 0);
        if (!ispivoting)
        {
            ledcWrite(CH_R1, 0);
            ledcWrite(CH_R2, right_spd);
        }
        else
        {
            ledcWrite(CH_R1, 0);
            ledcWrite(CH_R2, 0);
        }
    }
    else
    {
        ledcWrite(CH_R1, right_spd);
        ledcWrite(CH_R2, 0);
        if (!ispivoting)
        {
            ledcWrite(CH_L1, 0);
            ledcWrite(CH_L2, left_spd);
        }
        else
        {
            ledcWrite(CH_L1, 0);
            ledcWrite(CH_L2, 0);
        }
    }
}

void Motors::playStartSound()
{
    Note note1 = Note(496, 200);
    Note note2 = Note(624, 200);
    Note note3 = Note(990, 200);
    Note note4 = Note(936, 250);
    Note note5 = Note(990, 150, 0);
    Note note6 = Note(744, 250);
    Note note7 = Note(MAIN_MOT_FREQ, 250, 0); // reset della frequenza a quella normale
    sp->notes->push_back(note1);
    sp->notes->push_back(note2);
    sp->notes->push_back(note3);
    sp->notes->push_back(note4);
    sp->notes->push_back(note5);
    sp->notes->push_back(note6);
    sp->notes->push_back(note7);
}

bool Motors::toggleMainMotor(uint32_t spd, uint32_t stat)
{
    if (stat == TOGGLE)
    {
        if (!mainmotor.active)
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

void Motors::playInitSound()
{
    Note note1 = Note(496, 100);
    Note note2 = Note(496, 100, 0);
    Note note3 = Note(496, 100);
    Note note4 = Note(MAIN_MOT_FREQ, 250, 0);
    sp->notes->push_back(note1);
    sp->notes->push_back(note2);
    sp->notes->push_back(note3);
    sp->notes->push_back(note4);

    while (sp->notes->size() > 0)
    {
        if (sp->time1 >= sp->time2)
        {
            ledcSetup(CHANNEL_MAIN, sp->notes->at(0).freq, 12);
            ledcWrite(CHANNEL_MAIN, sp->notes->at(0).intensity);
            sp->time1 = millis();
            sp->time2 = millis() + sp->notes->at(0).duration;
            sp->notes->erase(sp->notes->begin());
        }
        else
            sp->time1 = millis();
    }
}

void Motors::brake()
{
    ledcWrite(CH_R1, MOT_MAX_VAL);
    ledcWrite(CH_R2, MOT_MAX_VAL);
    ledcWrite(CH_L1, MOT_MAX_VAL);
    ledcWrite(CH_L2, MOT_MAX_VAL);
}

void Motors::playInactiveSound()
{
    sp->notes->push_back(Note(496, 250));
    sp->notes->push_back(Note(496, 250, 0));
    sp->notes->push_back(Note(496, 250));
    sp->notes->push_back(Note(MAIN_MOT_FREQ, 250, 0));
}

void Motors::begin()
{
    ledcSetup(CH_R1, MOVEMENT_MOT_FREQ, 12);
    ledcSetup(CH_R2, MOVEMENT_MOT_FREQ, 12);
    ledcSetup(CH_L1, MOVEMENT_MOT_FREQ, 12);
    ledcSetup(CH_L2, MOVEMENT_MOT_FREQ, 12);
    ledcAttachPin(MOT_R_CTRL1, CH_R1);
    ledcAttachPin(MOT_R_CTRL2, CH_R2);
    ledcAttachPin(MOT_L_CTRL1, CH_L1);
    ledcAttachPin(MOT_L_CTRL2, CH_L2);
    ledcWrite(CH_R1, 0);
    ledcWrite(CH_R2, 0);
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, 0);
    ledcSetup(5, MAIN_MOT_FREQ, 12);
    ledcAttachPin(13, 5);
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

    if (sp->notes->size() > 0)
    {
        if (sp->time1 >= sp->time2)
        {
            ledcSetup(CHANNEL_MAIN, sp->notes->at(0).freq, 12);
            ledcWrite(CHANNEL_MAIN, sp->notes->at(0).intensity);
            sp->time1 = millis();
            sp->time2 = millis() + sp->notes->at(0).duration;
            sp->notes->erase(sp->notes->begin());

        }
        else
            sp->time1 = millis();
    }

    if (mainmotor.startup.active)
    {
        if (millis() - mainmotor.startup.time > 50)
        {
            ledcWrite(CHANNEL_MAIN, mainmotor.startup.speed);
            mainmotor.startup.speed += 75;
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
