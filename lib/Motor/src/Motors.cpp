#include <Motors.h>
#include <Sensors.h>
#include <Status.h>
#include <esp32-hal-gpio.h>
#include <BluetoothSerial.h>
#include <Core.h>
#include <Navigation.h>
#include <cmath>
#include <Mux.h>

BluetoothSerial SBT;
Sensors motorsensor;
Status motorstatus;
Core motorscore;
NAV motornav;
Mux motormux;
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

uint8_t left_spd = 255;
uint8_t right_spd = 255;

bool robot_moving = false;
bool move = false;
uint8_t robot_not_moving;
int32_t heading_to_maintain;
bool maintain_heading = false;
char direction;

uint32_t time_1 = millis();
uint32_t time_2 = millis();

void Motors::begin()
{
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);
    ledcAttachPin(MOT_R_SPD, CHANNEL_RIGHT);
    ledcAttachPin(MOT_L_SPD, CHANNEL_LEFT);
    ledcSetup(CHANNEL_RIGHT, 100, 8);
    ledcSetup(CHANNEL_LEFT, 100, 8);
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
            time_2 = millis();

            if (time_2 - time_1 > 150)
            {
                time_1 = millis();

                if (direction == 's')
                    maintainHeading(true);
                else
                    maintainHeading();
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
                ledcSetup(CHANNEL_MAIN, 2000, 8);
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

uint32_t Motors::getTime()
{
    return t1;
}

void Motors::forward()
{
    stop();
    motorsensor.checkFrontObstacle(MOTORS); // manda subito una richiesta per controllare se c'è un ostacolo
    motorscore.println((char *)"(MOT) MOTORS FORWARD");
    motorstatus.setRunning(true);

    if (!maintain_heading)
    {
        left_spd = 255;
        right_spd = 255;
        digitalWrite(MOTOR_RIGHT_DIR, FWD);
        digitalWrite(MOTOR_LEFT_DIR, FWD);
        ledcWrite(CHANNEL_RIGHT, 255);
        ledcWrite(CHANNEL_LEFT, 255);
        heading_to_maintain = motorsensor.getHeading();
        maintain_heading = true;
        robot_moving = true;
        motorsensor.setMotorsRotating();
    }

    direction = 'w';
    motorsensor.checkMovement('y');
}

void Motors::backwards()
{
    stop();
    motorscore.println((char *)"(MOT) MOTORS BACKWARDS");
    motorstatus.setRunning(true);

    if (!maintain_heading)
    {
        digitalWrite(MOTOR_RIGHT_DIR, BCK);
        digitalWrite(MOTOR_LEFT_DIR, BCK);
        ledcWrite(CHANNEL_RIGHT, 255);
        ledcWrite(CHANNEL_LEFT, 255);
        heading_to_maintain = motorsensor.getHeading();
        maintain_heading = true;
        robot_moving = true;
        motorsensor.setMotorsRotating();
    }

    direction = 's';
    motorsensor.checkMovement('y');
}

void Motors::right()
{
    stop();
    digitalWrite(MOTOR_RIGHT_DIR, BCK);
    digitalWrite(MOTOR_LEFT_DIR, FWD);
    ledcWrite(CHANNEL_RIGHT, 255);
    ledcWrite(CHANNEL_LEFT, 255);
    motorstatus.setRunning(true);
    direction = 'd';
    robot_moving = true;
    motorsensor.setMotorsRotating();
    motorsensor.checkRotation();
    maintain_heading = false;
}

void Motors::left()
{
    stop();
    digitalWrite(MOTOR_RIGHT_DIR, FWD);
    digitalWrite(MOTOR_LEFT_DIR, BCK);
    ledcWrite(CHANNEL_RIGHT, 255);
    ledcWrite(CHANNEL_LEFT, 255);
    motorstatus.setRunning(true);
    direction = 'a';
    robot_moving = true;
    motorsensor.checkRotation();
    motorsensor.setMotorsRotating();
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
    motorsensor.stopMoving();
    motorsensor.setMotorsStop();
}

char Motors::getDirection()
{
    return direction;
}

void Motors::maintainHeading(bool reverse)
{
    // DESTRA: DIFF NEGATIVO; SINISTRA: DIFF POSITIVO

    int32_t current_heading = motorsensor.getHeading();
    int32_t diff = heading_to_maintain - current_heading;

    if (reverse)
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
        a = valore massimo motore (255 - 150 = 105; MOT_BASE_VAL - MOT_MIN_VAL = c)
        b = costante (rateo del decay, 0.642223)
        x = diff (floatdiff)
        */

        float floatdiff = (float)diff / 100;
        uint8_t motor_value = MOT_MIN_VAL + ((MOT_BASE_VAL - MOT_MIN_VAL) * pow((0.35777), floatdiff));
        if (motor_value > MOT_MIN_VAL)
        {
            left_spd = motor_value;
            right_spd = 255;
        }
    }
    else
    {
        // SINISTRA

        float floatdiff = (float)diff / 100;
        uint8_t motor_value = MOT_MIN_VAL + ((MOT_BASE_VAL - MOT_MIN_VAL) * pow((0.35777), floatdiff));
        if (motor_value > MOT_MIN_VAL)
        {
            right_spd = motor_value;
            left_spd = 255;
        }
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

// FORWARD PER CONTROLLO VELOCITA' IN MM/S
/*
void Motors::forward(float desired_rps)
{
    if (robot_not_moving < 10)
    {
        move = true;
        motorstatus.setRunning(true);
        ledcWrite(0, MOTOR_VALUE);

        int period = motorsensor.getEncoderPeriod();
        if (period != -1 && period > 10)
        {
            int revolution_time = period * ENCODER_TEETH;
            float encoder_current_rps = 1000.00 / (float)revolution_time;
            float wheel_current_rps = encoder_current_rps / 5;

            wheel_spd = wheel_circumference * wheel_current_rps;

            float rps_difference = encoder_current_rps - desired_rps;
            int value_to_apply = rps_difference * STEPS_MULTIPLIER * -1;

            if (value_to_apply > 0 && value_to_apply < 1)
            {
                value_to_apply = 1;
            }
            else if (value_to_apply < 0 && value_to_apply > -1)
            {
                value_to_apply = -1;
            }

            if (MOTOR_VALUE + value_to_apply > MIN_MOTOR_VALUE && MOTOR_VALUE + value_to_apply < 256)
            {
                // ci dovrebbe essere accelerazione quindi controlla se c'è effettivamente
                int accX = motorsensor.getAccY();
                if (accX > ACCELERATION_ACTIVATION || accX < -ACCELERATION_ACTIVATION)
                {
                    // il robot sa che sta accelerando
                    robot_moving = true;
                    robot_not_moving = 0;
                }
                else
                {
                    // se non percepisce accelerazione, controlla se il robot ha già accelerato, quindi si sta muovendo
                    if(robot_moving == false)
                    {
                        robot_not_moving++;
                    }
                }
                MOTOR_VALUE += value_to_apply;
            }

            #ifdef ENABLE_LOGGING
            SBT.print("  MOTOR_CURRENT_VALUE: ");
            SBT.print(MOTOR_VALUE);
            SBT.print("  PERIOD: ");
            SBT.print(period);
            SBT.print("  VALUE_TO_APPLY: ");
            SBT.print(value_to_apply);
            SBT.print("  CURRENT_RPS: ");
            SBT.print(encoder_current_rps);
            SBT.print("  WHEEL_RPS: ");
            SBT.print(wheel_current_rps);
            SBT.print("  WHEEL_SPD: ");
            SBT.println(wheel_spd);
            #endif
        }
    }
    else
    {
        // se il counter è troppo alto vuol dire effettivamente che il robot è fermo,
        // quindi ferma i motori
        // TODO: provare ad andare avanti e indietro, destra e sinistra per riuscire a muoversi
        motorstatus.setError(true);
        stop();
    }
}
*/
