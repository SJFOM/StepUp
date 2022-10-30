#include <Arduino.h>
#include "CRC.h"
#include "ESP32TimerInterrupt.h"
#include "Functions.h"
#include "TMC2300.h"

/***************************/
/* Pin definitions - START */
/***************************/
#define LED_BUILTIN_GREEN (18U)
#define LED_BUILTIN_RED   (23U)

#define LED_STATUS       LED_BUILTIN_GREEN
#define LED_MOTOR_ACTIVE LED_BUILTIN_RED

#define PIN_TMC_POWER  (5U)
#define PIN_TMC_ENABLE (32U)
/*************************/
/* Pin definitions - END */
/*************************/

#define TIMER0_INTERVAL_MS 1000

// Init ESP32 timer 0
ESP32Timer ITimer0(0);

/**************************/
/* User variables - START */
/**************************/
// TMC variables
static int s_target_velocity = 0;
static bool s_direction = true;
static bool s_enable = false;

// Job processing boolean
volatile bool s_run_job = false;
/************************/
/* User variables - END */
/************************/

/*******************************/
/* TMC control methods - START */
/*******************************/

/**
 * @brief Set the Current object
 *
 * @param current - a scaled value between 0 -> 31
 * @return true - if config success
 * @return false - if config fail
 */
bool setCurrent(int current)
{
    Serial.print("New MaxCurrent set: ");
    Serial.println(current);

    uint32_t value = 1 << TMC2300_IHOLDDELAY_SHIFT |
                     ((current << TMC2300_IRUN_SHIFT) & TMC2300_IRUN_MASK) |
                     8 << TMC2300_IHOLD_SHIFT;
    tmc2300_writeInt(TMC2300_IHOLD_IRUN, value);
    return (bool)(tmc2300_readInt(TMC2300_IHOLD_IRUN) != 0);
}

bool setVelocity(int velocity)
{
    Serial.print("New TargetVelocity set: ");
    Serial.println(velocity);

    s_target_velocity = velocity;

    tmc2300_writeInt(TMC2300_VACTUAL,
                     s_direction ? s_target_velocity : -s_target_velocity);
    return (bool)(tmc2300_readInt(TMC2300_VACTUAL) != 0);
}

bool setDirection(bool direction)
{
    Serial.println("Changed Direction");

    s_direction = direction != 0;

    tmc2300_writeInt(TMC2300_VACTUAL,
                     s_direction ? s_target_velocity : -s_target_velocity);
    return (bool)(tmc2300_readInt(TMC2300_VACTUAL) != 0);
}

void setEnable(int enable)
{
    s_enable = enable != 0;

    if (s_enable)
    {
        Serial.println("Enable Motor: True");
    }
    else
    {
        Serial.println("Enable Motor: False");
    }

    digitalWrite(PIN_TMC_ENABLE, s_enable ? HIGH : LOW);
}

void tmcPowerOn(bool power_on)
{
    // Power pin is active low
    digitalWrite(PIN_TMC_POWER, (power_on) ? LOW : HIGH);
    delay(100);
}

/*****************************/
/* TMC control methods - END */
/*****************************/

/***************************/
/* Process methods - START */
/***************************/

// Called once per second by the timer
void periodicJob()
{
    if (s_enable)
    {
        // Toggle the status LED while the motor is active
        digitalWrite(LED_MOTOR_ACTIVE, HIGH);
        delay(250);
        digitalWrite(LED_MOTOR_ACTIVE, LOW);
        delay(250);
        digitalWrite(LED_MOTOR_ACTIVE, HIGH);
        delay(250);
        digitalWrite(LED_MOTOR_ACTIVE, LOW);
    }

    // Re-write the CHOPCONF register periodically
    tmc2300_writeInt(TMC2300_CHOPCONF, 0x14008001);
}

void printMotorControlOptions()
{
    Serial.println("Choose option");
    Serial.println(" 0 -> halt all motion");
    Serial.println(" 1 -> swap direction");
    Serial.println(" 2 -> start motor - slow");
    Serial.println(" 3 -> start motor - fast");
}

bool IRAM_ATTR TimerHandler0(void *timerNo)
{
    s_run_job = true;
    return true;
}

/*************************/
/* Process methods - END */
/*************************/

void setup()
{
    bool init_success = true;

    // Debug console
    Serial.begin(115200);
    while (!(Serial && Serial.available()))
    {
        // wait for serial port to connect. Needed for native USB
        // also wait for enter key press
        ;
    }

    // TMC2300 IC UART connection
    Serial1.begin(115200);
    while (!Serial1)
    {
        ;  // wait for serial port to connect. Needed for native USB
    }

    // Status LED
    pinMode(LED_STATUS, OUTPUT);

    // Motor LED
    pinMode(LED_MOTOR_ACTIVE, OUTPUT);

    // TMC power pin
    pinMode(PIN_TMC_POWER, OUTPUT);

    // Enable Pin
    pinMode(PIN_TMC_ENABLE, OUTPUT);

    // Initialize CRC calculation for TMC2300 UART datagrams
    tmc_fillCRC8Table(0x07, true, 0);

    Serial.print(F("\nStarting TimerInterruptTest on "));
    Serial.println(ARDUINO_BOARD);
    Serial.println(ESP32_TIMER_INTERRUPT_VERSION);
    Serial.print(F("CPU Frequency = "));
    Serial.print(F_CPU / 1000000);
    Serial.println(F(" MHz"));

    // Using ESP32  => 80 / 160 / 240MHz CPU clock ,
    // For 64-bit timer counter
    // For 16-bit timer prescaler up to 1024

    // Interval in microsecs
    if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000,
                                        TimerHandler0))
    {
        Serial.print(F("Starting  ITimer0 OK, millis() = "));
        Serial.println(millis());
    }
    else
    {
        Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
    }

    // Power on TMC2300 IC
    tmcPowerOn(true);

    // Set the motor to default off state
    setEnable(false);

    // Read IFCNT register
    init_success &= (bool)(tmc2300_readInt(TMC2300_IFCNT) != 0);

    // Set the motor current & velocities to low values
    init_success &= setVelocity(0);
    init_success &= setCurrent(0);

    if (init_success)
    {
        Serial.println("Initialization complete");
        digitalWrite(LED_STATUS, HIGH);
    }
    else
    {
        Serial.println("ERROR: Bad TMC UART comms");
        digitalWrite(LED_BUILTIN_RED, HIGH);
        tmcPowerOn(false);

        while (1)
        {
            ;
        }
    }
}

void loop()
{
    if (s_run_job)
    {
        s_run_job = false;
        periodicJob();
    }

    if (Serial.available())
    {
        String input = Serial.readString();
        switch (input.toInt())
        {
            case 0:  // halt motion
                Serial.println("Halt motion");
                setCurrent(0);
                setVelocity(0);
                setEnable(false);
                break;
            case 1:  // swap direction
                Serial.println("Swap direction");
                setDirection(!s_direction);
                break;
            case 2:  // start motor - slow
                Serial.println("start motor - slow");
                setCurrent(8);
                setVelocity(20);
                setEnable(true);
                break;
            case 3:  // start motor - fast
                Serial.println("tart motor - fast");
                setCurrent(16);
                setVelocity(40);
                setEnable(true);
                break;
            default:
                Serial.println("invalid option");
                break;
        }
        Serial.print("\n\n");
        printMotorControlOptions();
    }
}
