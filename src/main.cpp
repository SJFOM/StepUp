#define BLYNK_PRINT Serial

// #include <WiFi.h>
// #include <WiFiClient.h>
// #include <BlynkSimpleEsp32.h>

#include <Arduino.h>
#include "CRC.h"
#include "ESP32TimerInterrupt.h"
#include "Functions.h"
#include "TMC2300.h"

#define PIN_ENABLE (32U)

#define LED_BUILTIN_GREEN (18U)
#define LED_BUILTIN_RED   (23U)

#define TIMER0_INTERVAL_MS 1000

// Init ESP32 timer 0
ESP32Timer ITimer0(0);

static int s_target_velocity = 0;
static bool s_direction = true;
static bool s_enable = false;

volatile bool s_run_job = false;

/**
 * @brief Set the Current object
 *
 * @param current - a value between 0 -> 31
 */
void setCurrent(int current)
{
    Serial.print("New MaxCurrent set: ");
    Serial.println(current);

    uint32_t value = 1 << TMC2300_IHOLDDELAY_SHIFT |
                     ((current << TMC2300_IRUN_SHIFT) & TMC2300_IRUN_MASK) |
                     8 << TMC2300_IHOLD_SHIFT;
    tmc2300_writeInt(TMC2300_IHOLD_IRUN, value);
}

void setVelocity(int velocity)
{
    Serial.print("New TargetVelocity set: ");
    Serial.println(velocity);

    s_target_velocity = velocity;

    tmc2300_writeInt(TMC2300_VACTUAL,
                     s_direction ? s_target_velocity : -s_target_velocity);
}

void setDirection(bool direction)
{
    Serial.println("Changed Direction");

    s_direction = direction != 0;

    tmc2300_writeInt(TMC2300_VACTUAL,
                     s_direction ? s_target_velocity : -s_target_velocity);
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

    digitalWrite(PIN_ENABLE, s_enable ? HIGH : LOW);
}

/******************************************************************************/

// Called once per second by the timer
void periodicJob()
{
    if (s_enable)
    {
        // Toggle the status LED while the motor is active
        digitalWrite(LED_BUILTIN_GREEN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN_GREEN, LOW);
        delay(250);
        digitalWrite(LED_BUILTIN_GREEN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN_GREEN, LOW);
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

void setup()
{
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
    pinMode(LED_BUILTIN_GREEN, OUTPUT);

    // Debug LED
    pinMode(LED_BUILTIN_RED, OUTPUT);

    // Enable Pin
    pinMode(PIN_ENABLE, OUTPUT);

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

    Serial.println("Initialization complete");
    digitalWrite(LED_BUILTIN_GREEN, HIGH);

    // printMotorControlOptions();
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
            case 3:  // tart motor - fast
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
