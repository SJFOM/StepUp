#include <Arduino.h>
#include <Wire.h>
#include "CRC.h"
#include "ESP32TimerInterrupt.h"
#include "Functions.h"
#include "TMC2300.h"

/*******************************/
/* Runtime definitions - START */
/*******************************/
#define PASSTHROUGH_MODE true
/*****************************/
/* Runtime definitions - END */
/*****************************/

/***************************/
/* Pin definitions - START */
/***************************/
#define LED_BUILTIN_GREEN (18U)
#define LED_BUILTIN_RED   (23U)

#define LED_STATUS       LED_BUILTIN_GREEN
#define LED_MOTOR_ACTIVE LED_BUILTIN_RED

#define PIN_TMC_POWER  (5U)
#define PIN_TMC_ENABLE (32U)

#if (PASSTHROUGH_MODE)
#    define PIN_PROXY_TMC_POWER  (14U)  // DIO0
#    define PIN_PROXY_TMC_ENABLE (19U)  // DIO1
#endif

#define PIN_ANALOG_JOYSTICK_X (13U)  // DIO2
#define PIN_ANALOG_JOYSTICK_Y (26U)  // DIO4
#define PIN_JOYSTICK_BUTTON   (25U)  // DIO3
/*************************/
/* Pin definitions - END */
/*************************/

/********************************/
/* Hardware definitions - START */
/********************************/
#define I2C_SLAVE_ADDRESS  (uint8_t)(5U)
#define TIMER0_INTERVAL_MS (1000UL)
#define STEPPER_MAX_SPEED  (250000UL)
/******************************/
/* Hardware definitions - END */
/******************************/

/**************************/
/* User variables - START */
/**************************/
// TMC variables
static struct tmc_config
{
    int target_velocity = 0;
    bool direction = true;
    bool enable = false;
} s_tmc_config;

// I2C message data
static struct i2c_packet
{
    uint8_t buf[16U];
    uint8_t buf_idx = 0;
} s_i2c_packet;
const uint8_t i2c_packet_id_byte0_value = 0xAB;
const uint8_t i2c_packet_id_byte1_value = 0xCD;
const uint8_t i2c_packet_id_byte0_idx = 0;
const uint8_t i2c_packet_id_byte1_idx = 1;
const uint8_t i2c_packet_read_length_idx = 2;
const uint8_t i2c_packet_write_length_idx = 3;
const uint8_t i2c_packet_data_idx = 4;

// ESP32 Timer 0 Hardware
ESP32Timer ITimer0(0);

// Job processing booleans
volatile bool run_job = false;
volatile bool print_once = false;

// Joystick variables
volatile bool joystick_active = false;
volatile long joystick_read_value = 0;
long joystick_offset = 0;
unsigned long joystick_button_timestamp = 0;

/************************/
/* User variables - END */
/************************/

/*******************************/
/* Function prototypes - START */
/*******************************/
void receiveEvent(int);

// TMC Register control
bool setCurrent(uint8_t);
bool setVelocity(int);
bool setDirection(bool);

// TMC Pin control
void setEnable(int);
void tmcPowerOn(bool);
void tmcEnablePinControl();
void tmcPowerPinControl(bool);
/*****************************/
/* Function prototypes - END */
/*****************************/

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
bool setCurrent(uint8_t current)
{
    if (current > 31)
    {
        return false;
    }
    if (!joystick_active)
    {
        Serial.print("New MaxCurrent set: ");
        Serial.println(current);
    }

    uint32_t value = 1 << TMC2300_IHOLDDELAY_SHIFT |
                     ((current << TMC2300_IRUN_SHIFT) & TMC2300_IRUN_MASK) |
                     8 << TMC2300_IHOLD_SHIFT;
    tmc2300_writeInt(TMC2300_IHOLD_IRUN, value);
    return (bool)(tmc2300_readInt(TMC2300_IHOLD_IRUN) != -1);
}

bool setVelocity(int velocity)
{
    if (abs(velocity) >= pow(2, 24) - 1)
    {
        return false;
    }

    if (!joystick_active)
    {
        // VACTUAL: 2^24 - 1
        Serial.print("New TargetVelocity set: ");
        Serial.println(velocity);
    }

    s_tmc_config.target_velocity = velocity;

    tmc2300_writeInt(TMC2300_VACTUAL,
                     s_tmc_config.direction ? s_tmc_config.target_velocity
                                            : -s_tmc_config.target_velocity);
    return (bool)(tmc2300_readInt(TMC2300_VACTUAL) != -1);
}

bool setDirection(bool direction)
{
    Serial.println("Changed Direction");

    s_tmc_config.direction = direction != 0;

    tmc2300_writeInt(TMC2300_VACTUAL,
                     s_tmc_config.direction ? s_tmc_config.target_velocity
                                            : -s_tmc_config.target_velocity);
    return (bool)(tmc2300_readInt(TMC2300_VACTUAL) != -1);
}

void setEnable(int enable)
{
    s_tmc_config.enable = enable != 0;

    if (!joystick_active)
    {
        if (s_tmc_config.enable)
        {
            Serial.println("Enable Motor");
        }
        else
        {
            Serial.println("Disable Motor");
        }
    }

    tmcEnablePinControl();
    delay(100);
}

void tmcPowerOn(bool power_on)
{
    tmcPowerPinControl(power_on);
    delay(100);
}

void tmcEnablePinControl()
{
    uint8_t enable_pin_value = s_tmc_config.enable ? HIGH : LOW;
#if (PASSTHROUGH_MODE)
    enable_pin_value = digitalRead(PIN_PROXY_TMC_ENABLE);
#endif
    digitalWrite(PIN_TMC_ENABLE, enable_pin_value);
}

void tmcPowerPinControl(bool power_on)
{
    // Power pin is active low
    uint8_t power_pin_value = (power_on) ? LOW : HIGH;
#if (PASSTHROUGH_MODE)
    power_pin_value = digitalRead(PIN_PROXY_TMC_POWER);
#endif
    digitalWrite(PIN_TMC_POWER, power_pin_value);
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
    if (s_tmc_config.enable)
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
    // tmc2300_writeInt(TMC2300_CHOPCONF, 0x14008001);
    tmc2300_writeInt(TMC2300_CHOPCONF, 0x10008001);  // 256 usteps
}

void printMotorControlOptions()
{
    Serial.println("Choose option");
    Serial.println(" 0 -> halt all motion");
    Serial.println(" 1 -> swap direction");
    Serial.println(" 2 -> start motor - slow");
    Serial.println(" 3 -> start motor - fast");
    Serial.println(" 4 -> start motor - custom input");
}

bool IRAM_ATTR TimerHandler0(void *timerNo)
{
    run_job = true;
    return true;
}

bool IRAM_ATTR TimerHandler0Analog(void *timerNo)
{
    unsigned adc_read = analogRead(PIN_ANALOG_JOYSTICK_X);
    joystick_read_value =
        map(adc_read, 0, 4095, -STEPPER_MAX_SPEED, STEPPER_MAX_SPEED);
    return true;
}

void joystickButtonHandler()
{
    if (millis() - joystick_button_timestamp > 500)
    {
        joystick_button_timestamp = millis();
        print_once = true;
        joystick_active = !joystick_active;
        s_tmc_config.enable = joystick_active;
        digitalWrite(PIN_TMC_ENABLE, s_tmc_config.enable ? HIGH : LOW);
    }
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
    Serial1.begin(460800);
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

#if (PASSTHROUGH_MODE)
    // Enable Pin
    pinMode(PIN_PROXY_TMC_POWER, INPUT);
    // Enable Pin
    pinMode(PIN_PROXY_TMC_ENABLE, INPUT);

    Wire.begin(I2C_SLAVE_ADDRESS);  // join i2c bus with address
    Wire.onReceive(receiveEvent);   // register event
#endif

    // Initialize CRC calculation for TMC2300 UART datagrams
    tmc_fillCRC8Table(0x07, true, 0);

#if (false == PASSTHROUGH_MODE)
    Serial.print(F("\nStarting TimerInterrupt on "));
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

    // Interval in microsecs
    if (ITimer0.attachInterruptInterval(100 * 1000, TimerHandler0Analog))
    {
        Serial.print(F("Starting  ITimer0 OK, millis() = "));
        Serial.println(millis());
    }
    else
    {
        Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
    }
#endif

    // Power on TMC2300 IC
    tmcPowerOn(true);

    // Set the motor to default off state
    setEnable(false);

    // Read IFCNT register
    init_success &= (bool)(tmc2300_readInt(TMC2300_IFCNT) != -1);
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

#if (false == PASSTHROUGH_MODE)
    // set the resolution to 12 bits (0-4096)
    analogReadResolution(12);
    pinMode(PIN_JOYSTICK_BUTTON, INPUT_PULLUP);

    attachInterrupt(PIN_JOYSTICK_BUTTON, joystickButtonHandler, FALLING);

    Serial.println("ADC calibration step");
    // const unsigned pin_in_test = PIN_ANALOG_JOYSTICK_X;
    // Serial.println("Setting up ADC pins");
    unsigned long adc_accumulator = 0;
    for (int i = 0; i <= 10; i++)
    {
        // uint32_t x_axis = analogRead(PIN_ANALOG_JOYSTICK_X);
        // uint32_t y_axis = analogRead(PIN_ANALOG_JOYSTICK_Y);
        // bool btn = (bool)digitalRead(PIN_JOYSTICK_BUTTON);
        // Serial.printf("%d - %d - %d\n", x_axis, y_axis, btn);
        adc_accumulator += analogRead(PIN_ANALOG_JOYSTICK_X);
        delay(20);
    }
    joystick_offset = map(adc_accumulator / 10,
                          0,
                          4095,
                          -STEPPER_MAX_SPEED,
                          STEPPER_MAX_SPEED);
#endif
}

void loop()
{
    static unsigned long last_adc_value = 0;

#if (PASSTHROUGH_MODE)
#else
    if (print_once)
    {
        print_once = false;
        Serial.printf("Joystick %s\n",
                      (joystick_active) ? "ACTIVE" : "INACTIVE");
    }

    if (run_job)
    {
        run_job = false;
        periodicJob();
    }

    if (joystick_active && joystick_read_value != last_adc_value)
    {
        last_adc_value = joystick_read_value;

        if (abs(joystick_read_value) > STEPPER_MAX_SPEED / 4)
        {
            setVelocity(joystick_read_value);
        }
        else
        {
            setVelocity(0);
        }
        setCurrent(20);
        setEnable(true);
    }

    if (Serial.available() && !joystick_active)
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
                setDirection(!s_tmc_config.direction);
                break;
            case 2:  // start motor - slow
                Serial.println("start motor - slow");
                setCurrent(8);
                setVelocity(10000);
                setEnable(true);
                break;
            case 3:  // start motor - fast
                Serial.println("start motor - fast");
                setCurrent(16);
                setVelocity(100000);
                setEnable(true);
                break;
            case 4:  // start motor - custom input
                Serial.println("start motor - input");
                while (!Serial.available())
                    ;
                input = Serial.readString();
                setCurrent(31);
                setVelocity(input.toInt());
                setEnable(true);
                break;
            default:
                Serial.println("invalid option");
                break;
        }
        Serial.print("\n\n");
        printMotorControlOptions();
    }
#endif
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
    // Receive bytes
    while (Wire.available())  // loop through all received bytes
    {
        s_i2c_packet.buf[s_i2c_packet.buf_idx++] = Wire.read();
    }

    // Check the received packet is valid
    if ((s_i2c_packet.buf[i2c_packet_id_byte0_idx] ==
         i2c_packet_id_byte0_value) &&
        (s_i2c_packet.buf[i2c_packet_id_byte1_idx] ==
         i2c_packet_id_byte1_value))
    {
        uint8_t write_length = s_i2c_packet.buf[i2c_packet_write_length_idx];
        uint8_t read_length = s_i2c_packet.buf[i2c_packet_read_length_idx];

        // Send our bytes over UART as normal
        tmc2300_readWriteArray(&s_i2c_packet.buf[i2c_packet_data_idx],
                               write_length,
                               read_length);

        // Return the received bytes (via UART) back to master over I2C
        Wire.write(&s_i2c_packet.buf[i2c_packet_data_idx], read_length);
    }
}