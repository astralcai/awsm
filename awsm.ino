#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <DS3231.h>
#include <Wire.h>
#include "Queue.h"
#include "DS18B20.h"
#include "Pump.h"
#include "FlowSensor.h"
#include "TurbiditySensor.h"
#include <SparkFun_LTE_Shield_Arduino_Library.h>

// Device ID, this needs to be unique
#define DEVICE_ID 0

// pin definitions
#define TEMPERATURE_SENSOR_PIN 28
#define PRESSURE_SENSOR_PIN A2
#define TURBIDITY_SENSOR_PIN A5
#define FORWARD_VALVE_PIN 22
#define REVERSE_VALVE_PIN 23
#define FORWARD_PUMP_PIN 25
#define REVERSE_PUMP_PIN 26
#define FLOW_SENSOR_PIN 2
#define LTE_SERIAL_PIN_1 A10
#define LTE_SERIAL_PIN_2 A11
#define SD_CARD_PIN 53

// device runtime configurations
#define SAMPLE_INTERVAL 5000
#define SAMPLE_INTERVAL_EVENT 1000
// #define SAMPLE_INTERVAL 15 * 60000
// #define SAMPLE_INTERVAL_EVENT 30000
#define PRESSURE_SAMPLE_INTERVAL 1000
#define PRESSURE_LOOKBACK_TIME 10000 // AT LEAST 10 TIMES THE PRESSURE SAMPLE INTERVAL
#define SAMPLE_TIME_UNCERTAINTY 10
#define PRESSURE_CHANGE_THRESHOLD 5
#define PUMP_SAMPLE_TIME 80000
#define PUMP_CLEAN_TIME 40000
#define UPLOAD_INTERVAL 24 * 60 * 60000 // milliseconds in a day
#define EVENT_DURATION 2 * 60 * 60000

// calibration settings
#define FLOW_CALIBRATION_FACTOR 7.5

// device controller definitions
DS18B20 tempSensor(TEMPERATURE_SENSOR_PIN);
Pump pump(FORWARD_PUMP_PIN, REVERSE_PUMP_PIN, FORWARD_VALVE_PIN, REVERSE_VALVE_PIN);
FlowSensor flowSensor(FLOW_SENSOR_PIN, FLOW_CALIBRATION_FACTOR);
TurbiditySensor turbiditySensor(TURBIDITY_SENSOR_PIN);
DS3231 clock;

// pressure change tracker
Queue pressure_history(PRESSURE_LOOKBACK_TIME / PRESSURE_SAMPLE_INTERVAL);

// define LTE objects
SoftwareSerial lteSerial(LTE_SERIAL_PIN_1, LTE_SERIAL_PIN_2);
LTE_Shield lte;

const String APN = "hologram";

// define server address
const char SERVER_URL[] = "ec2-35-183-113-176.ca-central-1.compute.amazonaws.com";
const unsigned int SERVER_PORT = 9000;
const unsigned int HOLOGRAM_LISTEN_PORT = 9998;

int socket = -1;

bool century = false;
bool h12Flag;
bool pmFlag;

// device state variables
enum DeviceState
{
    SAMPLE_EMPTY,
    TAKING_SAMPLE,
    SAMPLE_COLLECTED,
    SYSTEM_CLEANED
};

enum DeviceState state = SAMPLE_EMPTY;
long pumpStartTime = 0;
bool uploading = false;
bool uploadReady = false;
bool lte_active = false;
bool event = false;

// initialize data
long start_time;
long event_timer;
long curr_time;
float temperature;
float turbidity;
float pressure;

// initialize file
File file;
String text;
String writeFile;
String readFile;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ; // wait till Serial port connects

    // initialize lte shield
    lte_active = lte_init();
    if (!lte_active)
        lte_active = lte_init(); // try again if fails

    if (lte_active)
    {
        Serial.println("LTE initialized");
    }
    else
    {
        Serial.println("LTE not initialized");
    }

    // initialize the SD card
    if (SD.begin(SD_CARD_PIN))
    {
        Serial.println("SD card initialized");
    }
    else
    {
        Serial.println("SD card initiliazation failed");
    }

    // initialize the flow sensor
    flowSensor.init();

    // initialize the real time clock
    Wire.begin();
    clock.setClockMode(false);

    // initialize first file to write to
    writeFile = String(clock.getYear()) + "-" + String(clock.getMonth(century)) + "-" + String(clock.getDate());

    Serial.println("Initialization Complete");
    start_time = millis();
}

void loop()
{

    curr_time = millis() - start_time;

    if (event && curr_time > SAMPLE_INTERVAL_EVENT && curr_time % SAMPLE_INTERVAL_EVENT < SAMPLE_TIME_UNCERTAINTY)
    {
        take_measurements();
    }
    else if (!event && curr_time > SAMPLE_INTERVAL && curr_time % SAMPLE_INTERVAL < SAMPLE_TIME_UNCERTAINTY)
    {
        take_measurements();
    }

    if (event && millis() - event_timer >= EVENT_DURATION)
    {
        event = false;
    }

    if (curr_time % PRESSURE_SAMPLE_INTERVAL < SAMPLE_TIME_UNCERTAINTY)
    {
        pressure = analogRead(PRESSURE_SENSOR_PIN);
        pressure_history.enqueue(pressure);
        Serial.println("Pressure: " + String(pressure));

        if (pressure_history.tail3() - pressure_history.head3() > PRESSURE_CHANGE_THRESHOLD * 3 && state == SAMPLE_EMPTY)
        {
            pumpStartTime = millis();
            pump.forward();
            state = TAKING_SAMPLE;
            event_timer = millis();
            event = true;
        }
    }

    // UNCOMMET THE FOLLOWING CODE IF YOU WANT THE PUMP TO AUTO START IN FIVE SECONDS
    // INSTEAD OF WAITING TO BE TRIGGERED BY THE PRESSURE SENSOR

    // if (curr_time >= 5000 && state == SAMPLE_EMPTY)
    // {
    //     pumpStartTime = millis();
    //     pump.forward();
    //     state = TAKING_SAMPLE;
    //     event_timer = millis();
    //     event = true;
    // }

    if (millis() - pumpStartTime >= PUMP_SAMPLE_TIME && state == TAKING_SAMPLE)
    {
        pump.reverse();
        pumpStartTime = millis();
        state = SAMPLE_COLLECTED;
    }

    if (millis() - pumpStartTime >= PUMP_CLEAN_TIME && state == SAMPLE_COLLECTED)
    {
        pump.off();
        state = SYSTEM_CLEANED;
    }

    if (curr_time > UPLOAD_INTERVAL && curr_time % UPLOAD_INTERVAL < SAMPLE_TIME_UNCERTAINTY && lte_active)
    // UNCOMMENT THE FOLLOWING LINE AND COMMENT OUT THE LINE ABOVE TO LET THE TRANSMISSION START HERE
    // IN 20 SECONDS INSTEAD OF WAITING TILL THE END OF DAY. FEEL FREE TO CHANGE THE TIME
    // if (millis() - start_time >= 20000 && lte_active)
    {
        // set file to read from and increment read file
        readFile = writeFile + ".txt";
        writeFile = String(clock.getYear()) + "-" + String(clock.getMonth(century)) + "-" + String(clock.getDate());
        uploading = true;
    }

    if (uploading && !uploadReady)
    {
        uploadReady = sendMessage(DEVICE_ID + " new " + readFile);
    }

    if (uploading && uploadReady)
    {
        file = SD.open(readFile);
        text = "" + DEVICE_ID;
        while (!file.available())
        {
            text += " " + file.readStringUntil('\n');
        }
        if (sendMessage(text))
        {
            uploading = false;
            uploadReady = false;
        }
    }

    // lte.poll();
}

void take_measurements()
{
    temperature = tempSensor.readTempC();
    turbidity = turbiditySensor.read();

    file = SD.open(writeFile + ".txt", FILE_WRITE);
    file.print(String(clock.getHour(h12Flag, pmFlag)) + ":" + String(clock.getMinute()) + ":" + String(clock.getSecond()));
    file.print(",");
    file.print(temperature);
    file.print(",");
    file.print(turbidity);
    file.print(",");
    file.println(pressure);
    file.close();

    // sendMessage(String(clock.getHour(h12Flag, pmFlag)) + ":" + String(clock.getMinute()) + ":" + String(clock.getSecond()) + "," + String(temperature) + "," + String(turbidity));
}

bool lte_init()
{
    int opsAvailable;
    struct operator_stats ops[5];
    String currentOperator = "";

    Serial.println(F("Initializing the LTE Shield..."));

    if (!lte.begin(lteSerial))
    {
        Serial.println("Could not initialize LTE Shield");
        return false;
    }

    if (lte.getOperator(&currentOperator) == LTE_SHIELD_SUCCESS)
    {
        Serial.print(F("Already connected to: "));
        Serial.println(currentOperator);
        return true;
    }
    else
    {
        Serial.println(F("Setting mobile-network operator"));
        if (lte.setNetwork(MNO_SW_DEFAULT))
        {
            Serial.print(F("Set mobile network operator to "));
            Serial.println(F("Default\r\n"));
        }
        else
        {
            Serial.println(F("Error setting MNO. Try cycling power to the shield/Arduino."));
            return false;
        }

        // Set the APN -- Access Point Name -- e.g. "hologram"
        Serial.println(F("Setting APN..."));
        if (lte.setAPN(APN) == LTE_SHIELD_SUCCESS)
        {
            Serial.println(F("APN successfully set.\r\n"));
        }
        else
        {
            Serial.println(F("Error setting APN. Try cycling power to the shield/Arduino."));
            return false;
        }

        Serial.println(F("Scanning for operators...this may take up to 3 minutes\r\n"));
        // lte.getOperators takes in a operator_stats struct pointer and max number of
        // structs to scan for, then fills up those objects with operator names and numbers
        opsAvailable = lte.getOperators(ops, 5); // This will block for up to 3 minutes

        if (opsAvailable > 0)
        {
            // Pretty-print operators we found:
            Serial.println("Found " + String(opsAvailable) + " operators");
            int selection = 0;
            while (selection < opsAvailable)
            {
                Serial.println("Attempting connection " + String(selection) + ".");
                lte.registerOperator(ops[selection++]);
                if (lte.getOperator(&currentOperator) == LTE_SHIELD_SUCCESS)
                {
                    Serial.println("Network " + ops[selection].longOp + " registered\r\n");
                    return true;
                }
            }
            Serial.println(F("Error connecting to operator. Reset and try again, or try another network."));
            return false;
        }
    }
}

bool sendMessage(String message)
{
    socket = lte.socketOpen(LTE_SHIELD_TCP);
    if (socket >= 0)
    {
        if (lte.socketConnect(socket, SERVER_URL, SERVER_PORT) == LTE_SHIELD_SUCCESS)
        {
            Serial.println("Sending: " + String(message));
            if (lte.socketWrite(socket, message) == LTE_SHIELD_SUCCESS)
            {
                lte.socketClose(socket);
                return true;
            }
            else
            {
                Serial.println(F("Failed to write"));
                return false;
            }
        }
    }
}
