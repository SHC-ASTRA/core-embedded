/**
 * @file Main.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @author Charles Marmann (cmm0077@uah.edu)
 * @brief Core Embedded Main MCU
 *
 */

//------------//
//  Includes  //
//------------//

#include <Arduino.h>
#include <FastLED.h>

#include <cmath>
// Our own resources
#include "AstraMisc.h"
#include "AstraSensors.h"
#include "AstraVicCAN.h"
#include "CoreMainMCU.h"
#ifdef FLIPSKY
#    include <VescUart.h>

#    include "FlipskyMotor.h"
using Motor = FlipskyMotor;
#else
#    include "AstraMotors.h"
using Motor = AstraMotors;
#endif

//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

// strip 1: 1-40
// strip 2: 41-82
// strip 3: 83-124
// strip 4: 125-166
// CCW: 1,2,3,4
#define NUM_LEDS 166

// Clucky: 1.11715
#define WHEEL_CIRCUMFERENCE 1.11715
// Testbed: 0.6168
// #define WHEEL_CIRCUMFERENCE 0.6168

// Clucky: 100
#define WHEEL_GEARBOX 100
// Testbed: 64
// #define WHEEL_GEARBOX 64

// REV Motor IDs
#define MOTOR_ID_FL 2  // REV motor ID for front left wheel
#define MOTOR_ID_FR 1  // REV motor ID for front right wheel
#define MOTOR_ID_BL 4  // REV motor ID for back left wheel
#define MOTOR_ID_BR 3  // REV motor ID for back right wheel
#define MOTOR_AMOUNT 4

//---------------------//
//  Component classes  //
//---------------------//

// LED Strip
int led_counter = 0;
CRGB leds[NUM_LEDS];

// Sensor declarations

Adafruit_BMP3XX bmp;

SFE_UBLOX_GNSS myGNSS;

Adafruit_BNO055 bno;

#ifdef FLIPSKY
// FlipskyMotor(uint8_t vescCanId, bool setInverted)
Motor MotorFL(MOTOR_ID_FL, false);  // Front Left
Motor MotorBL(MOTOR_ID_BL, false);  // Back Left
Motor MotorFR(MOTOR_ID_FR, true);   // Front Right
Motor MotorBR(MOTOR_ID_BR, true);   // Back Right
#else
// AstraMotors(int setMotorID, bool setInverted, int setGearBox)
Motor MotorFL(MOTOR_ID_FL, false, WHEEL_GEARBOX);  // Front Left
Motor MotorBL(MOTOR_ID_BL, false, WHEEL_GEARBOX);  // Back Left
Motor MotorFR(MOTOR_ID_FR, true, WHEEL_GEARBOX);   // Front Right
Motor MotorBR(MOTOR_ID_BR, true, WHEEL_GEARBOX);   // Back Right
#endif

Motor *motorList[4] = {&MotorFL, &MotorBL, &MotorFR, &MotorBR};  // Left motors first, right motors second

//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

unsigned long clockTimer = 0;
unsigned long lastFeedback = 0;
unsigned long lastCtrlCmd = 0;
unsigned long lastVoltRead = 0;
unsigned long lastAccel = 0;
unsigned long lastMotorStatus = 0;
unsigned long lastVersionSend = 0;
long lastTurn = 0;

//--------------//
//  Prototypes  //
//--------------//

int findRotationDirection(float current_direction, float target_direction);
String outputBno();
String outputBmp();
String outputGPS();
void setLED(int r_val, int b_val, int g_val);
float clamp_angle(float angle);
void Stop();

#ifndef FLIPSKY
void loop2(void *pvParameters) {
    static uint8_t heartBeatNum = 1;
    while (true) {
        CAN_sendHeartbeat(heartBeatNum);
        heartBeatNum++;
        if (heartBeatNum > 4) {
            heartBeatNum = 1;
        }
        delay(5);
    }
}
#endif

//--------//
//  Misc  //
//--------//

struct TurningToStatus {
    bool enabled = false;
    int targetHeading = 0;  // Degrees
    long timeoutStamp = 0;  // Seconds
} turningToStatus;

//------------------------------------------------------------------------------------------------//
//  Setup
//------------------------------------------------------------------------------------------------//
//
//
//------------------------------------------------//
//                                                //
//      ////////    //////////    //////////      //
//    //                //        //        //    //
//    //                //        //        //    //
//      //////          //        //////////      //
//            //        //        //              //
//            //        //        //              //
//    ////////          //        //              //
//                                                //
//------------------------------------------------//
void setup() {
    //--------//
    //  Pins  //
    //--------//

    pinMode(LED_BUILTIN, OUTPUT);

    //-----------//
    //  MCU LED  //
    //-----------//

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);

    if (ESP32Can.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX))
        Serial.println("CAN bus started!");
    else
        Serial.println("CAN bus failed!");

    //-----------//
    //  Sensors  //
    //-----------//

    if (!bno.begin())
        Serial.println("BNO 055 failed");
    else
        Serial.println("BNO 055 started successfully");

    if (!bmp.begin_I2C())
        Serial.println("BMP 388 failed");
    else
        Serial.println("BMP 388 started successfully");

    if (!myGNSS.begin())
        Serial.println("M9N GPS failed");
    else
        Serial.println("M9N GPS started successfully");

    initializeBMP(bmp);

    // Setup for GPS

    myGNSS.setI2COutput(COM_TYPE_UBX);  // Set the I2C port to output UBX only
                                        // (turn off NMEA noise)
    myGNSS.setNavigationFrequency(30);
    // Create storage for the time pulse parameters
    UBX_CFG_TP5_data_t timePulseParameters;

    // Get the time pulse parameters
    if (myGNSS.getTimePulseParameters(&timePulseParameters) == false)
        Serial.println(F("getTimePulseParameters failed! not Freezing..."));

    // Print the CFG TP5 version
    Serial.print(F("UBX_CFG_TP5 version: "));
    Serial.println(timePulseParameters.version);

    timePulseParameters.tpIdx = 0;  // Select the TIMEPULSE pin
    // timePulseParameters.tpIdx = 1; // Or we could select the TIMEPULSE2 pin
    // instead, if the module has one

    // We can configure the time pulse pin to produce a defined frequency or
    // period Here is how to set the frequency:

    // While the module is _locking_ to GNSS time, make it generate 2kHz
    timePulseParameters.freqPeriod = 2000;           // Set the frequency/period to 2000Hz
    timePulseParameters.pulseLenRatio = 0x55555555;  // Set the pulse ratio to 1/3 * 2^32 to produce 33:67
                                                     // mark:space

    // When the module is _locked_ to GNSS time, make it generate 1kHz
    timePulseParameters.freqPeriodLock = 1000;           // Set the frequency/period to 1000Hz
    timePulseParameters.pulseLenRatioLock = 0x80000000;  // Set the pulse ratio to 1/2 * 2^32 to produce 50:50
                                                         // mark:space

    timePulseParameters.flags.bits.active = 1;  // Make sure the active flag is set to enable the time pulse.
                                                // (Set to 0 to disable.)
    timePulseParameters.flags.bits.lockedOtherSet = 1;  // Tell the module to use freqPeriod while locking and
                                                        // freqPeriodLock when locked to GNSS time
    timePulseParameters.flags.bits.isFreq =
        1;  // Tell the module that we want to set the frequency (not the period)
    timePulseParameters.flags.bits.isLength =
        0;  // Tell the module that pulseLenRatio is a ratio / duty cycle (* 2^-32)
            // - not a length (in us)
    timePulseParameters.flags.bits.polarity = 1;  // Tell the module that we want the rising edge at the top
                                                  // of second. (Set to 0 for falling edge.)

    // Now set the time pulse parameters
    if (myGNSS.setTimePulseParameters(&timePulseParameters) == false)
        Serial.println(F("setTimePulseParameters failed!"));
    else
        Serial.println(F("Success!"));

    //--------------------//
    //  Misc. Components  //
    //--------------------//

#ifndef FLIPSKY
    xTaskCreatePinnedToCore(loop2,    // Function to implement the task
                            "loop2",  // Name of the task
                            1000,     // Stack size in bytes
                            NULL,     // Task input parameter
                            0,        // Priority of the task
                            NULL,     // Task handle.
                            0         // Core where the task should run
    );
#endif

    FastLED.addLeds<WS2812B, PIN_LED_STRIP, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    int led_rgb[3] = {0, 0, 10};
    for (int i = 0; i < NUM_LEDS; ++i) {
        leds[i] = CRGB(led_rgb[0], led_rgb[1], led_rgb[2]);
        FastLED.show();
        delay(10);
    }

#ifdef FLIPSKY
    flipskySerial.begin(FLIPSKY_BAUD, SERIAL_8N1, PIN_FLIPSKY_RX, PIN_FLIPSKY_TX);
    flipskyUart.setSerialPort(&flipskySerial);
    flipskyUart.setDebugPort(&Serial);
#else
    // The sparkmaxes are probably ready by now right
    for (int i = 0; i < MOTOR_AMOUNT; i++) {
        motorList[i]->setSlowStatusPeriods();
    }
#endif
}

//------------------------------------------------------------------------------------------------//
//  Loop
//------------------------------------------------------------------------------------------------//
//
//
//-------------------------------------------------//
//                                                 //
//    /////////      //            //////////      //
//    //      //     //            //        //    //
//    //      //     //            //        //    //
//    ////////       //            //////////      //
//    //      //     //            //              //
//    //       //    //            //              //
//    /////////      //////////    //              //
//                                                 //
//-------------------------------------------------//
void loop() {
    //----------//
    //  Timers  //
    //----------//
#ifdef BLINK
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
#endif

    // Accelerate motors; update the speed for all motors
    if (millis() - lastAccel >= 50) {
        lastAccel = millis();
        for (int i = 0; i < 4; i++) {
            motorList[i]->accelerate();
        }
    }

    // Motor status debug printout
    if (millis() - lastMotorStatus > 500) {
        lastMotorStatus = millis();

        for (int i = 0; i < 4; i++) {
#ifdef FLIPSKY
            motorList[i]->pollStatus();
#endif
            if (millis() - motorList[i]->status1.timestamp < 500) {
                vicCAN.send(CMD_REVMOTOR_FEEDBACK, motorList[i]->getID(),
                            motorList[i]->status1.motorTemperature * 10,
                            motorList[i]->status1.busVoltage * 10, motorList[i]->status1.outputCurrent * 10);
            }
            if (millis() - motorList[i]->status1.timestamp < 500 &&
                millis() - motorList[i]->status2.timestamp < 500) {
                vicCAN.send(58, motorList[i]->getID(), motorList[i]->status2.sensorPosition,
                            motorList[i]->status1.sensorVelocity);
            }
        }
    }

    if (millis() - lastTurn > 100 && turningToStatus.enabled) {
        lastTurn = millis();

        // NOTE: only used by legacy Autonomy
        if (millis() > turningToStatus.timeoutStamp) {
            turningToStatus.enabled = false;
            Stop();
        } else {
            // Get heading measurement from IMU
            sensors_event_t orientationData;
            bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
            int currentHeading = orientationData.orientation.x;
            currentHeading = clamp_angle(currentHeading);
            turningToStatus.targetHeading = clamp_angle(turningToStatus.targetHeading);

            // Make LSS rotate towards the required heading
            int error = turningToStatus.targetHeading - currentHeading;

            if (abs(error) < 10) {
                turningToStatus.enabled = false;
                Stop();
            } else if (error > 0) {
                motorList[0]->sendDuty(0.75);
                motorList[1]->sendDuty(0.75);

                motorList[2]->sendDuty(-0.75);
                motorList[3]->sendDuty(-0.75);
            } else {
                motorList[0]->sendDuty(-0.75);
                motorList[1]->sendDuty(-0.75);

                motorList[2]->sendDuty(0.75);
                motorList[3]->sendDuty(0.75);
            }
        }
    }

    if (millis() - lastFeedback >= 2000) {
        lastFeedback = millis();

        // Pull GPS
        double gpsData[4];
        getPosition(myGNSS, gpsData);

        // Pull IMU
        float bnoData2[7];
        pullBNOData(bno, bnoData2);

        // Calibration status
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        int calibStatus = system * 1000 + gyro * 100 + accel * 10 + mag;

        // M9N (GNSS) Data
        vicCAN.send(CMD_GNSS_LAT, gpsData[0]);
        vicCAN.send(CMD_GNSS_LON, gpsData[1]);
        vicCAN.send(CMD_GNSS_SAT, gpsData[2], gpsData[3]);

        // BNO (IMU) Data
        vicCAN.send(CMD_DATA_IMU_GYRO, bnoData2[0], bnoData2[1], bnoData2[2], calibStatus);
        vicCAN.send(CMD_DATA_IMU_ACCEL_HEADING, bnoData2[3], bnoData2[4], bnoData2[5], bnoData2[6]);

        // BMP (Humidity, altitude, pressure) Data
        if (bmp.performReading()) {
            float temp = bmp.temperature;
            float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
            float pressure = bmp.pressure * 0.1;  // Convert Pa to mBar*10
            vicCAN.send(CMD_DATA_BMP, temp, altitude, pressure);
        }
    }

    if (millis() - lastVoltRead > 1000) {
        lastVoltRead = millis();
        float vBatt = convertADC(analogRead(PIN_VDIV_BATT), 10, 2.21);
        float v12 = convertADC(analogRead(PIN_VDIV_12V), 10, 3.32);
        float v5 = convertADC(analogRead(PIN_VDIV_5V), 10, 10);
        float v33 = convertADC(analogRead(PIN_VDIV_3V3), 10, 10);

        vicCAN.send(CMD_POWER_VOLTAGE, vBatt * 100, v12 * 100, v5 * 100, v33 * 100);
    }

    if (millis() - lastVersionSend > 5000) {
        lastVersionSend = millis();
        SEND_VERSION_INFO
    }

    //-------------//
    //  CAN Input  //
    //-------------//
    CanFrame rxFrame;
    bool isREV;
    if (vicCAN.readCan(&isREV, &rxFrame)) {
        const uint8_t commandID = vicCAN.getCmdId();
        static std::vector<double> canData;
        vicCAN.parseData(canData);

        Serial.print("VicCAN: ");
        vicCAN.printFrame(&Serial);

        // Misc

        if (commandID == CMD_PING) {
            vicCAN.respond(1);  // "pong"
            Serial.println("Received ping over CAN");
        } else if (commandID == CMD_B_LED) {
            if (canData.size() == 1) {
                if (canData[0] == 0)
                    digitalWrite(LED_BUILTIN, false);
                if (canData[0] == 1)
                    digitalWrite(LED_BUILTIN, true);
            }
        }

        // REV

        else if (commandID == CMD_REV_STOP) {
            Stop();
        } else if (commandID == CMD_REV_IDENTIFY) {
#ifndef FLIPSKY
            if (canData.size() == 1) {
                CAN_identifySparkMax(canData[0]);
            }
#endif
        } else if (commandID == CMD_REV_IDLE_MODE) {
            if (canData.size() == 1 && (canData[0] == 0 || canData[0] == 1)) {
                lastCtrlCmd = millis();
                for (int i = 0; i < 4; i++)
                    motorList[i]->setBrake(canData[0]);
            }
        } else if (commandID == CMD_REV_SET_DUTY) {
            if (canData.size() == 2) {
                lastCtrlCmd = millis();
                motorList[0]->sendDuty(canData[0]);
                motorList[1]->sendDuty(canData[0]);

                motorList[2]->sendDuty(-1 * canData[1]);
                motorList[3]->sendDuty(-1 * canData[1]);
            } else if (canData.size() == 4) {
                lastCtrlCmd = millis();
                motorList[0]->sendDuty(canData[0]);
                motorList[1]->sendDuty(canData[1]);

                motorList[2]->sendDuty(-1 * canData[2]);
                motorList[3]->sendDuty(-1 * canData[3]);
            }
        } else if (commandID == CMD_REV_SET_VELOCITY) {
            if (canData.size() == 2) {
                lastCtrlCmd = millis();
                motorList[0]->sendSpeed(canData[0]);
                motorList[1]->sendSpeed(canData[0]);

                motorList[2]->sendSpeed(-1 * canData[1]);
                motorList[3]->sendSpeed(-1 * canData[1]);
            } else if (canData.size() == 4) {
                lastCtrlCmd = millis();
                motorList[0]->sendSpeed(canData[0]);
                motorList[1]->sendSpeed(canData[1]);

                motorList[2]->sendSpeed(-1 * canData[2]);
                motorList[3]->sendSpeed(-1 * canData[3]);
            }
        }

        // Submodule-specific

        else if (commandID == CMD_CORE_LED_STRIP) {
            if (canData.size() == 4) {
                setLED(canData[0], canData[1], canData[2]);
            }
        }

        else if (commandID == CMD_CORE_TURN_TO) {
            if (canData.size() == 2 && canData[1] != 0) {
                turningToStatus.enabled = true;
                turningToStatus.targetHeading = canData[0];
                turningToStatus.timeoutStamp = millis() + canData[1] * 1000;
            }
        }
    }
#ifndef FLIPSKY
    else if (isREV) {  // REV Feedback
        // Serial.print("Received CAN frame from REV device: ");
        // printREVFrame(rxFrame);
        uint8_t deviceId = rxFrame.identifier & 0x3F;        // [5:0]
        uint32_t apiId = (rxFrame.identifier >> 6) & 0x3FF;  // [15:6]

#    if defined(DEBUG_STATUS)
        // Log message if it seems interesting
        if (apiId == 0x99 || (apiId & 0x60) == 0x60 || (apiId & 0x300) == 0x300) {
            printREVFrame(rxFrame);
        }
#    endif

        if ((apiId & 0x60) == 0x60) {  // Periodic status
            for (int i = 0; i < 4; i++) {
                if (deviceId == motorList[i]->getID()) {
                    motorList[i]->parseStatus(apiId, rxFrame.data);
                    break;
                }
            }
        } else if ((apiId & 0x300) == 0x300) {  // Parameter
            printREVParameter(rxFrame);
#    ifdef DEBUG
            Serial.print("From frame: ");
            printREVFrame(rxFrame);
#    endif
        }
    }
#endif

    //------------------//
    //  UART/USB Input  //
    //------------------//
    //
    //
    //-------------------------------------------------------//
    //                                                       //
    //      /////////    //\\        ////    //////////      //
    //    //             //  \\    //  //    //        //    //
    //    //             //    \\//    //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //      /////////    //            //    //////////      //
    //                                                       //
    //-------------------------------------------------------//
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');

        input.trim();                   // Remove preceding and trailing whitespace
        std::vector<String> args = {};  // Initialize empty vector to hold separated arguments
        parseInput(input,
                   args);          // Separate `input` by commas and place into args vector
        args[0].toLowerCase();     // Make command case-insensitive
        String command = args[0];  // To make processing code more readable

        String prevCommand;

        //--------//
        //  Misc  //
        //--------//
        if (command == "ping") {
            Serial.println("pong");
        }

        else if (command == "time") {
            Serial.println(millis());
        }

        else if (command == "led") {
            if (args[1] == "on")
                digitalWrite(LED_BUILTIN, HIGH);
            else if (args[1] == "off")
                digitalWrite(LED_BUILTIN, LOW);
            else if (args[1] == "toggle") {
                ledState = !ledState;
                digitalWrite(LED_BUILTIN, ledState);
            }
        }

        else if (command == "can_relay_tovic") {
            vicCAN.relayFromSerial(args);
        }

        else if (args[0] == "can_relay_mode" && args.size() == 2) {
            if (args[1] == "on") {
                vicCAN.relayOn();
            } else if (args[1] == "off") {
                vicCAN.relayOff();
            }
        }

        //-----------//
        //  Sensors  //
        //-----------//

        else if (args[0] == "data") {
            if (args[1] == "sendGPS") {
                outputGPS();
            }

            else if (args[1] == "sendIMU") {
                Serial.println(outputBno());
            }

            else if (args[1] == "sendBMP") {
                Serial.println(outputBmp());
            }

            else if (args[1] == "everything") {
                // Serial.println(outputGPS());
                Serial.println(outputBno());
                Serial.println(outputBmp());
            }

            else if (args[1] == "getOrientation") {
                Serial.printf("orientation,%f\n", getBNOOrient(bno));
            }
        }

        //------------//
        //  Physical  //
        //------------//

        // set LED strip color format: led_set,r,b,g
        else if (args[0] == "led_set") {
            int led_rgb[3] = {0, 0, 0};
            for (int i = 0; i < 3; i++) {
                led_rgb[i] = args.at(i + 1).toInt();
            }

            setLED(led_rgb[0], led_rgb[1], led_rgb[2]);
        }
    }
}

//------------------------------------------------------------------------------------------------//
//  Function definitions
//------------------------------------------------------------------------------------------------//
//
//
//----------------------------------------------------//
//                                                    //
//    //////////    //          //      //////////    //
//    //            //\\        //    //              //
//    //            //  \\      //    //              //
//    //////        //    \\    //    //              //
//    //            //      \\  //    //              //
//    //            //        \\//    //              //
//    //            //          //      //////////    //
//                                                    //
//----------------------------------------------------//

// Bypasses the acceleration to make the rover stop
// Should only be used for autonomy, but it could probably be used elsewhere
void Stop() {
    for (int i = 0; i < 4; i++) {
        motorList[i]->stop();
    }
}

// Prints the output of the BNO in one line
String outputBno() {
    float bnoData2[7];
    pullBNOData(bno, bnoData2);
    String output;

    output = "bno," + String(bnoData2[0]) + ',' + String(bnoData2[1]) + ',' + String(bnoData2[2]) + ',' +
             String(bnoData2[3]) + ',' + String(bnoData2[4]) + ',' + String(bnoData2[5]) + ',' +
             String(bnoData2[6]);

    // sprintf(output,"%f,%f,%f,%f,%f,%f,%f",bnoData2[0],bnoData2[1],bnoData2[2],bnoData2[3],bnoData2[4],bnoData2[5],bnoData2[6]);

    return output;
}

// Prints the output of the GPS in one line
String outputGPS() {
    String output = "null";
    double gpsData[3];
    getPosition(myGNSS, gpsData);
    // Serial.print("gps,");
    // Serial.print(gpsData[0],7);
    // Serial.print(",");
    // Serial.print(gpsData[1],7);
    // Serial.println();

    output = "gps," + String(gpsData[0]) + ',' + String(gpsData[1]) + '\n';

    return output;
}

// Prints the output of the BMP in one line
String outputBmp() {
    float bmpData[3];
    pullBMPData(bmp, bmpData);
    String output;

    output = "bmp," + String(bmpData[0]) + ',' + String(bmpData[1]) + ',' + String(bmpData[2]);

    // sprintf(output, "%f,%f,%f",bmpData[0],bmpData[1],bmpData[2]);

    return output;
}

// Finds out which direction the rover should turn
int findRotationDirection(float current_direction, float target_direction) {
    int cw_dist = target_direction - current_direction + 360;
    cw_dist %= 360;
    int ccw_dist = current_direction - target_direction + 360;
    ccw_dist %= 360;

    if (cw_dist <= ccw_dist) {
        return 1;  // Rotate CW if distance is 180 or less
    } else {
        return 0;  // Rotate CCW if distance is greater than 180
    }
}

void setLED(int r_val, int b_val, int g_val) {
    for (int i = 0; i < NUM_LEDS; ++i) {
        leds[i] = CRGB(r_val, b_val, g_val);
        FastLED.show();
    }
}

float clamp_angle(float angle) {
    angle = fmod(angle, 360.0);
    if (angle < 0) {
        angle += 360;
    }
    if (angle > 180) {
        angle -= 360;
    }
    return angle;
}
