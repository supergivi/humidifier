#include <math.h>
#include <stdint.h>
#include "SparkFunBME280.h"
#include "Wire.h"
#include <avr/wdt.h>

BME280 mySensor;

#define STBY  13
#define PWMA  3
#define AIN1  9
#define AIN2  8
#define PWMB  11
#define BIN1  14
#define BIN2  12
#define WATER  4
#define PUMP  2
#define FAN  1
#define RED  5
#define GREEN  10
#define BLUE  6


bool waterPresent;
bool intermediateWaterPresent;
float currentHumidity;
double currentAbsoluteHumidity;

float currentTemperature;
bool readHumiditySensorError;
unsigned long clock;

unsigned long pumpStartAt;
unsigned long pumpStopAt;

unsigned long lastReadHumidifierSensorAt;
unsigned long lastReadWaterSensorAt;
unsigned long ultrasonicStartAt;
unsigned long ultrasonicStopAt;
unsigned long updatedHumidifierStatusAt;
unsigned long humidifierStartAt;
unsigned long humidifierStopAt;
unsigned long lastLedClockAt;
bool ledUp;

float minOptimumAbsoluteHumidity;
float maxOptimumAbsoluteHumidity;
int currentFanSpeed;
int lastGreenLedLevel;
int lastRedLedLevel;
int lastBlueLedLevel;

int waterCounter;
int noWaterCounter;


void setup() {
    TCCR2B = TCCR2B & B11111000 | B00000001;
    waterPresent = true;
    currentFanSpeed = 0;
    pumpStartAt = 0;
    pumpStopAt = 1;
    lastReadHumidifierSensorAt = 0;
    ultrasonicStartAt = 0;
    ultrasonicStopAt = 1;
    humidifierStartAt = 0;
    humidifierStopAt = 1;
    updatedHumidifierStatusAt = 0;
    lastLedClockAt = 0;
    lastGreenLedLevel = 0;
    lastRedLedLevel = 0;
    lastBlueLedLevel = 0;
    noWaterCounter = 0;
    waterCounter = 0;

    minOptimumAbsoluteHumidity = 0.009 - 0.00045;
    maxOptimumAbsoluteHumidity = 0.009 + 0.00045;
    ledUp = true;
    Serial.begin(9600);
    mySensor.settings.commInterface = I2C_MODE;
    mySensor.settings.I2CAddress = 0x76;
    mySensor.settings.runMode = 3; //Normal mode
    mySensor.settings.tStandby = 0;
    mySensor.settings.filter = 0;
    mySensor.settings.tempOverSample = 1;
    mySensor.settings.pressOverSample = 1;
    mySensor.settings.humidOverSample = 1;
    delay(10);
    mySensor.begin();

    pinMode(7, OUTPUT);

    pinMode(RED, OUTPUT);
    analogWrite(RED, 0);
    pinMode(BLUE, OUTPUT);
    analogWrite(BLUE, 0);

    pinMode(GREEN, OUTPUT);
    analogWrite(GREEN, 0);

    pinMode(STBY, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(WATER, INPUT_PULLUP);
    stopPump();
    stopHumidifier();
    setFanSpeed(80);
    digitalWrite(7, HIGH);
    wdt_enable(WDTO_8S);
}

void loop() {
    wdt_reset();

    if ((lastReadHumidifierSensorAt + 10000) < millis()) {
        lastReadHumidifierSensorAt = millis();
        readHumiditySensor();
        currentAbsoluteHumidity = getAbsoluteHumidity(currentTemperature, currentHumidity);


        Serial.print((float) (currentHumidity), 5);
        Serial.print(" ");
        Serial.println((float) (currentAbsoluteHumidity), 5);
        Serial.print(" ");
        Serial.println((float) (getAbsoluteHumidity2(currentTemperature, currentHumidity)), 5);
        Serial.println((float) (currentTemperature), 5);

    }

    if ((lastReadWaterSensorAt + 2000) < millis()) {
        lastReadWaterSensorAt = millis();
        bool v = !digitalRead(WATER);
        if (v){
           noWaterCounter = 0;
            if (waterCounter < 5){
                waterCounter += 1;
            } else {
                intermediateWaterPresent = true;
            }
        } else {
            waterCounter = 0;
            if (noWaterCounter < 5){
                noWaterCounter += 1;
            } else {
                intermediateWaterPresent = false;
            }
        }

        if (intermediateWaterPresent && !waterPresent) {
            waterPresent = true;
        }
        if (intermediateWaterPresent && isPumpOn()) {
            stopPump();
        } else if (!intermediateWaterPresent && !isPumpOn() && waterPresent) {
            startPump();
        }
        if (isPumpOn() && (pumpStartAt + 120000) < millis()) {
            stopPump();
            waterPresent = false;
        }
    }

    if ((updatedHumidifierStatusAt + 10000) < millis()) {
        updatedHumidifierStatusAt = millis();
        if (waterPresent) {
            if (currentAbsoluteHumidity < minOptimumAbsoluteHumidity) {
                startHumidifier();
            }

            if (currentAbsoluteHumidity > maxOptimumAbsoluteHumidity) {
                stopHumidifier();
            }
        } else {
            stopHumidifier();
        }
    }

    if ((clock + 300) < millis()) {
        clock = millis();
        if (isHumidifierOn()) {
            if (currentFanSpeed < 200) {
                increaseFanSpeed();
            } else if (!isUltrasonicOn()) {
                startUltraSonic();
            } else if ((humidifierStartAt + 300000) < millis() && currentFanSpeed < 255) {
                increaseFanSpeed();
            } else if ((humidifierStartAt + 600000) < millis() && currentFanSpeed < 255) {
                increaseFanSpeed();
            } else if ((humidifierStartAt + 1600000) < millis() && currentFanSpeed < 255) {
                increaseFanSpeed();
            }
        } else {
            if (isUltrasonicOn()) {
                stopUltraSonic();
            }
            if (currentFanSpeed > 80) {
                decreaseFanSpeed();
            }
        }
    }

    if (!waterPresent) {
        if ((lastLedClockAt + 300) < millis()) {
            lastLedClockAt = millis();
            setGreenLedLevel(0);
            setRedLedLevel(0);
            setBlueLedLevel(lastBlueLedLevel > 125 ? 0 : 255);
        }
    } else if (currentAbsoluteHumidity > 0) {
        if (currentAbsoluteHumidity >= (minOptimumAbsoluteHumidity - 0.0002) &&
            currentAbsoluteHumidity <= (maxOptimumAbsoluteHumidity + 0.0002)) {
            if ((lastLedClockAt + 300) < millis()) {
                lastLedClockAt = millis();
                int greenLedLevel = getLevelOfHumidity();
                int redLedLevel = 255 - greenLedLevel;

                if (lastBlueLedLevel > 0) {
                    setBlueLedLevel(lastBlueLedLevel - 1);

                }

                if (lastGreenLedLevel < greenLedLevel) {
                    greenLedLevel = lastGreenLedLevel + 1;
                } else if (lastGreenLedLevel > greenLedLevel) {
                    greenLedLevel = lastGreenLedLevel - 1;
                }
                setGreenLedLevel(greenLedLevel);

                if (lastRedLedLevel < redLedLevel) {
                    redLedLevel = lastRedLedLevel + 1;
                } else if (lastRedLedLevel > redLedLevel) {
                    redLedLevel = lastRedLedLevel - 1;
                }
                setRedLedLevel(redLedLevel);

            }
        } else if (currentAbsoluteHumidity < (minOptimumAbsoluteHumidity - 0.0002)) {
            if ((lastLedClockAt + 5) < millis()) {
                lastLedClockAt = millis();
                if (lastGreenLedLevel > 0) {
                    setGreenLedLevel(lastGreenLedLevel - 1);
                }
                if (lastBlueLedLevel > 0) {
                    setBlueLedLevel(lastBlueLedLevel - 1);
                }
                if (ledUp) {
                    if (lastRedLedLevel < 255) {
                        setRedLedLevel(lastRedLedLevel + 1);
                    } else {
                        ledUp = false;
                    }
                } else {
                    if (lastRedLedLevel > 20) {
                        setRedLedLevel(lastRedLedLevel - 1);
                    } else {
                        ledUp = true;
                    }
                }
            }

        } else if (currentAbsoluteHumidity > (maxOptimumAbsoluteHumidity + 0.0002)) {
            if ((lastLedClockAt + 5) < millis()) {
                lastLedClockAt = millis();
                if (lastGreenLedLevel > 0) {
                    setGreenLedLevel(lastGreenLedLevel - 1);
                }
                if (lastRedLedLevel > 0) {
                    setRedLedLevel(lastRedLedLevel - 1);

                }

                if (ledUp) {
                    if (lastBlueLedLevel < 255) {
                        setBlueLedLevel(lastBlueLedLevel + 1);
                    } else {
                        ledUp = false;
                    }
                } else {
                    if (lastBlueLedLevel > 20) {
                        setBlueLedLevel(lastBlueLedLevel - 1);
                    } else {
                        ledUp = true;
                    }
                }
            }
        }
    } else {
        setGreenLedLevel(0);
        setBlueLedLevel(255);
        setRedLedLevel(0);
    }


}

void setRedLedLevel(int level) {
    analogWrite(RED, level);
    lastRedLedLevel = level;
}

void setGreenLedLevel(int level) {
    analogWrite(GREEN, level);
    lastGreenLedLevel = level;
}

void setBlueLedLevel(int level) {
    analogWrite(BLUE, level);
    lastBlueLedLevel = level;
}


int getLevelOfHumidity() {
    float a = maxOptimumAbsoluteHumidity - minOptimumAbsoluteHumidity;
    float b = currentAbsoluteHumidity - minOptimumAbsoluteHumidity;
    float c = 255 / a;
    float d = b * c;
    if (d > 255) {
        d = 255;
    }
    if (d < 0) {
        d = 0;
    }
    return (int(d));
}

void move(int motor, int speed, int direction) {
    //Move specific motor at speed and direction
    //motor: 0 for B 1 for A
    //speed: 0 is off, and 255 is full speed
    //direction: 0 clockwise, 1 counter-clockwise

    digitalWrite(STBY, HIGH); //disable standby

    boolean inPin1 = LOW;
    boolean inPin2 = HIGH;

    if (direction == 1) {
        inPin1 = HIGH;
        inPin2 = LOW;
    }

    if (motor == 1) {
        digitalWrite(AIN1, inPin1);
        digitalWrite(AIN2, inPin2);
        analogWrite(PWMA, speed);
    } else {
        digitalWrite(BIN1, inPin1);
        digitalWrite(BIN2, inPin2);
        analogWrite(PWMB, speed);
    }
}


void stop() {
    //enable standby
    digitalWrite(STBY, LOW);
}

void stopPump() {
    if (isPumpOn()) {
        pumpStopAt = millis();
        move(PUMP, 0, 1);
    }
}

void startPump() {
    if (!isPumpOn()) {
        pumpStartAt = millis();
        move(PUMP, 255, 1); //motor 1, half speed, right
    }
}

void startUltraSonic() {
    if (!isUltrasonicOn()) {
        ultrasonicStartAt = millis();
        digitalWrite(7, LOW);
    }
}

void stopUltraSonic() {
    if (isUltrasonicOn()) {
        ultrasonicStopAt = millis();
        digitalWrite(7, HIGH);
    }
}

bool isUltrasonicOn() {
    return (ultrasonicStopAt < ultrasonicStartAt);
}

bool isPumpOn() {
    return (pumpStopAt < pumpStartAt);
}

bool isHumidifierOn() {
    return (humidifierStopAt < humidifierStartAt);
}

void setFanSpeed(int speed) {
    currentFanSpeed = speed;
    move(FAN, speed, 1);
}

void increaseFanSpeed() {
    setFanSpeed(currentFanSpeed + 1);
}

void decreaseFanSpeed() {
    setFanSpeed(currentFanSpeed - 1);
}

void startHumidifier() {
    if (!isHumidifierOn()) {
        humidifierStartAt = millis();
    }
}

void stopHumidifier() {
    if (isHumidifierOn()) {
        humidifierStopAt = millis();
    }
}
// float AbsoluteHumidity = (6.112 * pow(2.71828,((17.67 * Temperature)/(Temperature + 243.5))) * Humidity * 2.1674) / (273.15 + Temperature);

double getAbsoluteHumidity(double t, double rh) {
    double p = 760;
    double tp = (p * 133.322) / 100;
    double et = 6.112 * exp((17.62 * t) / (243.12 + t));
    double fp = 1.0016 + 3.15 * pow(10, -6) * tp - 0.074 / tp;
    double ew = fp * et;
    double e = (rh / 100) * ew;
    double Rv = 461.5;
    double T = t + 273.15;
    double absolute = (e * 100 / (Rv * T));
    return (absolute);
}

double getAbsoluteHumidity2(float t, float h)
{
    double temp;
    temp = pow(2.718281828, (17.67 * t) / (t + 243.5));
    return (6.112 * temp * h * 2.1674) / (273.15 + t);
}

void readHumiditySensor() {
    currentHumidity = mySensor.readFloatHumidity();
    currentTemperature = mySensor.readTempC();
    if (isnan(currentHumidity) || isnan(currentTemperature)) {
        readHumiditySensorError = true;
    } else {
        readHumiditySensorError = false;
    }
}





