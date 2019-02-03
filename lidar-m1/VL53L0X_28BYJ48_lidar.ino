//
// LIDAR M1 v.1
//

#define OUTPUT_PROCESSING

#include <Wire.h>

//BOF VL53L0X
#include <VL53L0X.h>
#define SENSOR1_PIN 11
#define SENSOR2_PIN 12
VL53L0X sensor1;
VL53L0X sensor2;
#define SENSOR_INTERVAL 20
unsigned long nextSensorTimer = 0;
byte nextSensorIndex = 1;
int sensor1Value = 0;
int sensor1Degrees = 0;
int sensor2Value = 0;
int sensor2Degrees = 0;

// 28BYJ-48
#include <AccelStepper.h>
#define stepperPin1  4     // IN1 on the ULN2003 driver 1
#define stepperPin2  5     // IN2 on the ULN2003 driver 1
#define stepperPin3  6     // IN3 on the ULN2003 driver 1
#define stepperPin4  7     // IN4 on the ULN2003 driver 1
AccelStepper stepper(AccelStepper::HALF4WIRE, stepperPin1, stepperPin3, stepperPin2, stepperPin4);

#define IVRX_THRS 6
#define STEPS_PER_TURN 4096
#define START_RANGE_COUNT_LIMIT 2
#define END_RANGE_COUNT_LIMIT 2

long stepperOrigin = 0;
boolean stepperOriginSet = false;
float stepperStepToRadians = 2 * PI /  STEPS_PER_TURN;

// IR sync
#define IVRX_DPIN 2
#define IVRX_APIN 0
#define LEDPIN LED_BUILTIN
#define SYNC_INTERVAL 20
unsigned long nextSyncTimer = 0;

// time control
unsigned long lastLoopMillis = 0;
byte loopTime;

void setup() {

  // IR sync
  pinMode(IVRX_DPIN, OUTPUT);  
  pinMode(LEDPIN, OUTPUT);  
  digitalWrite(IVRX_DPIN, HIGH);       // supply 5 volts to emitter photodiode TODO... get 5V on power? 
  digitalWrite(LEDPIN, LOW);        // builtin LED initially off
  nextSyncTimer = millis() + SYNC_INTERVAL;  

  //BOF VL53L0X
  // prepare address configuration
  pinMode(SENSOR1_PIN, OUTPUT);
  pinMode(SENSOR2_PIN, OUTPUT);
  digitalWrite(SENSOR1_PIN, LOW);
  digitalWrite(SENSOR2_PIN, LOW);

  // start i2c
  delay(500);
  Wire.begin();

  // start serial
  Serial.begin (115200);

  // sensor1 address configuration
  pinMode(SENSOR1_PIN, INPUT);
  delay(150);
  sensor1.init(true);
  delay(100);
  sensor1.setAddress((uint8_t)22);

  // sensor2 address configuration
  pinMode(SENSOR2_PIN, INPUT);
  delay(150);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)25);

  // configuration
  sensor1.setTimeout(500);
  sensor2.setTimeout(500);
  sensor1.startContinuous();
  sensor2.startContinuous();

  nextSensorTimer = millis() + SENSOR_INTERVAL;
  //EOF VL53L0X

  // 28BYJ-48
  stepper.setMaxSpeed(1500);
  stepper.setSpeed(1200);  

  // time control
  lastLoopMillis = millis();
  loopTime = 0;
}

void loop() {
  //
  // DEFINE ORIGIN
  // 
  static int syncVal = 0;
  static boolean startRangeSet = false;
  static long startRangeStep = 0;
  static byte startRangeCounter = 0;
  static long endRangeStep = 0;
  static byte endRangeCounter = 0;
  if(millis() > nextSyncTimer) {
    nextSyncTimer = millis() + SYNC_INTERVAL;
    syncVal = analogRead(IVRX_APIN);
    if(syncVal <= IVRX_THRS) {
      if(!startRangeSet) {
        if(startRangeCounter > START_RANGE_COUNT_LIMIT) {
          startRangeSet = true;
          startRangeStep = stepper.currentPosition ();
          startRangeCounter = 0;
          digitalWrite(LEDPIN, HIGH);
        } else {
          startRangeCounter++;
        }
      }
    }  else {
      if(startRangeSet) {
        if(endRangeCounter > END_RANGE_COUNT_LIMIT) {
          endRangeStep = stepper.currentPosition ();
          if(!stepperOriginSet) {
            stepperOrigin = startRangeStep + (endRangeStep - startRangeStep);
            stepperOriginSet = true;
          }
          startRangeSet = false;
          endRangeCounter = 0;
          digitalWrite(LEDPIN, LOW);
        } else {
          endRangeCounter++;
        }
      }    
    }  
  }
  
  //
  // DO VL53L0X READINGS
  //
  static long stepNumber;
  long relativeStep;
  if(stepperOriginSet && millis() > nextSensorTimer) {
    nextSensorTimer = millis() + SENSOR_INTERVAL;
    stepNumber = stepper.currentPosition() - stepperOrigin;
    relativeStep = stepNumber % STEPS_PER_TURN;
    switch(nextSensorIndex) {
      case 1:
        sensor1Value = sensor1.readRangeContinuousMillimeters();
        sensor1Degrees = relativeStep * stepperStepToRadians * 180 / PI;
        if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT S1 "); }        
        nextSensorIndex++;
        #ifdef OUTPUT_PROCESSING
          Serial.print(sensor1Degrees); Serial.print(" "); Serial.println(sensor1Value);
        #endif
        break;
      case 2:
        sensor2Value = sensor2.readRangeContinuousMillimeters();
        sensor2Degrees = relativeStep * stepperStepToRadians * 180 / PI - 180;
        if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT S2 "); }
        nextSensorIndex = 1;
        #ifdef OUTPUT_PROCESSING
          Serial.print(sensor2Degrees); Serial.print(" "); Serial.println(sensor2Value);
        #endif
        break;
    }
    
    /*
    // DEBUG OUTPUT
    Serial.print(syncVal); Serial.print("\t");
    Serial.print(stepperOrigin); Serial.print("\t");
    Serial.print(relativeStep * stepperStepToRadians * 180 / PI); Serial.print("\t");
    Serial.print(sensor1Value); Serial.print("\t");
    Serial.print(sensor2Value); Serial.print("\t");
    Serial.print(loopTime); Serial.print("\t");
    Serial.println();
    */    
  }

  // 28BYJ-48
  stepper.runSpeed();

  // loop time control
  loopTime = millis() - lastLoopMillis;
  lastLoopMillis = millis();
}
