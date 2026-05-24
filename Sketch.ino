#include <ESP32Servo.h>
// Group GC-1 2: Jacob Mereszko, Dilshan Leanage, Jess Eden

//Pins
#define STEP_PIN  12   // STEP
#define DIR_PIN 13   // DIR

//Sensor/Input
#define PIR_PIN 14
#define DOOR_BTN_PIN  2
#define ARM_BTN_PIN 4
#define LDR_PIN 34
#define TRIG_PIN  5
#define ECHO_PIN  18

//Output
#define BUZZER_PIN  26
#define LED_RED_PIN 25
#define LED_BLUE_PIN  27
#define LED_YEL_PIN 33
#define SERVO_PIN 32

//Stepper Configuration
//NEMA 17 = 200 steps/rev at full step
//Adjust LOCK_STEPS to change how far the door bolt turns
#define STEPS_PER_REV 200
#define LOCK_STEPS  50//quarter turn to lock
#define STEP_DELAY_US 800//microseconds between pulses

//Servo Config
#define SERVO_WINDOW_LOCKED 90
#define SERVO_WINDOW_UNLOCKED 0

//Tunable Constant
#define LDR_NIGHT_THRESHOLD 1800
#define DETECT_DISTANCE_CM  50
#define CAUTION_CONFIRM_MS  5000
#define ALARM_TIMEOUT_MS  10000
#define POLL_IDLE_MS  500
#define POLL_ACTIVE_MS  100
#define BLINK_INTERVAL_MS 300
#define DEBOUNCE_MS 250
#define FAULT_THRESHOLD 3

//FSM States
enum SystemState : uint8_t {DISARMED, ARMED, MONITORING, CAUTION, ALARM, FAILSAFE};
const char* STATES[] = {"DISARMED", "ARMED", "MONITORING", "CAUTION", "ALARM", "FAILSAFE"};

SystemState currentState = DISARMED;
Servo windowServo;

//Initialisation of Global Variables 

//Lock States
bool isDoorLocked = false;
bool isWindowLocked = false;

//Sensor States
bool motionDetected = false;
bool proximityAlert = false;
bool doorOpen = false;
bool isNight = false;
int  distanceCM = 0;
bool blinkLedState = false;

//Timers
unsigned long lastPollTime = 0;
unsigned long cautionStartTime = 0;
unsigned long alarmStartTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastArmPress = 0;
unsigned long lastDoorPress = 0;
int  sensorFaultCount = 0;

//Declaring Functions
void readSensors();
void runFSM();
void enterState(SystemState s);
void setLEDs(bool red, bool blue, bool yellow);
void blinkAlarmLEDs();
void buzzerOff();
void lockDoor();
void unlockDoor();
void lockWindow();
void unlockWindow();
void stepMotor(int steps, bool clockwise);
int  readUltrasonic();
bool validateSensors();
unsigned long pollInterval();

#define DIST_ALPHA 0.1
#define DIST_THRESHOLD_RATIO 0.7
float triggerDistance = DETECT_DISTANCE_CM;
void updateTriggerDistance(int distanceCM);

#define LDR_WINDOW_MS 5000
float averagedLdrVal;
void updateLdrVal();

//Setup Process
void setup() {
  //Initialise Serial Communication
  Serial.begin(115200);
  Serial.println("SecuraHome Initialising...");

  //Input Pins
  pinMode(PIR_PIN, INPUT);
  pinMode(DOOR_BTN_PIN, INPUT_PULLUP);
  pinMode(ARM_BTN_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  //LDR Pin
  pinMode(LDR_PIN, INPUT);
  averagedLdrVal = analogRead(LDR_PIN);

  //Output Pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_YEL_PIN, OUTPUT);

  //Stepper Motor Pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  //Servo Pins
  windowServo.attach(SERVO_PIN);
  windowServo.write(SERVO_WINDOW_UNLOCKED);

  //Set initial state
  enterState(DISARMED);
  Serial.println("Ready. Press GREEN button to ARM house.");
}

//Main Loop - Reponsible for continuously polling all sensors and inputs, 
//allowing for immediate system state transitions without blocking delays
void loop() {
  unsigned long now = millis();

  //ARM/DISARM button
  //Allows for transition between ARMED and DISARMED State with debounce protection 
  if (digitalRead(ARM_BTN_PIN) == LOW && (now - lastArmPress) > DEBOUNCE_MS) {
    lastArmPress = now;
    if (currentState == DISARMED) {
      enterState(ARMED);
    } else {
      enterState(DISARMED);
    }
  }

  //OPEN/CLOSE door
  //Toggles door open/closed and logs the change within the system
  if (digitalRead(DOOR_BTN_PIN) == LOW && (now - lastDoorPress) > DEBOUNCE_MS) {
    lastDoorPress = now;
    doorOpen = !doorOpen;
    if (doorOpen) {
      Serial.println("[Door] OPEN");
    } else {
      Serial.println("[Door] CLOSED");
    }
  }

  //Sensor polling
  //Only polls sensors when the system is active and not within the DISARMED OR FAILSAFE states
  //Poll rate varies depending on whether the system is within a high alert state or within normal monitoring
  if (currentState != DISARMED && currentState != FAILSAFE) {
    if (now - lastPollTime >= pollInterval()) {
      lastPollTime = now;
      readSensors();
    }
  }

  //Running FSM
  //Responsible for changing between different states based on the gathered information from inputs
  runFSM();
}

//Sensor Polling
void readSensors() {
  //Updates Motion Sensor readings
  //High signal indicates motion has been detected
  motionDetected = (digitalRead(PIR_PIN) == HIGH);

  //Updates Utrasonic Distance Sensor readings
  //Reads distance and updates triggerDistance based on certain conditions
  //Based on the distance, proximityAlert will vary based on the severity of how close a potential threat is
  distanceCM = readUltrasonic();
  updateTriggerDistance(distanceCM);
  proximityAlert = (distanceCM > 0 && distanceCM < triggerDistance * DIST_THRESHOLD_RATIO);

  //Updates LDR Readings
  //Determines whether it is night or day depending on the value received by the LDR
  updateLdrVal();
  isNight = (averagedLdrVal > LDR_NIGHT_THRESHOLD);

  //Determines if a sensor receives a faulty input
  //Transitioning the system into a FAILSAFE state if true
  if (!validateSensors()) {
    sensorFaultCount++;
    if (sensorFaultCount >= FAULT_THRESHOLD) {
      enterState(FAILSAFE);
      return;
    }
  } else {
    sensorFaultCount = 0;
  }

  //Responsible for displaying information to the terminal
  Serial.printf("[Sensors] PIR:%d", motionDetected);
  Serial.printf("  Prox:%d (%dcm)", proximityAlert, distanceCM);
  Serial.printf(" Door:");
  if (doorOpen) {
    Serial.print("OPEN");
  } else {
    Serial.print("CLOSED");
  }
  Serial.printf("  Night:%d", isNight);
  Serial.printf("  LDR:%.0f\n", averagedLdrVal);
}

//Ultrasonic Distance Sensor Reader
int readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    return -1;
  }
  return (int)(duration * 0.034 / 2);
}

//Updates Trigger Distance
//Only updates if valid motion during the day and not in an alarm state
//Uses a moving average to adapt the trigger distance based on the foot traffic within the day
//Ultimately changing the distance required to trigger caution to account for busier or quieter periods
void updateTriggerDistance(int distanceCM) {
  if (!motionDetected || isNight || currentState != MONITORING || distanceCM < 0) return;
  triggerDistance = DIST_ALPHA * (float)distanceCM + (1.0f - DIST_ALPHA) * triggerDistance;
  triggerDistance = constrain(triggerDistance, 100.0f, 1000.0f);
  Serial.printf("[Learn] triggerDistance: %.1fcm  threshold: %.1fcm\n", 
                triggerDistance, triggerDistance * DIST_THRESHOLD_RATIO);
}

//Updates LDR values
//Implemented a moving average to account for cloudcover or torches
void updateLdrVal() {
  int pollMS = pollInterval();
  float alpha = 1 - exp(-(float)pollMS/LDR_WINDOW_MS);
  averagedLdrVal = alpha*analogRead(LDR_PIN) + (1-alpha)*averagedLdrVal;
}

//Checks fuctionality of LDR sensor
//If reading extend out of determined range, the sensor is considered faulty
bool validateSensors() {
  int ldrVal = analogRead(LDR_PIN);
  return (ldrVal >= 10 && ldrVal <= 4085);
}

//Determines Poll Intervals
//Will vary polling depending on the state
//CAUTION and ALARM state allows for fasting polling times
//While MONITORING state is provided a lower polling time
unsigned long pollInterval() {
  if (currentState == CAUTION || currentState == ALARM){
    return POLL_ACTIVE_MS;
  }
  return POLL_IDLE_MS;
}

//FSM thinking
//Responsible for changing between different states
//The monitoringfusion variable is responsible for acting as a fusion score, scaling the threat relevance
//States escalate in severity from: DISARMED -> ARMED -> MONITORING -> CAUTION -> ALARM
//Thus, the system is able to respond proportionally to the situation
void runFSM() {
  unsigned long now = millis();
  float monitoringFusion = 3*proximityAlert + 2*motionDetected + 1.5*doorOpen + 1*isNight;
  switch (currentState) {

    case DISARMED: //System inactive
      break;

    case ARMED: //Transition from ARMED state to MONITORING state
      enterState(MONITORING);
      break;

    case MONITORING: //Escalates to CAUTION if fusion score exceeds danger threshold
      if (monitoringFusion >= 4) {
        Serial.println("[MONITORING] Danger threshold met - escalating");
        enterState(CAUTION);
      }
      break;

    case CAUTION: //Deescalates to MONTIORING if fusion score reduces
      if (monitoringFusion < 1.5) {
        Serial.println("[CAUTION] Threat resolved. Back to MONITORING.");
        enterState(MONITORING);
      } else if (now - cautionStartTime >= CAUTION_CONFIRM_MS) { //If threat persists, escalates to ALARM
        enterState(ALARM);
      }
      break;

    case ALARM: //Blinks alarm LEDs and returns to monitoring after a certain time
      blinkAlarmLEDs();
      if (now - alarmStartTime >= ALARM_TIMEOUT_MS) {
        Serial.println("[ALARM] Timeout. Resetting to MONITORING.");
        enterState(MONITORING);
      }
      break;

    case FAILSAFE: //System inactive due to sensor faults
      break;
  }
}


//Transitions between states
//Handles activing LEDs, buzzer and locks to match the threat level
void enterState(SystemState newState) {
  if (newState == currentState){
    return;
  }

  Serial.printf("\n>>> %s --> %s\n", STATES[currentState], STATES[newState]);
  currentState = newState;

  switch (newState) {

    case DISARMED: //System is fully reset, with all outputs and locks off
      setLEDs(false, false, false);
      buzzerOff();
      unlockDoor();
      unlockWindow();
      doorOpen = false;
      sensorFaultCount = 0;
      Serial.println("[DISARMED] All locks released. System off.");
      break;

    case ARMED: //Actives Blue led
      setLEDs(false, true, false);
      buzzerOff();
      Serial.println("[ARMED] Blue LED on. Entering monitoring...");
      break;

    case MONITORING: //Locks are released and blue led continues to function
      setLEDs(false, true, false);
      buzzerOff();
      unlockDoor();
      unlockWindow();
      Serial.println("[MONITORING] Standby. Sensor fusion active.");
      break;

    case CAUTION: //Locks are armed and tracks the amount of time within the CAUTION state
      setLEDs(true, false, false);
      buzzerOff();
      lockDoor();
      lockWindow();
      cautionStartTime = millis();
      Serial.println("[CAUTION] Door + Window locked! Confirming threat...");
      break;

    case ALARM: //All locks engaged, tracks the amount of time within the ALARM state and flashes LEDs
      lockDoor();
      lockWindow();
      alarmStartTime = millis();
      lastBlinkTime  = millis();
      blinkLedState  = false;
      Serial.println("[ALARM] !! ALARM TRIGGERED !! All locks engaged.");
      break;

    case FAILSAFE: //Actives yellow led, all locks are engaged and awaits for manual reset
      setLEDs(false, false, true);
      buzzerOff();
      lockDoor();
      lockWindow();
      Serial.println("[FAILSAFE] Sensor fault! All locks engaged. Press GREEN to reset.");
      break;
  }
}

//FSM ACTING
//Blue Led indicates MONITORING and ARMED
//Red Led indicates CAUTION
//Yellow Led indicates FAILSAFE
//When in ALARM state, system will flash between red and blue
void setLEDs(bool red, bool blue, bool yellow) {
  digitalWrite(LED_RED_PIN, red);
  digitalWrite(LED_BLUE_PIN, blue);
  digitalWrite(LED_YEL_PIN, yellow);
}

//Allows for blinking between red and blue led when in ALARM state
void blinkAlarmLEDs() {
  unsigned long now = millis();
  if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
    lastBlinkTime = now;
    //flips the blink state each time the interval passes
    blinkLedState = !blinkLedState;
    if (blinkLedState) {
      digitalWrite(LED_RED_PIN,  HIGH);
      digitalWrite(LED_BLUE_PIN, LOW);
      tone(BUZZER_PIN, 2000);
    } else {
      digitalWrite(LED_RED_PIN,  LOW);
      digitalWrite(LED_BLUE_PIN, HIGH);
      tone(BUZZER_PIN, 500); 
    }
    // yellow always stays off during alarm
    digitalWrite(LED_YEL_PIN, LOW);
  }
}

//Turns Buzzer off
void buzzerOff() { 
  noTone(BUZZER_PIN); 
  digitalWrite(BUZZER_PIN, LOW); 
}


//Send pulses to the STEP pin in the chosen direction
void stepMotor(int steps, bool clockwise) {
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  delayMicroseconds(1);  //direction setup time
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

//Stepper locks door
void lockDoor() {
  if (isDoorLocked){
    return;
  }
  Serial.println("[Stepper] Locking DOOR...");
  stepMotor(LOCK_STEPS, true); //clockwise = lock
  isDoorLocked = true;
  Serial.println("[Stepper] DOOR LOCKED");
}

//Stepper unlocks door
void unlockDoor() {
  if (!isDoorLocked) return;
  Serial.println("[Stepper] Unlocking DOOR...");
  stepMotor(LOCK_STEPS, false);  //counter-clockwise = unlock
  isDoorLocked = false;
  Serial.println("[Stepper] DOOR UNLOCKED");
}

// Servo window lock
void lockWindow() {
  if (isWindowLocked){
    return;
  }
  Serial.println("[Servo] Locking WINDOW...");
  windowServo.write(SERVO_WINDOW_LOCKED);
  isWindowLocked = true;
  Serial.println("[Servo] WINDOW LOCKED (90deg)");
}

// Servo window unlock
void unlockWindow() {
  if (!isWindowLocked){
    return;
  }
  Serial.println("[Servo] Unlocking WINDOW...");
  windowServo.write(SERVO_WINDOW_UNLOCKED);
  isWindowLocked = false;
  Serial.println("[Servo] WINDOW UNLOCKED (0deg)");
}