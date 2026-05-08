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
#define INTRUSION_CONFIRM_MS  2000
#define ALARM_TIMEOUT_MS  10000
#define POLL_IDLE_MS  500
#define POLL_ACTIVE_MS  100
#define BLINK_INTERVAL_MS 300
#define DEBOUNCE_MS 250
#define FAULT_THRESHOLD 3

//FSM States
enum SystemState : uint8_t {DISARMED, ARMED, MONITORING, INTRUSION, ALARM, FAILSAFE};

const char* STATES[] = {"DISARMED", "ARMED", "MONITORING", "INTRUSION", "ALARM", "FAILSAFE"};

SystemState currentState = DISARMED;
Servo windowServo;

bool isDoorLocked = false;
bool isWindowLocked = false;

bool motionDetected = false;
bool proximityAlert = false;
bool doorOpen = false;
bool isNight = false;
int  distanceCM = 0;

unsigned long lastPollTime = 0;
unsigned long intrusionStartTime = 0;
unsigned long alarmStartTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastArmPress = 0;
unsigned long lastDoorPress = 0;

bool blinkLedState = false;
int  sensorFaultCount = 0;

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

void setup() {
  Serial.begin(115200);
  Serial.println("SecuraHome Initialising...");

  pinMode(PIR_PIN, INPUT);
  pinMode(DOOR_BTN_PIN, INPUT_PULLUP);
  pinMode(ARM_BTN_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_YEL_PIN, OUTPUT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  windowServo.attach(SERVO_PIN);
  windowServo.write(SERVO_WINDOW_UNLOCKED);

  enterState(DISARMED);
  Serial.println("Ready. Press GREEN button to ARM house.");
}


void loop() {
  unsigned long now = millis();

  //ARM/DISARM button
  if (digitalRead(ARM_BTN_PIN) == LOW && (now - lastArmPress) > DEBOUNCE_MS) {
    lastArmPress = now;
    if (currentState == DISARMED) {
      enterState(ARMED);
    } else {
      enterState(DISARMED);
    }
  }

  //OPEN/CLOSE door
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
  if (currentState != DISARMED && currentState != FAILSAFE) {
    if (now - lastPollTime >= pollInterval()) {
      lastPollTime = now;
      readSensors();
    }
  }

  runFSM();
}

//FSM sensing
void readSensors() {
  bool pirTriggered = (digitalRead(PIR_PIN) == HIGH);

  distanceCM = readUltrasonic();
  proximityAlert = (distanceCM > 0 && distanceCM < DETECT_DISTANCE_CM);

  //Sensor fusion: PIR or Proximity = motion detected
  motionDetected = pirTriggered || proximityAlert;

  int ldrVal = analogRead(LDR_PIN);
  isNight = (ldrVal > LDR_NIGHT_THRESHOLD);

  if (!validateSensors()) {
    sensorFaultCount++;
    Serial.printf("[WARN] Sensor fault #%d\n", sensorFaultCount);
    if (sensorFaultCount >= FAULT_THRESHOLD) {
      enterState(FAILSAFE);
      return;
    }
  } else {
    sensorFaultCount = 0;
  }

  Serial.printf("[Sensors] PIR:%d", pirTriggered);
  Serial.printf("  Prox: %d", proximityAlert, "(%d cm)", distanceCM);
  Serial.printf(" Door:");
  if (doorOpen) {
    Serial.print("OPEN");
  } else {
    Serial.print("CLOSED");
  }
  Serial.printf("  Night:%d", isNight);
  Serial.printf("  LDR:%d\n", ldrVal);
}

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

bool validateSensors() {
  int ldrVal = analogRead(LDR_PIN);
  return (ldrVal >= 0 && ldrVal <= 4095);
}

unsigned long pollInterval() {
  if (currentState == INTRUSION || currentState == ALARM){
    return POLL_ACTIVE_MS;
  }
  return POLL_IDLE_MS;
}

//FSM thinking
void runFSM() {
  unsigned long now = millis();

  switch (currentState) {

    case DISARMED:
      break;

    case ARMED:
      enterState(MONITORING);
      break;

    case MONITORING:
      if (motionDetected && doorOpen) {
        Serial.println("[MONITORING] Motion + open door detected!");
        enterState(INTRUSION);
      } else if (isNight && motionDetected) {
        Serial.println("[Night Mode] Elevated sensitivity – escalating.");
        enterState(INTRUSION);
      }
      break;

    case INTRUSION:
      if (now - intrusionStartTime >= INTRUSION_CONFIRM_MS) {
        enterState(ALARM);
      } else if (!motionDetected && !doorOpen && !isNight) {
        Serial.println("[INTRUSION] Threat resolved. Back to MONITORING.");
        enterState(MONITORING);
      }
      break;

    case ALARM:
      blinkAlarmLEDs();
      if (now - alarmStartTime >= ALARM_TIMEOUT_MS) {
        Serial.println("[ALARM] Timeout. Resetting to MONITORING.");
        enterState(MONITORING);
      }
      break;

    case FAILSAFE:
      break;
  }
}


//Transitions between states
void enterState(SystemState newState) {
  if (newState == currentState){
    return;
  }

  Serial.printf("\n>>> %s --> %s\n", STATES[currentState], STATES[newState]);
  currentState = newState;

  switch (newState) {

    case DISARMED:
      setLEDs(false, false, false);
      buzzerOff();
      unlockDoor();
      unlockWindow();
      doorOpen = false;
      sensorFaultCount = 0;
      Serial.println("[DISARMED] All locks released. System off.");
      break;

    case ARMED:
      setLEDs(false, true, false);
      buzzerOff();
      Serial.println("[ARMED] Blue LED on. Entering monitoring...");
      break;

    case MONITORING:
      setLEDs(false, true, false);
      buzzerOff();
      unlockDoor();
      unlockWindow();
      Serial.println("[MONITORING] Standby. Sensor fusion active.");
      break;

    case INTRUSION:
      setLEDs(true, false, false);
      buzzerOff();
      lockDoor();
      lockWindow();
      intrusionStartTime = millis();
      Serial.println("[INTRUSION] Door + Window locked! Confirming threat...");
      break;

    case ALARM:
      lockDoor();
      lockWindow();
      alarmStartTime = millis();
      lastBlinkTime  = millis();
      blinkLedState  = false;
      Serial.println("[ALARM] !! ALARM TRIGGERED !! All locks engaged.");
      break;

    case FAILSAFE:
      setLEDs(false, false, true);
      buzzerOff();
      lockDoor();
      lockWindow();
      Serial.println("[FAILSAFE] Sensor fault! All locks engaged. Press GREEN to reset.");
      break;
  }
}

//FSM ACTING
void setLEDs(bool red, bool blue, bool yellow) {
  digitalWrite(LED_RED_PIN, red);
  digitalWrite(LED_BLUE_PIN, blue);
  digitalWrite(LED_YEL_PIN, yellow);
}

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

void unlockWindow() {
  if (!isWindowLocked){
    return;
  }
  Serial.println("[Servo] Unlocking WINDOW...");
  windowServo.write(SERVO_WINDOW_UNLOCKED);
  isWindowLocked = false;
  Serial.println("[Servo] WINDOW UNLOCKED (0deg)");
}
