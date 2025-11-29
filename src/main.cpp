#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// Easing profiles
float getSCurveProgress(float t) { t = constrain(t, 0.0, 1.0); return 3.0 * t * t - 2.0 * t * t * t; }
float getSmoothstepProgress(float t) { t = constrain(t, 0.0, 1.0); return t * t * (3.0 - 2.0 * t); }

// Robot dimensions (mm)
const float L1 = 90.0;   // Base height
const float L2 = 105.0;  // Shoulder link
const float L3 = 145.0;  // Elbow link
const float L4 = 150.0;  // Gripper length

// Calibrated offsets
const int OFFSET_BASE     = 5;
const int OFFSET_SHOULDER = 10;
const int OFFSET_ELBOW    = 0;
const int OFFSET_WRIST    = -10;

// Gripper
const int GRIPPER_OPEN_ANGLE = 80;
const int GRIPPER_CLOSED_ANGLE = 100;

// Pins
const int PIN_BASE     = 3;
const int PIN_SHOULDER = 5;
const int PIN_ELBOW    = 6;
const int PIN_WRIST    = 10;
const int PIN_GRIPPER  = 9;

// Debug switches
const bool DEBUG_IK = false;

Servo sBase, sShoulder, sElbow, sWrist, sGripper;

// Current joint states (deg)
float curBase = 90, curShoulder = 120, curElbow = 90, curWrist = 120;

// Prototypes
void smoothMoveAll(float targetShoulder, float targetElbow, float targetWrist, int duration_ms = 1500);
void BaseMove(float targetBase, int duration_ms = 800);
void calculateIK(float r, float theta, float z, float &t1, float &t2, float &t3);
void moveToPosition(float r, float theta, float z);
void pickAndPlace(float pickR, float pickTheta, float pickZ, float placeR, float placeTheta, float placeZ);

void setup() {
  Serial.begin(9600);
  sBase.attach(PIN_BASE);
  sShoulder.attach(PIN_SHOULDER);
  sElbow.attach(PIN_ELBOW);
  sWrist.attach(PIN_WRIST);
  sGripper.attach(PIN_GRIPPER);

  // Home
  sGripper.write(GRIPPER_OPEN_ANGLE);
  delay(300);
  BaseMove(90);
  smoothMoveAll(90, 90, 90);

  Serial.println("=== ROBOT ARM CONTROL ===");
  Serial.println("B - Base rotation test");
  Serial.println("A - Arm movement test");
  Serial.println("P - Pick & Place 1");
  Serial.println("2 - Pick & Place 2");
  Serial.println("H - Home");
  Serial.println("M - Menu");
}

void loop() {
  if (Serial.available() == 0) return;
  String input = Serial.readStringUntil('\n');
  input.trim(); input.toUpperCase();
  if (input.length() == 0) return;
  char command = input.charAt(0);

  if (command == 'B') {
    Serial.println("Base test");
    BaseMove(0, 900); delay(300);
    BaseMove(120, 900); delay(300);
    BaseMove(180, 900); delay(300);

  } else if (command == 'A') {
    Serial.println("Arm test");
    smoothMoveAll(120, 90, 120, 1200); delay(300);
    smoothMoveAll(30, 75, 45, 1200);

  } else if (command == '1') {
    Serial.println("Pick & Place 1");
    pickAndPlace(300, 120, 50, 350, 45, 50);

  } else if (command == '2') {
    Serial.println("Pick & Place 2");
    pickAndPlace(320, 60, 50, 280, 120, 50);

  } else if (command == 'H') {
    Serial.println("Home");
    BaseMove(90);
    smoothMoveAll(90, 90, 90);
    sGripper.write(GRIPPER_OPEN_ANGLE);

  } else if (command == 'M') {
    Serial.println("=== ROBOT ARM CONTROL ===");
    Serial.println("B - Base rotation test");
    Serial.println("A - Arm movement test");
    Serial.println("1 - Pick & Place 1");
    Serial.println("2 - Pick & Place 2");
    Serial.println("H - Home");
    Serial.println("M - Menu");

  } else {
    Serial.println("Unknown command. Press M for menu.");
  }
}

// Base movement with S-curve
void BaseMove(float targetBase, int duration_ms) {
  int totalSteps = 80;
  int stepDelay = max(5, duration_ms / totalSteps);
  float diffBase = targetBase - curBase;
  for (int i = 0; i <= totalSteps; i++) {
    float eased = getSCurveProgress((float)i / totalSteps);
    float tempBase = curBase + (diffBase * eased);
    sBase.write(constrain(tempBase + OFFSET_BASE, 0, 180));
    delay(stepDelay);
  }
  sBase.write(constrain(targetBase + OFFSET_BASE, 0, 180));
  curBase = targetBase;
}

// Shoulder/Elbow/Wrist movement with smoothstep
void smoothMoveAll(float targetShoulder, float targetElbow, float targetWrist, int duration_ms) {
  int totalSteps = 100;
  int stepDelay = max(5, duration_ms / totalSteps);
  float diffShoulder = targetShoulder - curShoulder;
  float diffElbow = targetElbow - curElbow;
  float diffWrist = targetWrist - curWrist;
  for (int i = 0; i <= totalSteps; i++) {
    float eased = getSmoothstepProgress((float)i / totalSteps);
    float tempShoulder = curShoulder + (diffShoulder * eased);
    float tempElbow = curElbow + (diffElbow * eased);
    float tempWrist = curWrist + (diffWrist * eased);
    sShoulder.write(constrain(tempShoulder + OFFSET_SHOULDER, 0, 180));
    sElbow.write(constrain(tempElbow + OFFSET_ELBOW, 0, 180));
    sWrist.write(constrain(tempWrist + OFFSET_WRIST, 0, 180));
    delay(stepDelay);
  }
  sShoulder.write(constrain(targetShoulder + OFFSET_SHOULDER, 0, 180));
  sElbow.write(constrain(targetElbow + OFFSET_ELBOW, 0, 180));
  sWrist.write(constrain(targetWrist + OFFSET_WRIST, 0, 180));
  curShoulder = targetShoulder; curElbow = targetElbow; curWrist = targetWrist;
}

// IK path planning with waypoints along R from 240 to target R
void moveToPosition(float r, float theta, float z) {
  const int NUM_WAYPOINTS = 3;
  float waypointAngles[NUM_WAYPOINTS][3];
  float startR = 240.0;
  float stepR = (r - startR) / (NUM_WAYPOINTS - 1);

  for (int i = 0; i < NUM_WAYPOINTS; i++) {
    float currentR = startR + (stepR * i);
    float t1, t2, t3;
    calculateIK(currentR, theta, z, t1, t2, t3);
    waypointAngles[i][0] = t1;
    waypointAngles[i][1] = t2;
    waypointAngles[i][2] = t3;
  }

  BaseMove(theta, 1000);
  delay(200);

  // Move to first waypoint
  smoothMoveAll(waypointAngles[0][0], waypointAngles[0][1], waypointAngles[0][2], 1200);
  delay(150);

  // Sweep through waypoints
  int totalSteps = 150;
  int stepDelay = 15;
  float targetShoulder = waypointAngles[NUM_WAYPOINTS - 1][0];
  float targetElbow = waypointAngles[NUM_WAYPOINTS - 1][1];
  float targetWrist = waypointAngles[NUM_WAYPOINTS - 1][2];

  for (int step = 0; step <= totalSteps; step++) {
    float eased = getSmoothstepProgress((float)step / totalSteps);
    float waypointProgress = eased * (NUM_WAYPOINTS - 1);
    int idx = (int)waypointProgress;
    float localProgress = waypointProgress - idx;
    if (idx >= NUM_WAYPOINTS - 1) { idx = NUM_WAYPOINTS - 2; localProgress = 1.0; }

    float shoulder = waypointAngles[idx][0] + (waypointAngles[idx + 1][0] - waypointAngles[idx][0]) * localProgress;
    float elbow    = waypointAngles[idx][1] + (waypointAngles[idx + 1][1] - waypointAngles[idx][1]) * localProgress;
    float wrist    = waypointAngles[idx][2] + (waypointAngles[idx + 1][2] - waypointAngles[idx][2]) * localProgress;

    sShoulder.write(constrain(shoulder + OFFSET_SHOULDER, 0, 180));
    sElbow.write(constrain(elbow + OFFSET_ELBOW, 0, 180));
    sWrist.write(constrain(wrist + OFFSET_WRIST, 0, 180));
    delay(stepDelay);
  }

  sShoulder.write(constrain(targetShoulder + OFFSET_SHOULDER, 0, 180));
  sElbow.write(constrain(targetElbow + OFFSET_ELBOW, 0, 180));
  sWrist.write(constrain(targetWrist + OFFSET_WRIST, 0, 180));
  curShoulder = targetShoulder; curElbow = targetElbow; curWrist = targetWrist;
}

// Pick & Place sequence (with safe poses)
void pickAndPlace(float pickR, float pickTheta, float pickZ, float placeR, float placeTheta, float placeZ) {
  // Open gripper
  sGripper.write(GRIPPER_OPEN_ANGLE); delay(300);

  // Approach pick
  moveToPosition(pickR, pickTheta, pickZ);

  // Pick
  sGripper.write(GRIPPER_CLOSED_ANGLE); delay(600);

  // Lift up Z+50
  float t1_lift, t2_lift, t3_lift;
  calculateIK(pickR, pickTheta, pickZ + 50, t1_lift, t2_lift, t3_lift);
  smoothMoveAll(t1_lift, t2_lift, t3_lift, 900); delay(200);

  // Move to safe joint pose before base turn
  smoothMoveAll(120, 90, 120, 1200); delay(300);

  // Approach place from above (Z+50)
  moveToPosition(placeR, placeTheta, placeZ + 50);

  // Vertical down to place
  float t1_place, t2_place, t3_place;
  calculateIK(placeR, placeTheta, placeZ, t1_place, t2_place, t3_place);
  smoothMoveAll(t1_place, t2_place, t3_place, 900); delay(200);

  // Release
  sGripper.write(GRIPPER_OPEN_ANGLE); delay(600);

  // Retract horizontally to (240, theta, Z)
  float t1_ret, t2_ret, t3_ret;
  calculateIK(240.0, placeTheta, placeZ, t1_ret, t2_ret, t3_ret);
  smoothMoveAll(t1_ret, t2_ret, t3_ret, 1200); delay(200);

  // Lift to safe pose, then base to 90
  smoothMoveAll(120, 90, 120, 1200); delay(200);
  BaseMove(90, 900); delay(200);
}

// Inverse kinematics
void calculateIK(float r, float theta, float z, float &t1, float &t2, float &t3) {
  float a = r - L4; // planar reach correction
  float d = L1 - z; // vertical offset
  float b = L3;
  float c = L2;

  float K = (a*a + b*b + d*d - c*c) / 2.0;
  float sqrt_term = sqrt(a*a + d*d);
  float Alfa = asin(K / (b * sqrt_term)) - atan2(d, a); // radians

  float X = atan2(b * cos(Alfa) - d, a - b * sin(Alfa));
  t1 = X * (180.0 / PI);

  Alfa = Alfa * (180.0 / PI);
  t2 = 90 + t1 - Alfa;
  t3 = Alfa;

  if (DEBUG_IK) {
    Serial.println("--- IK ---");
    Serial.print("r="); Serial.print(r);
    Serial.print(" th="); Serial.print(theta);
    Serial.print(" z="); Serial.println(z);
    Serial.print("t1="); Serial.print(t1);
    Serial.print(" t2="); Serial.print(t2);
    Serial.print(" t3="); Serial.println(t3);
  }
}
