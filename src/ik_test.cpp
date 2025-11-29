#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// --- ROBOT DIMENSIONS ---
const float L1 = 90.0;   // Base Height
const float L2 = 105.0;  // Shoulder Link
const float L3 = 145.0;  // Elbow Link
const float L4 = 170.0;  // Gripper Length

// --- CALIBRATED OFFSETS ---
const int OFFSET_BASE     = 5;
const int OFFSET_SHOULDER = 10;
const int OFFSET_ELBOW    = 0;
const int OFFSET_WRIST    = -10;

// --- PINS ---
const int PIN_BASE     = 3;
const int PIN_SHOULDER = 5;
const int PIN_ELBOW    = 6;
const int PIN_WRIST    = 10;
const int PIN_GRIPPER  = 8;

Servo sBase, sShoulder, sElbow, sWrist, sGripper;

// Global variables to store current positions
float curBase = 90, curShoulder = 90, curElbow = 90, curWrist = 90;

// Function prototypes
void smoothMoveAll(float targetShoulder, float targetElbow, float targetWrist);
void BaseMove(float targetBase);
void calculateIK(float r, float theta, float z, float &t1, float &t2, float &t3);

void setup() {
  Serial.begin(9600);
  sBase.attach(PIN_BASE);
  sShoulder.attach(PIN_SHOULDER);
  sElbow.attach(PIN_ELBOW);
  sWrist.attach(PIN_WRIST);
  sGripper.attach(PIN_GRIPPER);

  // Initialize Home
  BaseMove(90);
  smoothMoveAll(90, 90, 90);
  
  Serial.println("=== IK TEST MODE ===");
  Serial.println();
  
  // Test IK with your values
  Serial.println("=== Testing IK with R=275, Z=53.36 ===");
  float test_r = 275.0;
  float test_theta = 90.0;
  float test_z = 53.36;
  float test_t1, test_t2, test_t3;
  
  calculateIK(test_r, test_theta, test_z, test_t1, test_t2, test_t3);
  
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  T - Test with R=275, Z=53.36");
  Serial.println("  M R,Theta,Z - Manual input (e.g., M 275,90,53.36)");
  Serial.println("  H - Home position");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();
    
    if (input.length() == 0) return;
    
    char command = input.charAt(0);
    
    if (command == 'T') {
      // Test with specified values
      Serial.println("\n=== Testing R=275, Z=53.36 ===");
      float t1, t2, t3;
      calculateIK(275.0, 90.0, 53.36, t1, t2, t3);
      
      Serial.print("\nMoving to calculated position...");
      BaseMove(90.0);
      smoothMoveAll(t1, t2, t3);
      Serial.println("Done!");
      
    } else if (command == 'M') {
      // Manual input: M R,Theta,Z
      int comma1 = input.indexOf(',');
      int comma2 = input.indexOf(',', comma1 + 1);
      
      if (comma1 > 0 && comma2 > 0) {
        float r = input.substring(2, comma1).toFloat();
        float theta = input.substring(comma1 + 1, comma2).toFloat();
        float z = input.substring(comma2 + 1).toFloat();
        
        Serial.print("\n=== Testing R="); Serial.print(r);
        Serial.print(", Theta="); Serial.print(theta);
        Serial.print(", Z="); Serial.print(z); Serial.println(" ===");
        
        float t1, t2, t3;
        calculateIK(r, theta, z, t1, t2, t3);
        
        Serial.print("\nMoving to calculated position...");
        BaseMove(theta);
        smoothMoveAll(t1, t2, t3);
        Serial.println("Done!");
      } else {
        Serial.println("Format: M R,Theta,Z (e.g., M 275,90,53.36)");
      }
      
    } else if (command == 'H') {
      // Home position
      Serial.println("\nReturning to home position...");
      BaseMove(90);
      smoothMoveAll(90, 90, 90);
      Serial.println("Home position reached!");
    }
  }
}

// Smooth movement for Base only
void BaseMove(float targetBase) {
  int steps = 20; // Number of small steps
  int stepDelay = 50; // Delay between steps in ms
  
  float diffBase = (targetBase - curBase) / steps;
  
  for (int i = 1; i <= steps; i++) {
    float tempBase = curBase + (diffBase * i);
    sBase.write(constrain(tempBase + OFFSET_BASE, 0, 180));
    delay(stepDelay);
  }
  
  // Ensure final position is exact
  sBase.write(constrain(targetBase + OFFSET_BASE, 0, 180));
  curBase = targetBase;
}

// Smooth movement for Shoulder, Elbow, and Wrist (no Base)
void smoothMoveAll(float targetShoulder, float targetElbow, float targetWrist) {
  int steps = 20; // Number of small steps (Higher = Smoother/Slower)
  int stepDelay = 50; // Delay between steps in ms
  
  float diffShoulder = (targetShoulder - curShoulder) / steps;
  float diffElbow = (targetElbow - curElbow) / steps;
  float diffWrist = (targetWrist - curWrist) / steps;
  
  for (int i = 1; i <= steps; i++) {
    // Calculate intermediate positions
    float tempShoulder = curShoulder + (diffShoulder * i);
    float tempElbow = curElbow + (diffElbow * i);
    float tempWrist = curWrist + (diffWrist * i);

    // Write to servos simultaneously
    sShoulder.write(constrain(tempShoulder + OFFSET_SHOULDER, 0, 180));
    sElbow.write(constrain(tempElbow + OFFSET_ELBOW, 0, 180));
    sWrist.write(constrain(tempWrist + OFFSET_WRIST, 0, 180));
    
    delay(stepDelay); // Wait between steps
  }
  
  // Ensure final position is exact
  sShoulder.write(constrain(targetShoulder + OFFSET_SHOULDER, 0, 180));
  sElbow.write(constrain(targetElbow + OFFSET_ELBOW, 0, 180));
  sWrist.write(constrain(targetWrist + OFFSET_WRIST, 0, 180));

  // Update global current positions
  curShoulder = targetShoulder;
  curElbow = targetElbow;
  curWrist = targetWrist;
}

// Inverse Kinematics: Calculate joint angles from cylindrical coordinates
// Input: r (radial distance), theta (base angle in degrees), z (height)
// Output: t1 (shoulder angle), t2 (elbow angle), t3 (wrist angle)
void calculateIK(float r, float theta, float z, float &t1, float &t2, float &t3) {
  float a = r - L4; // Adjust for gripper length
  float d = L1 - z; // Adjust for base height
  float b = L3; // Elbow link length
  float c = L2; // Shoulder link length
  
  // Calculate K using the formula: K = (a² + b² + d² - c²) / 2
  float K = (a*a + b*b + d*d - c*c) / 2.0;
  
  // Calculate Y = arcsin(K / b*√(a² + d²)) - arctan(d/a)
  float sqrt_term = sqrt(a*a + d*d);
  float arcsin_term = asin(K / (b * sqrt_term));
  float arctan_term = atan2(d, a);
  
  float Alfa = arcsin_term - arctan_term; // Y in radians
  
  // Calculate X = atan2(b*cos(Y) - d, a - b*sin(Y))
  // This gives us t1 (shoulder angle)
  float X = atan2(b * cos(Alfa) - d, a - b * sin(Alfa));
  t1 = X * (180.0 / PI); // Convert to degrees
  
  // Calculate elbow angle (Alfa converted to degrees)
  Alfa = Alfa * (180.0 / PI);
  t2 = 90 + t1 - Alfa;
  
  // Calculate wrist angle to keep gripper horizontal
  t3 = Alfa;
  
  // Debug output
  Serial.println("--- IK Calculation Details ---");
  Serial.print("Input: r="); Serial.print(r);
  Serial.print(" theta="); Serial.print(theta);
  Serial.print(" z="); Serial.println(z);
  Serial.println();
  Serial.print("Intermediate values:");
  Serial.print("  a="); Serial.print(a);
  Serial.print("  d="); Serial.print(d);
  Serial.print("  K="); Serial.println(K);
  Serial.println();
  Serial.println("Output angles:");
  Serial.print("  Alfa(Y) = "); Serial.print(Alfa); Serial.println("°");
  Serial.print("  t1(X)   = "); Serial.print(t1); Serial.println("° (Shoulder)");
  Serial.print("  t2      = "); Serial.print(t2); Serial.println("° (Elbow)");
  Serial.print("  t3      = "); Serial.print(t3); Serial.println("° (Wrist)");
  Serial.println("------------------------------");
}
