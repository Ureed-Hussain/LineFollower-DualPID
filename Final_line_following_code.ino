// ====== SENSOR PINS ======
#define SENSOR_LEFT2   A4   // far left
#define SENSOR_LEFT1   A3
#define SENSOR_CENTER  A2
#define SENSOR_RIGHT1  A1
#define SENSOR_RIGHT2  A0   // far right

// ====== MOTOR DRIVER PINS ======
#define ENA 9   // PWM left
#define IN1 4   // left dir
#define IN2 5
#define ENB 10  // PWM right
#define IN3 6   // right dir
#define IN4 7

// ====== SPEED & PID PARAMS ======
int BASE_SPEED        = 65;   // 65 base when using PID control
int MAX_SPEED         = 200;  // absolute PWM limit (0-255). Keep some headroom.
int turnSpeed         = 85;
int sharpTurnSpeed    = 110;
int searchSpeed       = 110;
int parallel_speed    = 55;

float Kp = 25.0;
float Ki = 0.0;
float Kd = 15.0;

// ====== Forward PID (to keep straight without oscillation) ======
float f_Kp = 6.0;     // small proportional gain 6 best put 4 value when base_speed is 70
float f_Kd = 5.0;     // small derivative gain 5
float f_lastError = 0.0;
unsigned long f_lastTime = 0;


float integralTerm = 0.0;
float lastError    = 0.0;
unsigned long lastTime = 0;

int lostCounter = 0;
const int LOST_LIMIT = 200;  // how many loops we tolerate no line

// helper - map signed speed to direction pins
void setMotorSigned(int left, int right) {
  // left: positive -> forward, negative -> reverse
  if (left >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(left, 0, 255));
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-left, 0, 255));
  }
  // right: positive -> forward, negative -> reverse
  if (right >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, constrain(right, 0, 255));
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, constrain(-right, 0, 255));
  }
}

// convenience wrappers (use signed speeds)
void forwardMotion(int speed) { setMotorSigned(speed, speed); }
void turnLeftPivot(int speed)  { setMotorSigned(-speed, speed); } // pivot left
void turnRightPivot(int speed) { setMotorSigned(speed, -speed); } // pivot right
void softLeft(int leftSpeed, int rightSpeed)  { setMotorSigned(leftSpeed, rightSpeed); }
void softRight(int leftSpeed, int rightSpeed) { setMotorSigned(leftSpeed, rightSpeed); }
void stopMotors() { setMotorSigned(0, 0); }

void setup() {
  pinMode(SENSOR_LEFT2,  INPUT);
  pinMode(SENSOR_LEFT1,  INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT1, INPUT);
  pinMode(SENSOR_RIGHT2, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
  Serial.println("INIT MERGED PID + CONDITIONS");

  // default to forward direction (in case we use positive speeds)
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

  lastTime = millis();
}

// Main loop: PID for normal operation, but condition checks can override with pivots/searchs
void loop() {
  // ---- Read sensors ----
  int L2 = digitalRead(SENSOR_LEFT2);
  int L1 = digitalRead(SENSOR_LEFT1);
  int C  = digitalRead(SENSOR_CENTER);
  int R1 = digitalRead(SENSOR_RIGHT1);
  int R2 = digitalRead(SENSOR_RIGHT2);

  // Debug print
  Serial.print("S: "); Serial.print(L2); Serial.print(" ");
  Serial.print(L1); Serial.print(" ");
  Serial.print(C);  Serial.print(" ");
  Serial.print(R1); Serial.print(" ");
  Serial.print(R2);

  // ============================
  // FIRST: check explicit conditions 
  // These are high-priority overrides, when they match we perform the corresponding maneuver.
  // ============================
  bool forwardStable = (L1 == 0 && C == 1 && R1 == 0);
  bool allMiddleBlack = (L1 == 1 && C == 1 && R1 == 1); //all three sensor back means to deal with the parallel condition



  if (forwardStable) {
      // --- Forward PID ---
      float f_error = 0;  // ideally center = 0, no deviation
      unsigned long now = millis();
      float dt = (now - f_lastTime) / 1000.0;
      if (dt <= 0) dt = 0.001;

      float f_derivative = (f_error - f_lastError) / dt;
      float f_correction = f_Kp * f_error + f_Kd * f_derivative;

      f_lastError = f_error;
      f_lastTime = now;

      int leftSpeed  = BASE_SPEED + f_correction;
      int rightSpeed = BASE_SPEED - f_correction;

      forwardMotion(BASE_SPEED); // fallback if needed
      setMotorSigned(leftSpeed, rightSpeed);

      Serial.println(" | action=forwardPID");

      return;
  }



  if (allMiddleBlack) {
    // keep going straight with base speed
    forwardMotion(BASE_SPEED);
    Serial.println(" | action=forward");
    // update PID timing/state to avoid large derivative spikes
    lastTime = millis();
    integralTerm = 0; // optional: reduce leftover integral when stable
    lastError = 0;
    // delay(5);
    return;
  }

  if (L2 == 1 && L1 == 0 && C == 0 && R1 == 1 && R2 == 1) {
    // keep going straight with base speed
    forwardMotion(BASE_SPEED);
    Serial.println(" | action=forward");
    // update PID timing/state to avoid large derivative spikes
    lastTime = millis();
    integralTerm = 0; // optional: reduce leftover integral when stable
    lastError = 0;
    // delay(5);
    return;
  }


// infinity

  if (L2 == 0 && L1 == 1 && C == 0 && R1 == 1 && R2 == 0) {
    // lost line to left → pivot left (search)
    turnRightPivot(searchSpeed);
    Serial.println(" | action=searchRightPivot");
    delay(20);
    return;
  }

  if (L2 == 0 && L1 == 0 && C == 0 && R1 == 1 && R2 == 0) {
    // lost line to left → pivot left (search)
    turnRightPivot(searchSpeed);
    Serial.println(" | action=searchRightPivot");
    delay(20);
    return;
  }


  if (L2 == 1 && L1 == 0 && C == 0 && R1 == 1 && R2 == 0) {
    // lost line to left → pivot left (search)
    turnRightPivot(searchSpeed);
    Serial.println(" | action=searchRightPivot");
    delay(20);
    return;
  }




  // some of the discrete pattern cases you used previously:
  if ((L1 == 1 && C == 1 && R1 == 0) || (L1 == 1 && C == 0 && R1 == 0)) {
    // small left deviation — soft turn left by reducing left motor (or minor pivot)
    // We'll use PID normally below; this pattern is lower priority — fall through to PID.
  }
  if ((R1 == 1 && C == 1 && L1 == 0) || (R1 == 1 && C == 0 && L1 == 0)) {
    // small right deviation — handled by PID below
  }

  // Hard "lost to left" pivot/search from previous code:
  if (L2 == 1 && L1 == 0 && C == 0 && R1 == 0 && R2 == 0) {
    // lost line to left → pivot left (search)
    turnLeftPivot(searchSpeed);
    Serial.println(" | action=searchLeftPivot");
    delay(20);
    return;
  }
  // other left-side patterns (kept behavior but converted to pivot/turns)    
  if (L2 == 1 && L1 == 0 && C == 0 && R1 == 0 && R2 == 1) {
    // ambiguous — do a gentle right correction
    turnRightPivot(turnSpeed);
    Serial.println(" | action=ambig_left -> rightPivot");
    delay(5);
    return;
  }



// circle ....................

  // other left-side patterns (kept behavior but converted to pivot/turns)    
  if (L2 == 1 && L1 == 1 && C == 0 && R1 == 0 && R2 == 0) {
    // ambiguous — do a gentle right correction
    turnLeftPivot(turnSpeed);
    Serial.println(" | action=ambig_left -> rightPivot");
    delay(5);
    return;
  }

 // dimond condtion

  // Hard "lost to right" pivot/search:
  if (R2 == 1 && R1 == 0 && C == 0 && L1 == 0 && L2 == 0) {
    searchRight();
    // searchRight implemented below as right pivot
    Serial.println(" | action=searchRight");
    delay(5);
    return;
  }
  if (R2 == 1 && R1 == 0 && C == 0 && L1 == 1 && L2 == 0) {
    turnRightPivot(turnSpeed);
    Serial.println(" | action=rightPattern -> rightPivot");
    delay(5);
    return;
  }

  // If no explicit override matched, use PID-based continuous control
  // ---- Weighted sensors -> position ----
  int s[5];
  s[0] = L2; s[1] = L1; s[2] = C; s[3] = R1; s[4] = R2;
  int weights[5] = { -2, -1, 0, 1, 2 };

  int sumValues = 0;
  int weightedSum = 0;
  for (int i = 0; i < 5; i++) {
    if (s[i] == 1) {
      sumValues += 1;
      weightedSum += weights[i];
    }
  }

  bool lineFound = (sumValues > 0);
  float error;
  if (lineFound) {
    float position = (float)weightedSum / (float)sumValues;
    // -2 .. 0 .. +2
    error = position;
    lostCounter = 0;
  } else {
    // no sensor sees the line → keep last error and increment lost counter
    error = lastError;
    lostCounter++;
  }

  // ---- PID ----
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  float P = error;
  integralTerm += error * dt;
  // anti-windup
  if (integralTerm > 100) integralTerm = 100;
  if (integralTerm < -100) integralTerm = -100;

  float derivative = (error - lastError) / dt;
  float correction = Kp * P + Ki * integralTerm + Kd * derivative;

  lastError = error;
  lastTime = now;

  // convert correction to signed motor speeds
  // error > 0 (line to right) -> need to steer right -> left > right (correction positive)
  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;

  // constrain magnitudes
  leftSpeed  = constrain(leftSpeed,  -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // If line is really lost, do search/pivot toward last seen direction
  if (!lineFound && lostCounter > LOST_LIMIT) {
    if (lastError > 0) { // last seen on right -> pivot right (turnRightPivot)
      turnRightPivot(searchSpeed);
      Serial.println(" | action=lost -> pivotRight");
      delay(5);
      return;
    } else {
      turnLeftPivot(searchSpeed);
      Serial.println(" | action=lost -> pivotLeft");
      delay(5);
      return;
    }
  }

  // set motors with signed speeds (allow small negative values for reverse if PID demands)
  setMotorSigned(leftSpeed, rightSpeed);

  // ---- Debug ----
  Serial.print(" | err="); Serial.print(error);
  Serial.print(" | L="); Serial.print(leftSpeed);
  Serial.print(" R="); Serial.println(rightSpeed);

  delay(5); // smooth loop
}

// small helper to match previous naming (searchRight used earlier)
void searchRight() {
  // pivot right at searchSpeed
  turnRightPivot(searchSpeed);
}
