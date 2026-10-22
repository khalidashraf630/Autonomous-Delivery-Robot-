// PINS 
#define PWM_LEFT  
#define DIR_LEFT  
#define PWM_RIGHT 
#define DIR_RIGHT 



// Everything else calls this
// speed: -255 to +255
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(DIR_LEFT, HIGH);
    analogWrite(PWM_LEFT, speed);
  } else {
    digitalWrite(DIR_LEFT, LOW);
    analogWrite(PWM_LEFT, abs(speed));
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(DIR_RIGHT, HIGH);
    analogWrite(PWM_RIGHT, speed);
  } else {
    digitalWrite(DIR_RIGHT, LOW);
    analogWrite(PWM_RIGHT, abs(speed));
  }
}

// MOVEMENT FUNCTIONS 

void moveForward(int speed) {
  setLeftMotor(speed);
  setRightMotor(speed);
}

void moveBackward(int speed) {
  setLeftMotor(-speed);
  setRightMotor(-speed);
}

void turnLeft(int speed) {
  setLeftMotor(-speed);  
  setRightMotor(speed);  
}

void turnRight(int speed) {
  setLeftMotor(speed);   
  setRightMotor(-speed); 
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

// PID SPEED CONTROL 
// this is how i imagine the logic for this :)
void PIDControl(int base, float pidOutput) {
  int leftSpeed  = constrain(base + pidOutput, -255, 255);
  int rightSpeed = constrain(base - pidOutput, -255, 255);
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

// SETUP 
void setup() {
  Serial.begin(115200);

  pinMode(PWM_LEFT,  OUTPUT);
  pinMode(DIR_LEFT,  OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);

  stopMotors();
  Serial.println("Motors ready!");
}

//  TEST 
void loop() {
  moveForward(150);   delay(2000);
  stopMotors();       delay(1000);
  moveBackward(150);  delay(2000);
  stopMotors();       delay(1000);
  turnLeft(150);      delay(1000);
  stopMotors();       delay(1000);
  turnRight(150);     delay(1000);
  stopMotors();       delay(1000);
}