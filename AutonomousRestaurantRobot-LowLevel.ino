#include <PID_v1.h>

// 1-MOTOR PINS 

#define PWM_LEFT
#define DIR_LEFT

#define PWM_RIGHT
#define DIR_RIGHT


// 2-UART COMMUNICATION

#define PI_LINK Serial2      
#define RXD2 16
#define TXD2 17
#define BAUD_RATE 115200
#define COMM_TIMEOUT 500

#define MSG_SIZE 24         
char msgBuffer[MSG_SIZE];
byte msgIndex = 0;

unsigned long lastMessageTime = 0;
bool communicationActive = false;


// 3-PID VARIABLES

double Setpoint = 0.0;
double Input = 0.0;
double Output = 0.0;

double Kp = 100.0;
double Ki = 0.0;
double Kd = 0.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// 4-RECEIVE AND UNDERSTAND MESSAGES

// understanding messages:
void handleMessage(char* msg) {

    // ---- E:<number>  line error ----
    if (msg[0] == 'E' && msg[1] == ':') {

        char* end;
        double value = strtod(msg + 2, &end);



        if (end == msg + 2 || *end != '\0') return;

        Input = constrain(value, -1.0, 1.0);   // error is a fraction

        lastMessageTime = millis();
        communicationActive = true;

        Serial.print("Received Error: ");
        Serial.println(Input);
    }

    // ---- S  stop ----
    else if (strcmp(msg, "S") == 0) {
        communicationActive = false;
        stopMotors();
    }

    // for future cases like turning
}

// receive messages:
void receiveFromPi() {

    while (PI_LINK.available()) {

        char c = PI_LINK.read();

        if (c == '\n') {
            msgBuffer[msgIndex] = '\0';          
            if (msgIndex > 0) handleMessage(msgBuffer);
            msgIndex = 0;                        
        }

        else if (c != '\r') {
            if (msgIndex < MSG_SIZE - 1) {
                msgBuffer[msgIndex++] = c;
            } else {
                msgIndex = 0;                    
            }
        }
    }
}


// 5-MOTOR CONTROL

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


// 6-MOVEMENT FUNCTIONS

void moveForward(int speed)  { setLeftMotor(speed);  setRightMotor(speed);  }
void moveBackward(int speed) { setLeftMotor(-speed); setRightMotor(-speed); }
void turnLeft(int speed)     { setLeftMotor(-speed); setRightMotor(speed);  }
void turnRight(int speed)    { setLeftMotor(speed);  setRightMotor(-speed); }
void stopMotors()            { setLeftMotor(0);      setRightMotor(0);      }


// 7-PID MOTOR CONTROL

void PIDControl(int base, double pidOutput) {

    int leftSpeed  = constrain(base + pidOutput, 0, 255);
    int rightSpeed = constrain(base - pidOutput, 0, 255);

    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
}


// 8-SETUP

void setup() {

    Serial.begin(115200);

    pinMode(PWM_LEFT, OUTPUT);
    pinMode(DIR_LEFT, OUTPUT);
    pinMode(PWM_RIGHT, OUTPUT);
    pinMode(DIR_RIGHT, OUTPUT);

    Serial2.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);

    myPID.SetOutputLimits(-100, 100);
    myPID.SetSampleTime(50);
    myPID.SetMode(AUTOMATIC);

    stopMotors();

    Serial.println("Robot controller ready!");
}


// 9-MAIN LOOP

void loop() {

    receiveFromPi();

    if (communicationActive &&
        millis() - lastMessageTime < COMM_TIMEOUT) {

        if (myPID.Compute()) {
            PIDControl(150, Output);
        }
    }
    else {
        communicationActive = false;
        stopMotors();
    }
}
