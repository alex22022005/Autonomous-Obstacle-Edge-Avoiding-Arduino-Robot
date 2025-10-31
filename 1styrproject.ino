#include <Servo.h>
#include <AFMotor.h>

#define IR_SENSOR A0
#define MOTOR_ENA 5
#define MOTOR_IN1 4
#define MOTOR_IN2 3

#define Echo A1
#define Trig A2
#define motor 10
#define Speed 170
#define spoint 103

char value;
int distance;
int Left;
int Right;
int L = 0;
int R = 0;

Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

#define IR_THRESHOLD 500
#define OBSTACLE_DISTANCE_THRESHOLD 5

void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(IR_SENSOR, INPUT);

  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(motor);
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
}

void loop() {
  Obstacle();
  EdgeDetection();
}

void EdgeDetection() {
  int irValue = analogRead(IR_SENSOR);

  if (irValue > IR_THRESHOLD) {
    stop();
    reverse();
    delay(1000);
    stop();
    // Implement a turn to the side based on your robot's design.
    // For example, turn left:
    left();
    delay(500);
    stop();
  } else {
    forward();
  }
}

void Obstacle() {
  distance = ultrasonic();
  if (distance < OBSTACLE_DISTANCE_THRESHOLD) {
    stop();
    delay(100);
    stop();
    L = leftsee();
    servo.write(spoint);
    delay(800);
    R = rightsee();
    servo.write(spoint);
    if (L < R) {
      right();
      delay(500);
      stop();
      delay(200);
    } else if (L > R) {
      left();
      delay(500);
      stop();
      delay(200);
    }
  } else {
    forward();
  }
}

int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 15 / 2;
  return cm;
}

void forward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

void reverse() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void stop() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}

void left() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void right() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

int rightsee() {
  servo.write(20);
  delay(800);
  Left = ultrasonic();
  return Left;
}

int leftsee() {
  servo.write(180);
  delay(800);
  Right = ultrasonic();
  return Right;
}
