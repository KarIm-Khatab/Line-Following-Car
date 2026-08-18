#include <L298N.h>

#define right_motors_in1 8
#define right_motors_in2 7
#define right_motors_enable A0
#define left_motors_in1 2
#define left_motors_in2 4 
#define left_motors_enable A1

int speed = 10;

void stop_all_motors(){
  digitalWrite(right_motors_in1, LOW);
  digitalWrite(right_motors_in2, LOW);
  analogWrite(right_motors_enable, 0);
  digitalWrite(left_motors_in1, LOW);
  digitalWrite(left_motors_in2, LOW);
  analogWrite(left_motors_enable, 0);
}

void move_forward(){
  stop_all_motors();
  digitalWrite(right_motors_in1, HIGH);
  digitalWrite(right_motors_in2, LOW);
  analogWrite(right_motors_enable, speed);
  digitalWrite(left_motors_in1, HIGH);
  digitalWrite(left_motors_in2, LOW);
  analogWrite(left_motors_enable, speed);
}

void move_backward(){
  stop_all_motors();
  digitalWrite(right_motors_in1, LOW);
  digitalWrite(right_motors_in2, HIGH);
  analogWrite(right_motors_enable, speed);
  digitalWrite(left_motors_in1, LOW);
  digitalWrite(left_motors_in2, HIGH);
  analogWrite(left_motors_enable, speed);
}

void move_left(){
  stop_all_motors();
  digitalWrite(right_motors_in1, HIGH);
  digitalWrite(right_motors_in2, LOW);
  analogWrite(right_motors_enable, speed);
}

void move_right(){
  stop_all_motors();
  digitalWrite(left_motors_in1, HIGH);
  digitalWrite(left_motors_in2, LOW);
  analogWrite(left_motors_enable, speed);
}

void setup(){
  pinMode(right_motors_in1, OUTPUT);
  pinMode(right_motors_in2, OUTPUT);
  pinMode(right_motors_enable, OUTPUT);
  pinMode(left_motors_in1, OUTPUT);
  pinMode(left_motors_in2, OUTPUT);
  pinMode(left_motors_enable, OUTPUT);
  stop_all_motors();
}

void loop(){
  stop_all_motors();
  delay(1000);
  move_forward();
  delay(3000);
  stop_all_motors();
  delay(1000);
  move_backward();
  delay(3000);
  stop_all_motors();
  delay(1000);
  move_right();
  delay(3000);
  stop_all_motors();
  delay(1000);
  move_left();
  delay(3000);
  stop_all_motors();
  delay(1000);
}