//<<<<<<< Updated upstream
#include <L298N.h>
#include <MPU6050.h>
//=======
//>>>>>>> Stashed changes

#define right_motors_in1 6
#define right_motors_in2 7
#define right_motors_enable 5
#define left_motors_in1 2
#define left_motors_in2  4
#define left_motors_enable 3 
#define buzzer 8
#define RED 9
#define GREEN 10
#define BLUE 11

//<<<<<<< Updated upstream
int speed = 150;
bool beeb = true;
int last_beeb_time = 0;
float dt=0;
//mpu global variables
float Roll=0;
float Pitch=-0;
unsigned long last_time=0;
float prev_error=0;
float error_sum=0;
float error=0;
float error_derv=0;
float kp=0.5,ki=0.1,kd=0.1;
int pid_output=0;
float actual_speed=0;


MPU6050 mpu;

void get_dt();
// mpu function declariton, MPU is placed vertically so Readings of Z and X are switched
void get_roll(float AccY,float AccX,float GyroZ);
void get_pitch(float AccZ,float AccY,float AccX,float GyroY);
void get_speed(float AccZ);
void pid();
//=======
//>>>>>>> Stashed changes

void buzzer_backward_beeb(){
  float time = millis();
  if(time - millis() >= 425){
    beeb = !beeb;
    last_beeb_time = millis();
  }
  digitalWrite(buzzer, beeb ? HIGH : LOW);
}

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
  buzzer_backward_beeb();
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

void get_dt(){
  dt=(millis()-last_time)/1000;
  last_time=millis();
}

void get_roll(float AccY,float AccX,float GyroZ){
    float acc_Roll= atan2(AccY,AccX)*(180/PI);
    float gyro_Roll= GyroZ/131;
    Roll=(0.98*(Roll+gyro_Roll*dt)+ 0.02*(acc_Roll));

}
void get_pitch(float AccZ,float AccY,float AccX,float GyroY){
    float acc_Pitch= atan2(-AccZ,sqrt(AccY*AccY+AccX*AccX))*(180/PI);
    float gyro_Pitch= GyroY/131;
    Pitch=(0.98*(Pitch+gyro_Pitch*dt)+ 0.02*(acc_Pitch));

}
void get_speed(float AccZ){
  actual_speed+=abs(AccZ*dt);
  
}
void pid(){
  error=speed-actual_speed;
  if(dt!=0){
      error_sum+=(error*dt);
      error_derv= (error-prev_error)/dt;
  }
  prev_error=error;
  pid_output= (kp*error)+(ki*error_sum)+(kd*error_derv);
}

void setup(){
  pinMode(right_motors_in1, OUTPUT);
  pinMode(right_motors_in2, OUTPUT);
  pinMode(right_motors_enable, OUTPUT);
  pinMode(left_motors_in1, OUTPUT);
  pinMode(left_motors_in2, OUTPUT);
  pinMode(left_motors_enable, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);
  pinMode(BLUE,OUTPUT);
  stop_all_motors();
  mpu.initialize();
  last_time=millis();
}

void stability_check(){
  if(abs(Roll)<5 && abs(Pitch)<5){
    digitalWrite(GREEN,HIGH);
    digitalWrite(RED,LOW);
    digitalWrite(BLUE,LOW);
  }
  else if((abs(Roll)>=5 && abs(Roll)<15)||(abs(Pitch)>=5 && abs(Pitch)<15)){
    digitalWrite(GREEN,HIGH);
    digitalWrite(RED,HIGH);
    digitalWrite(BLUE,LOW);
  }
  else if(abs(Roll)>=15||abs(Pitch)>=15){
    digitalWrite(GREEN,LOW);
    digitalWrite(RED,HIGH);
    digitalWrite(BLUE,LOW);
  }
}

void loop(){
  get_dt(); //automatically updates last time as well
  int16_t ax,ay,az,gx,gy,gz;// MPU IS ALIGNED WRONG. considering anything X to be Z and vice versa. so AX actually holds AZ
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  get_roll(ay,ax,gz);
  get_pitch(az,ay,ax,gy);
  get_speed(az);
  stability_check();
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