//<<<<<<< Updated upstream
#include <MPU6050.h>
#include <SoftwareSerial.h>
SoftwareSerial relaySerial(2, 3);
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
#define front_left_ir A0
#define front_right_ir A1
#define back_left_ir A2
#define back_right_ir A3


//<<<<<<< Updated upstream
int speed = 90;
bool beeb = true;
int fr_ir, fl_ir, br_ir, bl_ir;
bool started = true;
int last_beeb_time = 0;
bool moving_forward = true;
const int ir_threshold = 900;
float dt=0;
//mpu global variables
float Roll=0;
float Pitch=-0;
float Yaw=0;
unsigned long last_time=0;
float prev_error=0;
float error_sum=0;
float error=0;
float error_derv=0;
float kp=0.5,ki=0.1,kd=0.1;
int pid_output=0;
float actual_speed=0;

bool embedded = true;
bool app = false;

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
  if(time - last_beeb_time >= 425){
    beeb = !beeb;
    last_beeb_time = millis();
  }
  digitalWrite(buzzer, beeb ? HIGH : LOW);
}

void read_ir(){
  fr_ir= analogRead(front_right_ir);
  fl_ir= analogRead(front_left_ir);
  bl_ir = analogRead(back_left_ir);
  br_ir = analogRead(back_right_ir) ;
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
  digitalWrite(right_motors_in1, LOW);
  digitalWrite(right_motors_in2, HIGH);
  analogWrite(right_motors_enable, speed);
  digitalWrite(left_motors_in1, LOW);
  digitalWrite(left_motors_in2, HIGH);
  analogWrite(left_motors_enable, speed);
}

void move_backward(){
  stop_all_motors();
  buzzer_backward_beeb();
  digitalWrite(right_motors_in1, HIGH);
  digitalWrite(right_motors_in2, LOW);
  analogWrite(right_motors_enable, speed);
  digitalWrite(left_motors_in1, HIGH);
  digitalWrite(left_motors_in2, LOW);
  analogWrite(left_motors_enable, speed);
}

void move_left(){
  digitalWrite(left_motors_in1, HIGH);
  digitalWrite(left_motors_in2, LOW);
  analogWrite(left_motors_enable, speed );
  digitalWrite(right_motors_in1, LOW);
  digitalWrite(right_motors_in2, HIGH);
  analogWrite(right_motors_enable, speed * 1.7);
}

void move_right(){
  digitalWrite(right_motors_in1, HIGH);
  digitalWrite(right_motors_in2, LOW);
  analogWrite(right_motors_enable, speed );
  digitalWrite(left_motors_in1, LOW);
  digitalWrite(left_motors_in2, HIGH);
  analogWrite(left_motors_enable, speed * 1.7);
}

void spin(){
  stop_all_motors();
  
  digitalWrite(right_motors_in1, HIGH);
  digitalWrite(right_motors_in2, LOW);
  analogWrite(right_motors_enable, 250);
  digitalWrite(left_motors_in1, LOW);
  digitalWrite(left_motors_in2, HIGH);
  analogWrite(left_motors_enable, 250);
}
void stopping_inertia()
{
  if(moving_forward){
    if(bl_ir>= ir_threshold && br_ir>= ir_threshold)
      stop_all_motors();
  }
}
void movement_handler(){
  if(moving_forward){
    beeb = false;
    if(fl_ir< ir_threshold && fr_ir< ir_threshold)
    {
      //stopping_inertia();
      move_forward();
    }
    else if(fl_ir< ir_threshold && fr_ir >= ir_threshold){
      move_right(); 
    }
    else if(fl_ir >= ir_threshold && fr_ir< ir_threshold){
      move_left(); 
    }
    else {
      stop_all_motors();
    }
  }
  else {
    if(analogRead(back_left_ir)>= ir_threshold && analogRead(back_right_ir)>= ir_threshold)
    {
      move_backward();
    }
    else if(analogRead(back_left_ir)>= ir_threshold && analogRead(back_right_ir) < ir_threshold){
      move_right(); 
    }
    else if(analogRead(back_left_ir) < ir_threshold && analogRead(back_right_ir)>= ir_threshold){
      move_left(); 
    }
    else {
      stop_all_motors();
    }
  }
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

void get_yaw(float GyroX){
  Yaw=GyroX/131;
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
  pinMode(back_left_ir,INPUT);
  pinMode(back_right_ir,INPUT); 
  pinMode(front_left_ir,INPUT);
  pinMode(front_right_ir,INPUT); 
  Serial.begin(9600);
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
  get_yaw(gx);
  get_speed(az);
  stability_check();
  //spin();
  read_ir();
  Serial.println(fr_ir);
  //if(started)
  //{
  //}
}



void loop() {
  if (relaySerial.available()) {
    String COMAND = relaySerial.readStringUntil('\n');  
    COMAND.trim();

 if (COMAND == "START")
    {
      embedded = false;
      app = true; 
      }
      else if (COMAND == "END")
{
  
}
if(app){
    if (COMAND == "F") {
      moveForward();    
    }
    else if (COMAND == "B") {
      moveBackward();   
    }
    else if (COMAND == "L") {
  turnLeft();
}

else if (COMAND == "R") {
  turnRight();
}
    else if (COMAND== "S") {
      stopMotors();       
    }
    else if (COMAND.startsWith("V:")) {
      int speed = COMAND.substring(2).toInt();
      setSpeed(speed);    
    }}
    if (embedded)
  {
    //call emneded functions 
    movement_handler();

  }
  }
}
