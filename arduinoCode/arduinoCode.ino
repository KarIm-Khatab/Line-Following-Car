#include <L298N.h>
#include <MPU6050.h>

#define right_motors_in1 8
#define right_motors_in2 7
#define right_motors_enable A0
#define left_motors_in1 2
#define left_motors_in2 4 
#define left_motors_enable A1

int speed = 10;
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
// mpu function declariton, 
void get_roll(float AccY,float AccZ,float GyroX);
void get_pitch(float AccX,float AccY,float AccZ,float GyroY);
void get_speed(float AccX);
void pid();

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
  mpu.initialize();
  last_time=millis();
}

void loop(){
  get_dt(); //automatically updates last time as well
  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  get_roll(ay,az,gx);
  get_pitch(ax,ay,az,gy);
  get_speed(ax);
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


void get_dt(){
  dt=(millis()-last_time)/1000;
  last_time=millis();
}

void get_roll(float AccY,float AccZ,float GyroX){
    float acc_Roll= atan2(AccY,AccZ)*(180/PI);
    float gyro_Roll= GyroX/131;
    Roll=0.98*(Roll+gyro_Roll*dt)+ 0.02*(acc_Roll);

}
void get_pitch(float AccX,float AccY,float AccZ,float GyroY){
    float acc_Pitch= atan2(-AccX,sqrt(AccY*AccY+AccZ*AccZ))*(180/PI);
    float gyro_Pitch= GyroY/131;
    Pitch=0.98*(Pitch+gyro_Pitch*dt)+ 0.02*(acc_Pitch);

}
void get_speed(float AccX){
  actual_speed+=AccX*dt;
  
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