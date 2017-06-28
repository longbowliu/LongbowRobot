#include <Servo.h>
//  **************************** static parameters for wheels *************************************
int Left_motor_back = 7;   //左电机后退(IN1)
int Left_motor_go = 6;   //左电机前进(IN2)
int Right_motor_go = 5;  // 右电机前进(IN3)
int Right_motor_back = 4;  // 右电机后退(IN4)
double zhouchang = 21.5;
int mapan = 20;
// speed per second
double diSuPerSecond = 14; // cm/second   // 14 ,  14.5 ,15, 12
double zhongSuPerSecond = 34.5; // cm/second  35
double gaoSuPerSecond = 60; //cm/second
double suduBackPerSecond = 63; // 42, 45, 55, 58, 62
// pulse for wheels

int diSu_L = 96; // 96,99,90 , 95
int diSu_R = 84;  // 85
int zhongSu_L = 168;
int zhongSu_R = 150;
int gaoSu_L = 245;  // 235 , 245
int gaoSu_R = 225;
int back_sudu_L = 128;
int back_sudu_R = 234;

// time
double turn90_time = 13.5;
// *********************************** static parameters for servos ***********************************
Servo zz14; // clow
Servo zj13; // left shoulder
Servo yj12;  // right shoulder
Servo dp11;   // base plate
Servo neck1_15; // neck near shoulder  vetical / horizontal
Servo neck2_8;  // neck near clow up/down

Servo ytsA7;
Servo ytxA8;
Servo headA9;

// limit angles of servos
int dp11_left_max = 180;
int dp11_frond_max = 90;
int dp11_right_max = 1;
int yj12_backward_max = 40;
int yj12_forward_max = 180;
int zj13_backward_max = 160;
int zj13_forward_max = 20;
int zz14_open_max = 110;
int zz14_close_max = 1;
int neck_ping = 95;
int neck_zhi = 0;
// angles for original point
int yd_yj12 = 120;
int yd_zj13 = 160;
int yd_dp11 = 90;
int yd_zz14 = 1;
// highest point
int high_yj12 = 1;
int high_zj13 = 120;
// speed of servo , it is used for delay time
int servo_speed = 15;

unsigned int motor1 = 0;	 //计左电机码盘脉冲值
unsigned int motor2 = 0;	 //计右电机码盘脉冲值

const int SensorRight_2 = 16;   	//右红外传感器(P3.5 OUT4)
const int SensorLeft_2 = 17;     //左红外传感器(P3.4 OUT3)

int Echo = A1;  // Echo回声脚(P2.0)
int Trig =A0;  //  Trig 触发脚(P2.1)
int Distance = 0;

void setup()
{
  Serial.begin(9600);
 // attachInterrupt(0, left_motor, FALLING);
  //attachInterrupt(1, right_motor, FALLING);
  pinMode(2, INPUT);
  pinMode(3, INPUT);


  pinMode(Left_motor_go, OUTPUT); // PIN 8 (PWM)
  pinMode(Left_motor_back, OUTPUT); // PIN 9 (PWM)
  pinMode(Right_motor_go, OUTPUT); // PIN 10 (PWM)
  pinMode(Right_motor_back, OUTPUT); // PIN 11 (PWM)

  pinMode(SensorRight_2, INPUT); //定义右红外传感器为输入
  pinMode(SensorLeft_2, INPUT); //定义左红外传感器为输入
  
  pinMode(Echo, INPUT);    // 定义超声波输入脚
  pinMode(Trig, OUTPUT);   // 定义超声波输出脚

  // yuandian();
  //releaseAllServos();
}

float Distance_test()   // 量出前方距离 
{
  digitalWrite(Trig, LOW);   // 给触发脚低电平2μs
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // 给触发脚高电平10μs，这里至少是10μs
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);    // 持续给触发脚低电
  float Fdistance = pulseIn(Echo, HIGH);  // 读取高电平时间(单位：微秒)
  Fdistance= Fdistance/58;       //为什么除以58等于厘米，  Y米=（X秒*344）/2
  // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
  //Serial.print("Distance:");      //输出距离（单位：厘米）
  //Serial.println(Fdistance);         //显示距离
  //Distance = Fdistance;
  return Fdistance;
}   

void rotate(Servo t , int startAngle, int stopAngle, int speed , int direction ) {
  int angle;
  Serial.print("from ");
  Serial.print(startAngle);
  Serial.print(" to ");
  Serial.print(stopAngle);
  Serial.print(" direction ");
  Serial.println(direction);
  for (angle = startAngle; direction == 1 ? angle<stopAngle : angle>stopAngle; direction == 1 ? angle++ : angle--) {
    t.write(angle);
    delay(speed);
  }
}


void loop()
{
  delay(500);
  //Serial.print("left motor:");
  //    Serial.println(motor1);
  //    Serial.print("\tright motor:");
  //    Serial.println(motor2);


  String comdata = "";
  int numdata[6] = {0},  mark = 0;
  int j = 0;
  while (Serial.available() > 0)
  {
    comdata += char(Serial.read());
    delay(2);
    mark = 1;
  }
  if (mark == 1)
  {
    for (int i = 0; i < comdata.length() ; i++)
    {
      if (comdata[i] == ',')
      {
        j++;
      }
      else
      {
        if (comdata[i] != ' ') {
          numdata[j] = numdata[j] * 10 + (comdata[i] - '0');
        }
      }
    }
    int servoNum = numdata[0];
    int startAng = numdata[1];
    int stopAng = numdata[2];
    //int derc = numdata[3];
    Servo t ;
    if (servoNum <90 ) {
      t.attach(servoNum);
      rotate(t, startAng, stopAng, servo_speed, 1);
      rotate(t, stopAng, startAng, servo_speed, 0);
      t.detach();
    }else{
      if (servoNum == 97) {
        t.attach(A7);
      }
      if (servoNum == 98) {
        t.attach(A8);
      }
      if (servoNum == 99) {
        t.attach(A9);
      }
      rotate(t, startAng, stopAng, servo_speed, 1);
      rotate(t, stopAng, startAng, servo_speed, 0);
      t.detach();
    }
      
 /*
    if (servoNum == 0) {
     
     
      boolean noDistance=true;
      while(noDistance){
         Distance = Distance_test();
         //Serial.println(Distance);
         if( (2<Distance)&(Distance<400) ){
           Serial.println(Distance);
           noDistance = false;
         }
      }
      
      
      
      int SR_2 = digitalRead(SensorRight_2);
      int SL_2 = digitalRead(SensorLeft_2);
      Serial.println(SR_2);
      Serial.println(SL_2);
      */
      // Test all 9 servos start************************
    
      /*
       t.attach(14);
       Serial.println(14);
       rotate(t,startAng,stopAng,20,1);
       rotate(t,stopAng,startAng,20,0);
       t.detach();
       t.attach(15);
       Serial.println(15);
       rotate(t,startAng,stopAng,20,1);
       rotate(t,stopAng,startAng,20,0);
       t.detach();
        // Test all 9 servos end************************
       
       //closeClaw();
       motor1=0;
      motor2=0;
      goWithSpeedDistance(1,50);
     
        t.attach(14);
       Serial.println(14);
       rotate(t,startAng,stopAng,20,1);
       rotate(t,stopAng,startAng,20,0);
       t.detach();
       t.attach(15);
       Serial.println(15);
       rotate(t,startAng,stopAng,20,1);
       rotate(t,stopAng,startAng,20,0);
       t.detach();
      

      yuandian();
      openClaw();
      closeClaw();
      yd2Lowest();
      lowest2Highest();
      front2Left();
      highest2Lowest();
      backWithSpeedDistance(2,20);
      
     

      openClaw();
      lowest2YD();
      closeClaw();
      left2Front();
      yd2Highest();
      front2Right();
      releaseAllServos();
      
    }
    if (servoNum == 4) {
      dp11.attach(4);
      rotate(dp11, startAng, stopAng, servo_speed, 1);
      rotate(dp11, stopAng, startAng, servo_speed, 0);
      dp11.detach();
    }
    if (servoNum == 5) {
      zj13.detach();
      yj12.attach(5);
      rotate(yj12, startAng, stopAng, servo_speed, 1);
      rotate(yj12, stopAng, startAng, servo_speed, 0);
      yj12.detach();
    }
    if (servoNum == 13) {
      yj12.detach();
      zj13.attach(13);
      rotate(zj13, startAng, stopAng, servo_speed, 1);
      rotate(zj13, stopAng, startAng, servo_speed, 0);
      zj13.detach();
    }
    if (servoNum == 7) {
      zz14.attach(7);
      rotate(zz14, startAng, stopAng, servo_speed, 1);
      rotate(zz14, stopAng, startAng, servo_speed, 0);
      zz14.detach();
    }
    
    if (servoNum == 9) {
      goWithSpeedDistance(1, 100);
      brake(5);
      goWithSpeedDistance(2, 100);
      brake(5);
      //yd2Lowest();
      goWithSpeedDistance(3, 100);
      brake(5);
      backWithSpeedDistance(2, 200);
    }
     */
    // reset to null for next time use.
    comdata = String("");
    for (int i = 0; i < 6; i++)
    {
      numdata[i] = 0;
    }
    mark = 0;
  }

}

void left_motor()            //触发函数
{
  Serial.print("motor1 = ");
   Serial.println(motor1);
  motor1++;
//  if (motor1 >= 99)
//    motor1 = 0;
}

void right_motor()            //触发函数
{
  Serial.print("motor2 = ");
   Serial.println(motor2);
  motor2++;
//  if (motor2 >= 99)
//    motor2 = 0;
}

void closeClaw() {
  zz14.attach(7);
  rotate(zz14, zz14_open_max, yd_zz14, servo_speed, 0);
}
void openClaw() {
  zz14.attach(7);
  rotate(zz14, yd_zz14, zz14_open_max, servo_speed, 1);
}

void yd2Lowest() {
  zj13.attach(6);
  rotate(zj13, yd_zj13, zj13_forward_max, servo_speed, 0);
  yj12.attach(5);
  rotate(yj12, yd_yj12, yj12_forward_max, servo_speed, 1);
}
void yd2Highest() {
  zj13.attach(6);
  rotate(zj13, yd_zj13, high_zj13, servo_speed, 0);
  yj12.attach(5);
  rotate(yj12, yd_yj12, high_yj12, servo_speed, 1);
}
void lowest2Highest() {
  yj12.attach(5);
  zj13.attach(6);
  rotate(yj12, yj12_forward_max, yd_yj12, servo_speed, 0);
  rotate(zj13, zj13_forward_max, high_zj13, servo_speed, 1);
  rotate(yj12, yd_yj12, high_yj12, servo_speed, 0);
}
void lowest2YD() {
  yj12.attach(5);
  zj13.attach(6);
  rotate(yj12, yj12_forward_max, yd_yj12, servo_speed, 0);
  rotate(zj13, zj13_forward_max, yd_zj13, servo_speed, 1);
}

void highest2Lowest() {
  yj12.attach(5);
  zj13.attach(6);
  rotate(yj12, high_yj12, yd_yj12, servo_speed, 1);
  rotate(zj13, high_zj13, zj13_forward_max, servo_speed, 0);
  rotate(yj12, yd_yj12, yj12_forward_max, servo_speed, 1);
}
void front2Left() {
  dp11.attach(4);
  Serial.println(dp11_frond_max);
  Serial.println(dp11_left_max);
  rotate(dp11, dp11_frond_max, dp11_left_max, servo_speed, 1);
}
void left2Front() {
  dp11.attach(4);
  rotate(dp11, dp11_left_max, dp11_frond_max, servo_speed, 0);
}
void front2Right() {
  dp11.attach(4);
  rotate(dp11, dp11_frond_max, dp11_right_max, servo_speed, 0);
}
void right2Front() {
  dp11.attach(4);
  rotate(dp11, dp11_frond_max, dp11_right_max, servo_speed, 1);
}

void releaseAllServos() {
  dp11.detach();
  yj12.detach();
  zj13.detach();
  zz14.detach();
}


void yuandian() {
  zz14.attach(7);
  zz14.write(yd_zz14);
  delay(500);
  zz14.detach();
  yj12.attach(5);
  yj12.write(high_yj12);
  delay(800);
  yj12.detach();
  zj13.attach(6);
  zj13.write(high_zj13);
  delay(800);
  zj13.detach();
  dp11.attach(4);
  dp11.write(yd_dp11);
  delay(800);
  dp11.detach();
  yj12.attach(5);
  yj12.write(yd_yj12);
  delay(800);
  yj12.detach();
  zj13.attach(6);
  zj13.write(yd_zj13);
  delay(800);
  zj13.detach();
}

void qianZQ() {
  dp11.write(150);
  delay(200);
  zj13.write(100);
  delay(200);
  zz14.write(120);
  delay(200);
  yj12.write(80);
  delay(200);
  zj13.write(10);
  delay(1000);
  zz14.write(50);
}

void brake(int time)         //刹车，停车
{
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  delay(time * 100);//执行时间，可以调整
}

void diSuRun(double time) {
  run(time, diSu_L, diSu_R);
}
void zhongSuRun(double time) {
  run(time, zhongSu_L, zhongSu_R);
}
void gaoSuRun(double time) {
  run(time, gaoSu_L, gaoSu_R);
}

void run(double time , int sudu_L, int sudu_R)     // 前进
{
  //val_right=(float)val_right+(rpm1-rpm2)*0.4
  Serial.println("hi");
  digitalWrite(Right_motor_go, HIGH); // 右电机前进
  digitalWrite(Right_motor_back, LOW);
  digitalWrite(Left_motor_go, HIGH); // 左电机前进
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, sudu_L); //PWM比例0~255调速，左右轮差异略增减
  analogWrite(Right_motor_go, sudu_R); //PWM比例0~255调速，左右轮差异略增减
  analogWrite(Right_motor_back, 0);
  analogWrite(Left_motor_back, 0);
  delay(time * 100);   //执行时间，可以调整
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Right_motor_go, LOW);

}

void goWithSpeedDistance(int speed, int distance ) {
  double t = 0;
  if (speed == 1) {
    t = distance / diSuPerSecond * 10;
    diSuRun(t);
  }
  if (speed == 2) {
    t = distance / zhongSuPerSecond * 10;
    zhongSuRun(t);
  }
  if (speed == 3) {
    t = distance / gaoSuPerSecond * 10;
    gaoSuRun(t);
  }
}

// just has zhongSuBack for my Car
void zhongSuBack(double time) {
  back(time, back_sudu_L, back_sudu_R);
}

void backWithSpeedDistance(int speed, int distance ) {
  double t = 0;
  if (speed == 2) {
    t = distance / suduBackPerSecond * 10;
    zhongSuBack(t);
  }
}


void back(double time, int sudu_L, int sudu_R)        //后退
{
  digitalWrite(Right_motor_go, LOW); //右轮后退
  digitalWrite(Right_motor_back, HIGH);
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, sudu_R); //PWM比例0~255调速
  digitalWrite(Left_motor_go, LOW); //左轮后退
  digitalWrite(Left_motor_back, HIGH);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, sudu_L); //PWM比例0~255调速
  delay(time * 100);     //执行时间，可以调整
  digitalWrite(Right_motor_back, LOW);
  digitalWrite(Left_motor_back, LOW);
}

void left(double time)         //左转(左轮不动，右轮前进)
{
  if (time > turn90_time) {
    time = turn90_time;
  }
  if (time < 1) {
    time = 1;
  }
  digitalWrite(Right_motor_go, HIGH);	// 右电机前进
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, diSu_L);
  analogWrite(Right_motor_back, 0); //PWM比例0~255调速
  digitalWrite(Left_motor_go, LOW);  //左轮不动
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 0); //PWM比例0~255调速
  delay(time * 100);	//执行时间，可以调整
  digitalWrite(Right_motor_go, LOW);
}

void right(double time)        //右转(右轮不动，左轮前进)
{
  if (time > turn90_time) {
    time = turn90_time;
  }
  if (time < 1) {
    time = 1;
  }
  digitalWrite(Right_motor_go, LOW);  //右电机不动
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 0); //PWM比例0~255调速
  digitalWrite(Left_motor_go, HIGH); //左电机前进
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, diSu_L);
  analogWrite(Left_motor_back, 0); //PWM比例0~255调速
  delay(time * 100);	//执行时间，可以调整
  digitalWrite(Left_motor_go, LOW);
}




