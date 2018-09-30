/***************************************************************
 Motor driver definitions

 Add a "#elif defined" block to this file to include support
 for a particular motor driver.  Then add the appropriate
 #define near the top of the main ROSArduinoBridge.ino file.

 *************************************************************/

#include "motor_driver.h"

#ifdef L298N_DUAL_HBRIDGE
int STBY = 8; //使能端

//Motor A
int PWMA = 9;//左电机PWM输出控制脚
int PWMB = 10;//右电机PWM输出控制脚

boolean directionLeft = false;
boolean directionRight = false;

boolean direction(int i) {
	if(i == LEFT) {
		return directionLeft;
	} else {
		return directionRight;
	}
}
void initMotorController() {
	// set all the motor control pins to outputs
	pinMode(STBY, OUTPUT);

	pinMode(PWMA, OUTPUT);
	pinMode(PWMB, OUTPUT);
	pinMode(Right_motor_back, OUTPUT);
	pinMode(Right_motor_go, OUTPUT);
	pinMode(Left_motor_go, OUTPUT);
	pinMode(Left_motor_back, OUTPUT);
}

void setMotorSpeed(int i, int spd) {
	digitalWrite(STBY, HIGH);

	if(spd>MAX_PWM) {
		spd=MAX_PWM;
	}
	if(spd<-MAX_PWM) {
		spd=-1*MAX_PWM;
	}
	boolean Pin1 = HIGH;
	boolean Pin2 = LOW;
	if(spd < 0) {
		spd = -spd;
		Pin1 = LOW;
		Pin2 = HIGH;
	}
	if (i == RIGHT) {
		digitalWrite(Right_motor_go, Pin1);
		digitalWrite(Right_motor_back, Pin2);
		analogWrite(PWMB, spd);
		/*
		 if(spd != 0){
		 Serial.print("i = ");
		 Serial.print(i);
		 Serial.print(" ;spd = ");
		 Serial.println(spd);
		 Serial.print ("righ:");
		 Serial.println(spd);
		 }
		 */
	}
	else {
		digitalWrite(Left_motor_go, Pin1);
		digitalWrite(Left_motor_back, Pin2);
		analogWrite(PWMA, spd);
		/*
		 if(spd != 0) {
		 Serial.print("i = ");
		 Serial.print(i);
		 Serial.print(" ;spd = ");
		 Serial.println(spd);
		 Serial.print("left:");
		 Serial.println (spd);
		 }
		 */
	}
}

/*
 void runset(int motor, int speed, int direction){

 digitalWrite(STBY, HIGH); //使能驱动模块脚

 boolean Pin1 = LOW;
 boolean Pin2 = HIGH;

 if(direction == 1){
 Pin1 = HIGH;
 Pin2 = LOW;
 }

 if(motor == 1){
 digitalWrite(AIN1, Pin1);
 digitalWrite(AIN2, Pin2);
 analogWrite(PWMA, speed);
 }else{
 digitalWrite(BIN1, Pin1);
 digitalWrite(BIN2, Pin2);
 analogWrite(PWMB, speed);
 }
 }
 */

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
	setMotorSpeed(LEFT, leftSpeed);
	setMotorSpeed(RIGHT, rightSpeed);
}

void stop(){
  digitalWrite(STBY, LOW);
}

#else
#error A motor driver must be selected!
#endif

