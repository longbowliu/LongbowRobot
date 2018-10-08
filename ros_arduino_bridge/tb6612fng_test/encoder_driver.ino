/* *************************************************************
   Encoder definitions

   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   ************************************************************ */
#include "./PinChangeInt.h"


#ifdef ARDUINO_MY_COUNTER

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;


//unsigned long time1 = 0, time2 = 0; //左1，右2 轮 时间标记
void initEncoders() {
  // attachInterrupt used pin 2, 3;
  attachInterrupt(0, leftEncoderEvent, FALLING);
  //attachInterrupt(1, rightEncoderEvent, FALLING);
  attachPinChangeInterrupt(4, rightEncoderEvent, FALLING); //PinA_right=4
}

void leftEncoderEvent() {
	if(leftPID.TargetTicksPerFrame >0 || lEFT_LAST_DERECTION ){
		left_enc_pos++;
	}else {
		left_enc_pos--;
	}
 
}

void rightEncoderEvent() {
	if(rightPID.TargetTicksPerFrame > 0 || RIGHT_LAST_DERECTION){
			right_enc_pos++;
		}else{
			right_enc_pos--;
		}

  
}

long readEncoder(int i) {
  if (i == LEFT) 
  {
    return left_enc_pos;
  }
  else 
  {
    return right_enc_pos;
  }
}

void resetEncoder(int i) {
  if (i == LEFT) {
    left_enc_pos = 0L;
    return;
  } else {
    right_enc_pos = 0L;
    return;
  }
}

void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}
#endif


