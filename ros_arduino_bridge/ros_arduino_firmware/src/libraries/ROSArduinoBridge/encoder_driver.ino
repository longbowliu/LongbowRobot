/* *************************************************************
   Encoder definitions

   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   ************************************************************ */

#ifdef USE_BASE

#ifdef ARDUINO_MY_COUNTER

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
unsigned long time1 = 0, time2 = 0; //左1，右2 轮 时间标记
void initEncoders() {
  // attachInterrupt used pin 2, 3;
  attachInterrupt(0, leftEncoderEvent, FALLING);
  attachInterrupt(1, rightEncoderEvent, FALLING);
}

void leftEncoderEvent() {
   if ((millis() - time1) > 5){
    left_enc_pos++;
   time1 = millis();
  }
 
}

void rightEncoderEvent() {
   if ((millis() - time2) > 5){
      right_enc_pos++;
      time2 = millis();
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
#endif

