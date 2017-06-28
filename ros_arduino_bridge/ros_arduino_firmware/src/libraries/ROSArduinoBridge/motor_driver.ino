/***************************************************************
   Motor driver definitions

   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   *************************************************************/

#ifdef USE_BASE

#ifdef L298N_DUAL_HBRIDGE

  boolean directionLeft = false;
  boolean directionRight = false;

  boolean direction(int i){
     if(i == LEFT){
        return directionLeft;
     }else{
        return directionRight;
     }
  }
  void initMotorController() {
  // set all the motor control pins to outputs
//  pinMode(ENA, OUTPUT);
//  pinMode(ENB, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  }

  void setMotorSpeed(int i, int spd) {
    if(spd>MAX_PWM){
      spd=MAX_PWM;
    }
       if(spd<-MAX_PWM){
      spd=-1*MAX_PWM;
    }
    if (i == RIGHT){
        if(spd>=0){
            directionLeft = FORWARDS;
            digitalWrite(Right_motor_go, HIGH);
            digitalWrite(Right_motor_back, LOW);
            analogWrite(Right_motor_go, spd);
        }else if(spd < 0){
            directionLeft = BACKWARDS;
            digitalWrite(Right_motor_back, HIGH);
            digitalWrite(Right_motor_go, LOW);
            analogWrite(Right_motor_back, -spd);
        }
    }
    else {
        if(spd>=0){
            directionRight = FORWARDS;
            digitalWrite(Left_motor_go, HIGH);
            digitalWrite(Left_motor_back, LOW);
            analogWrite(Left_motor_go, spd);
        }else if(spd<0){
            directionRight = BACKWARDS;
            digitalWrite(Left_motor_back, HIGH);
            digitalWrite(Left_motor_go, LOW);
            analogWrite(Left_motor_back, -spd);
        }
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif

