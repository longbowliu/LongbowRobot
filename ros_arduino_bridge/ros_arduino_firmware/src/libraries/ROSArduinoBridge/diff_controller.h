

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID;
SetPointInfo rightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

int left_Kp=Kp;
int left_Kd=Kd;
int left_Ki=Ki;
int left_Ko=Ko;

int right_Kp=Kp;
int right_Kd=Kd;
int right_Ki=Ki;
int right_Ko=Ko;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void dorightID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;
  /*
  Serial.print("P");
  Serial.print(Perror);
   Serial.print("i");
  Serial.print(input);
  Serial.print("rKp");
  Serial.print(right_Kp);
   Serial.print("rKd");
  Serial.print(right_Kd);
  Serial.print("rKo");
  Serial.print(right_Ko);
   Serial.print("rKi");
  Serial.println(right_Ki);
  */
  
//  Serial.print("output");
  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (right_Kp * Perror - right_Kd * (input - p->PrevInput) + p->ITerm) / right_Ko;
  //Serial.println(output);
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
 // Serial.print("right = ");
  //Serial.println(output);
  
  if(output>=0){
    if(output <80){
      output = 80;
    }
    else if (output >= MAX_PWM){
      output = MAX_PWM;
    }else{
       p->ITerm += right_Ki * Perror;
    }
  }
  else{
    if(output>-80){
      output = -80;
    }
    else if (output <= -MAX_PWM){
      output = -MAX_PWM;
    }else{
       p->ITerm += right_Ki * Perror;
    }
  }
  

 // Serial.print("second = ");
  //Serial.println(output);
  p->output = output;
 // Serial.print(output);
  p->PrevInput = input;
}

/* PID routine to compute the next motor commands */
void doleftPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;
  
  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror =p->TargetTicksPerFrame - input;
  /*
  Serial.print("P");
  Serial.print(Perror);
   Serial.print("i");
  Serial.print(input);
  Serial.print("lKp");
  Serial.print(left_Kp);
   Serial.print("lKd");
  Serial.print(left_Kd);
  Serial.print("lKo");
  Serial.print(left_Ko);
   Serial.print("lKi");
  Serial.println(left_Ki);
  */
//  Serial.print("output");
  
  

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (left_Kp * Perror - left_Kd * (input - p->PrevInput) + p->ITerm) / left_Ko;
//  Serial.println(output);
//  Serial.print("output=");
//  Serial.println(output);
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  
  if(output>=0){
    if(output <80){
      output = 80;
    }
    else if (output >= MAX_PWM){
      output = MAX_PWM;
    }else{
       p->ITerm += left_Ki * Perror;
    }
  }
  else{
    if(output>-80){
      output = -80;
    }
    else if (output <= -MAX_PWM){
      output = -MAX_PWM;
    }else{
       p->ITerm += left_Ki * Perror;
    }
  }
  
 
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
   

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
//  Serial.print("leftPID.Encoder = ");
//  Serial.print(leftPID.Encoder);
  
  rightPID.Encoder = readEncoder(RIGHT);
  
//  Serial.print("rightPID.Encoder = ");
//  Serial.println(rightPID.Encoder);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  doleftPID(&leftPID);//执行左马达PID
  dorightID(&rightPID);//执行右马达PID
 

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}

