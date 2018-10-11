/* Functions and type-defs for PID control.

 Taken mostly from Mike Ferguson's ArbotiX code which lives at:

 http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
 */

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
	double ITerm;                    //integrated term

	double output;                    // last motor setting
} SetPointInfo;

SetPointInfo leftPID, rightPID;

unsigned char moving = 0; // is the base in motion?

/*
 * Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving
 * In particular, assign both Encoder and PrevEnc the current encoder value
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 * Note that the assumption here is that PID is only turned on
 * when going from stop to moving, that's why we can init everything on zero.
 */
void resetPID() {
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

/* PID Parameters 

*** it is better to keep the kd less than 0.35****

//kp= 0.01
//kd=0.02
//ki= 0.00003
//150-1500

kp= 0.006
kd= 0.006
ki= 0.00001
150 - 1800


kp= 0.001
kd= 0.003
ki= 0.000001
1800-4200


*/
// 0.03 : 60-1100;   0.01 :1200-1800   0.002 2500-3000
double Kp = 0.006;
double Kd = 0.006;
double Ki = 0.00001;
//double Ki = 0;
double Ko = 1;

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
	if (p->TargetTicksPerFrame > 1800) {
		Kp = 0.001;
		Kd = 0.003;
		Ki = 0.000001;
	}
	long Perror;
	double output;
	int input;
	//Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
	input = p->Encoder - p->PrevEnc;
	Perror = p->TargetTicksPerFrame - input;

	Serial.print("Perror_L:");
	Serial.print(Perror);
	Serial.print(" input_L:");
	Serial.print(input);
	output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
	p->PrevEnc = p->Encoder;
	output += p->output;
	p->ITerm += Ki * Perror;
	Serial.print(" Kd:");
	Serial.print(Kd * (input - p->PrevInput));
	Serial.print(" p->ITerm:");
	Serial.print(p->ITerm);
	Serial.print(" output_L:");
	Serial.println(output);


	//if(p->TargetTicksPerFrame <0 ){
	//	Perror = p->TargetTicksPerFrame + input;
	//}
/*
	output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
	p->PrevEnc = p->Encoder;
	output += p->output;
	p->ITerm += Ki * Perror;
*/
	p->output = output;
	p->PrevInput = input;

}

/* Read the encoder values and call the PID routine */
void updatePID() {
	/* Read the encoders */
	leftPID.Encoder = readEncoder(LEFT);
	rightPID.Encoder = readEncoder(RIGHT);

	/* If we're not moving there is nothing more to do */
	if (!moving) {
		/*
		 * Reset PIDs once, to prevent startup spikes,
		 * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
		 * PrevInput is considered a good proxy to detect
		 * whether reset has already happened
		 */
		if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0)
			resetPID();
		return;
	}

	/* Compute PID update for each motor */
	doPID(&rightPID);
	doPID(&leftPID);
/*
        Serial.print("leftPID.output : ");
        Serial.print(leftPID.output);
        Serial.print(" & ");
        Serial.print((int)(leftPID.output+0.5));
        Serial.print("rightPID.output : ");
        Serial.print(rightPID.output);
        Serial.print(" & ");
        Serial.println((int)(rightPID.output+0.5));
        */
	/* Set the motor speeds accordingly */
	//setMotorSpeeds(leftPID.output, rightPID.output);

    setMotorSpeeds((int)(leftPID.output+0.5), (int)(rightPID.output+0.5));
//        delay(33);
}

