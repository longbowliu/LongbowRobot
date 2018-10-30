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
	boolean last_derc ;

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
	leftPID.last_derc = 0;

	rightPID.TargetTicksPerFrame = 0.0;
	rightPID.Encoder = readEncoder(RIGHT);
	rightPID.PrevEnc = rightPID.Encoder;
	rightPID.output = 0;
	rightPID.PrevInput = 0;
	rightPID.ITerm = 0;
	rightPID.last_derc = 0;

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
/*
 double Kp = 0.1;
 double Ki = 0.001;
 double Kd = 0.001;
 */

double Ko = 1;
int pos_value[4] = { 5, 3, 4, 3 };
/* PID routine to compute the next motor commands */
void doLeftPID(SetPointInfo * p) {
	if (p->TargetTicksPerFrame > 1800 or p->TargetTicksPerFrame < -1800) {
		Kp = 0.001;
		Kd = 0.003;
		Ki = 0.000001;
	}
	boolean derc_chnaged = false;
	if( (p->TargetTicksPerFrame <0 && lEFT_LAST_DERECTION) || (p->TargetTicksPerFrame > 0 && !lEFT_LAST_DERECTION) ){
		derc_chnaged = true;
	}
	if(p->TargetTicksPerFrame>=0){
		lEFT_LAST_DERECTION =1;
	}
	else {
		lEFT_LAST_DERECTION =0;
	}


	long Perror;
	double output;
	int input;
	//Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
	int dw = int(p->TargetTicksPerFrame) / 50;
	if(p->TargetTicksPerFrame<0){
		dw = int(-p->TargetTicksPerFrame) / 50;
	}
	boolean first = false;
	int group = dw / 4;
	int position = (dw) % 4;
	int p_v = 0;
	for (int i = 0; i < position; i++) {
		p_v += pos_value[i];
	}
	int comp = 4 + group * 15 + p_v;

	if(group==0 && position==0){
		//comp = comp+5;  //empty
		comp = comp+11;    // loader
	}
	if(group==0 && position==1){
		comp = comp+7; //loader
	}
/*
	Serial.print("p_v:");
	Serial.print(p_v);
	Serial.print("; goup:");
	Serial.print(group);
	Serial.print("; position:");
	Serial.print(position);
	Serial.print("; comp:");
	Serial.print(comp);
	Serial.print("; dw:");
	Serial.println(dw);
*/
    //Serial.println(p->output < comp*0.7);

	//if (p->output < comp*0.7 || p->output > comp*1.3 || derc_chnaged) {
        if (p->output < comp*0.7 || derc_chnaged) {
		output = comp;
		first = true;
	} else {
		first = false;
	}

	if (first) {
                // Perror ,PreInput need change for the second.
		if(p->TargetTicksPerFrame<0){
			input = -p->Encoder - p->PrevEnc;
			Perror = -p->TargetTicksPerFrame ;
			p->PrevEnc = -p->Encoder;
		}else{
			input = p->Encoder - p->PrevEnc;
			Perror = p->TargetTicksPerFrame ;
			p->PrevEnc = p->Encoder;
		}
		//output = 10;
		//p->PrevEnc = 0;
		p->ITerm += Ki * Perror;
		p->output = output;
		p->PrevInput = 0;
/*
		Serial.print("****Perror_L:");
		Serial.print(Perror);
		Serial.print(" input_L:");
		Serial.print(input);
		Serial.print(" Kd:");
		Serial.print(Kd * (input - p->PrevInput));
		Serial.print(" p->ITerm:");
		Serial.print(p->ITerm);
		Serial.print(" output_L:");
		Serial.println(p->TargetTicksPerFrame<0?int(-output-0.5):int(output + 0.5));
		*/

	} else {

		if(p->TargetTicksPerFrame<0){
			input = -p->Encoder - p->PrevEnc;
			Perror = -p->TargetTicksPerFrame - input;
			p->PrevEnc = -p->Encoder;
		}else{
			input = p->Encoder - p->PrevEnc;
			Perror = p->TargetTicksPerFrame - input;
			p->PrevEnc = p->Encoder;
		}
/*
		Serial.print("Perror_L:");
		Serial.print(Perror);
		Serial.print(" input_L:");
		Serial.print(p->TargetTicksPerFrame<0?-input:input);
		output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
		output += p->output;
		p->ITerm += Ki * Perror;
		Serial.print(" Kd:");
		Serial.print(Kd * (input - p->PrevInput));
		Serial.print(" p->ITerm:");
		Serial.print(p->ITerm);
		Serial.print(" output_L:");
		Serial.println(p->TargetTicksPerFrame<0?int(-output-0.5):int(output + 0.5));

*/
		 output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
		 output += p->output;
		 p->ITerm += Ki * Perror;


		p->output = output;
		p->PrevInput = input;
	}

}

void doRightPID(SetPointInfo * p) {
	if (p->TargetTicksPerFrame > 1800 or p->TargetTicksPerFrame < -1800) {
			Kp = 0.001;
			Kd = 0.003;
			Ki = 0.000001;
		}
		boolean derc_chnaged = false;
		if( (p->TargetTicksPerFrame <0 && RIGHT_LAST_DERECTION) || (p->TargetTicksPerFrame > 0 && !RIGHT_LAST_DERECTION) ){
			derc_chnaged = true;
		}
		if(p->TargetTicksPerFrame>=0){
			RIGHT_LAST_DERECTION =1;
		}
		else {
			RIGHT_LAST_DERECTION =0;
		}


		long Perror;
		double output;
		int input;
		//Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
		int dw = int(p->TargetTicksPerFrame) / 50;
		if(p->TargetTicksPerFrame<0){
			dw = int(-p->TargetTicksPerFrame) / 50;
		}
		boolean first = false;
		int group = dw / 4;
		int position = (dw) % 4;
		int p_v = 0;
		for (int i = 0; i < position; i++) {
			p_v += pos_value[i];
		}
		int comp = 4 + group * 15 + p_v;

		if(group==0 && position==0){
			//comp = comp+5;  //empty
			comp = comp+11;    // loader
		}
		if(group==0 && position==1){
			comp = comp+7; //loader
		}
	/*
		Serial.print("p_v:");
		Serial.print(p_v);
		Serial.print("; goup:");
		Serial.print(group);
		Serial.print("; position:");
		Serial.print(position);
		Serial.print("; comp:");
		Serial.print(comp);
		Serial.print("; dw:");
		Serial.println(dw);
	*/
	    //Serial.println(p->output < comp*0.7);

		//if (p->output < comp*0.7 || p->output > comp*1.3 || derc_chnaged) {
                if (p->output < comp*0.7 || derc_chnaged) {
			output = comp;
			first = true;
		} else {
			first = false;
		}

		if (first) {
	                // Perror ,PreInput need change for the second.
			if(p->TargetTicksPerFrame<0){
				input = -p->Encoder - p->PrevEnc;
				Perror = -p->TargetTicksPerFrame ;
				p->PrevEnc = -p->Encoder;
			}else{
				input = p->Encoder - p->PrevEnc;
				Perror = p->TargetTicksPerFrame ;
				p->PrevEnc = p->Encoder;
			}
			//output = 10;
			//p->PrevEnc = 0;
			p->ITerm += Ki * Perror;
			p->output = output;
			p->PrevInput = 0;
/*
			Serial.print("****Perror_R:");
			Serial.print(Perror);
			Serial.print(" input_R:");
			Serial.print(input);
			Serial.print(" Kd:");
			Serial.print(Kd * (input - p->PrevInput));
			Serial.print(" p->ITerm:");
			Serial.print(p->ITerm);
			Serial.print(" output_R:");
			Serial.println(p->TargetTicksPerFrame<0?int(-output-0.5):int(output + 0.5));
*/
		} else {

			if(p->TargetTicksPerFrame<0){
				input = -p->Encoder - p->PrevEnc;
				Perror = -p->TargetTicksPerFrame - input;
				p->PrevEnc = -p->Encoder;
			}else{
				input = p->Encoder - p->PrevEnc;
				Perror = p->TargetTicksPerFrame - input;
				p->PrevEnc = p->Encoder;
			}
/*
			Serial.print("Perror_R:");
			Serial.print(Perror);
			Serial.print(" input_R:");
			Serial.print(p->TargetTicksPerFrame<0?-input:input);
			output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
			output += p->output;
			p->ITerm += Ki * Perror;
			Serial.print(" Kd:");
			Serial.print(Kd * (input - p->PrevInput));
			Serial.print(" p->ITerm:");
			Serial.print(p->ITerm);
			Serial.print(" output_R:");
			Serial.println(p->TargetTicksPerFrame<0?int(-output-0.5):int(output + 0.5));

	*/
			 output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
			 output += p->output;
			 p->ITerm += Ki * Perror;


			p->output = output;
			p->PrevInput = input;
		}

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
	doLeftPID(&leftPID);
	doRightPID(&rightPID);
//	doLeftPID(&rightPID);
    int left_speed = int(leftPID.output+0.5);
	if(leftPID.TargetTicksPerFrame<0){
		left_speed = int(-leftPID.output-0.5);
	}
	int right_speed = int(rightPID.output+0.5);
	if(rightPID.TargetTicksPerFrame<0){
		right_speed = int(-rightPID.output-0.5);
	}
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
	setMotorSpeeds(left_speed , right_speed);
//        delay(33);
}

