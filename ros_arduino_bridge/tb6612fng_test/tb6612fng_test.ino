#include "PinChangeInt.h"
#include "commands.h"
#include "sensors.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"

#define L298N_DUAL_HBRIDGE
#define ARDUINO_MY_COUNTER
#define PinA_left 2  //中断0
#define PinA_right 4 //中断1

volatile long count_right = 0; //使用volatile lon类型是为了外部中断脉冲计数值在其他函数中使用时，确保数值有效
volatile long count_left = 0; //使用volatile lon类型是为了外部中断脉冲计数值在其他函数中使用时，确保数值有效

int preLeft = 0;
int preRight = 0;

#define BAUDRATE     57600
#define MAX_PWM        255

int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[32];
char argv2[32];

long arg1;
long arg2;
#define AUTO_STOP_INTERVAL 8000
long lastMotorCommand = AUTO_STOP_INTERVAL;

void setup() {
	initMotorController();

	initEncoders();

	Serial.begin(57600);
}

void loop() {

	while (Serial.available() > 0) {

		chr = Serial.read();
		// Terminate a command with a CR
		if (chr == 13) {
			if (arg == 1)
				argv1[index] = NULL;
			else if (arg == 2)
				argv2[index] = NULL;
			runCommand();
			resetCommand();
		}
		// Use spaces to delimit parts of the command
		else if (chr == ' ') {
			// Step through the arguments
			if (arg == 0)
				arg = 1;
			else if (arg == 1) {
				argv1[index] = NULL;
				arg = 2;
				index = 0;
			}
			continue;
		} else {
			if (arg == 0) {
				// The first arg is the single-letter command
				cmd = chr;
			} else if (arg == 1) {
				// Subsequent arguments can be more than one character
				argv1[index] = chr;
				index++;
			} else if (arg == 2) {
				argv2[index] = chr;
				index++;
			}
		}
	}

// If we are using base control, run a PID calculation at the appropriate intervals
//  #ifdef USE_BASE
	//  if (millis() > nextPID) {
	//    updatePID();
	//    nextPID += PID_INTERVAL;
	//  }
	updatePID();
	//delay(33);

	// Check to see if we have exceeded the auto-stop interval
	if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
		setMotorSpeeds(0, 0);
		moving = 0;
	}
//  #endif
	/*
	 for(int i=1;i<=18;i++){
	 runset(1, i*5, 1); //左电机全速向前转
	 runset(2, i*5, 1); //右电机全速向前转
	 delay(33); //1秒
	 if(count_left< 10000 || (count_left %5000==0)){
	 Serial.print("L: ");
	 Serial.print(count_left-preLeft);
	 Serial.print(";    R: ");
	 Serial.println(count_right-preRight);
	 preLeft = count_left;
	 preRight = count_right;
	 }
	 }
	 */
}


int runCommand() {
	int i = 0;
	char *p = argv1;
	char *str;
	int pid_args[8];
	arg1 = atoi(argv1);
	arg2 = atoi(argv2);
	/*
	 Serial.print("cmd : ");
	 Serial.println(cmd);
	 Serial.print("arg1 : ");
	 Serial.println(arg1);
	 Serial.print("arg2 : ");
	 Serial.println(arg2);
	 */
	switch (cmd) {
	case MOTOR_SPEEDS:
		/* Reset the auto stop timer */
		lastMotorCommand = millis();
		if (arg1 == 0 && arg2 == 0) {
			setMotorSpeeds(0, 0);
			resetPID();
			moving = 0;
		} else
			moving = 1;
		leftPID.TargetTicksPerFrame = arg1; 
		rightPID.TargetTicksPerFrame = arg2;
		//Serial.println("OK");
		break;

	default:
		Serial.println("Invalid Command");
		break;
	}
}

void resetCommand() {
	cmd = NULL;
	memset(argv1, 0, sizeof(argv1));
	memset(argv2, 0, sizeof(argv2));
	arg1 = 0;
	arg2 = 0;
	arg = 0;
	index = 0;
}
