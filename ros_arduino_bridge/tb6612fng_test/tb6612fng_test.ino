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

#define BAUDRATE     19200
#define MAX_PWM        255

int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[32];
char argv2[32];

long arg1;
long arg2;

#define PID_RATE           30
/* Convert the rate into an interval */
int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;

void setup() {
	Serial.begin(19200);

	initMotorController();

	initEncoders();

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

	if (millis() > nextPID) {
		updatePID();
		nextPID += PID_INTERVAL;
	}

	// Check to see if we have exceeded the auto-stop interval
	if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
		setMotorSpeeds(0, 0);
		moving = 0;
	}
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
	case GET_BAUDRATE:
		Serial.println(BAUDRATE);
		break;
	case READ_ENCODERS:
		Serial.print(readEncoder(LEFT));
		Serial.print(" ");
		Serial.println(readEncoder(RIGHT));
		break;
	case RESET_ENCODERS:
		resetEncoders();
		resetPID();
		Serial.println("OK");
		break;
	case MOTOR_SPEEDS:
		/* Reset the auto stop timer */
		lastMotorCommand = millis();
		if (arg1 == 0 && arg2 == 0) {
			stop();
			resetPID();
			moving = 0;
		} else
			moving = 1;
		leftPID.TargetTicksPerFrame = arg1;
		rightPID.TargetTicksPerFrame = arg2;


		Serial.println("OK");
		break;
	case UPDATE_PID:
		while ((str = strtok_r(p, ":", &p)) != '\0') {
			pid_args[i] = atoi(str);
			i++;
		}

//		Kp = pid_args[0];
//		Kd = pid_args[1];
//		Ki = pid_args[2];
//		Ko = pid_args[3];

		Serial.println("OK");
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
