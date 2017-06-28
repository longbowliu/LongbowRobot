#ifndef SERVOS_H
#define SERVOS_H


#define N_SERVOS 9

// This delay in milliseconds determines the pause 
// between each one degree step the servo travels.  Increasing 
// this number will make the servo sweep more slowly.  
// Decreasing this number will make the servo sweep more quickly.
// Zero is the default number and will make the servos spin at
// full speed.  150 ms makes them spin very slowly.
int stepDelay [N_SERVOS] = { 15,15,15,15,15,15 ,15,15,15}; // ms

// Pins
//byte servoPins [N_SERVOS] = {8,11,12,13,14,15};
byte servoPins [N_SERVOS] = {8,11,12,13,14,15,48,50,52};

// Initial Position
byte servoInitPosition [N_SERVOS] = { 90, 90,120,160,1,90,90 ,90 ,90 }; // [0, 180] degrees


class SweepServo
{
  public:
    SweepServo();
    void initServo(
        int servoPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    Servo getServo();

  private:
    Servo servo;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};

SweepServo servos [N_SERVOS];

#endif
