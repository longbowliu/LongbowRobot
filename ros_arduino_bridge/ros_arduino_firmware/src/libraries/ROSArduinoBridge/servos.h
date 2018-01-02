#ifndef SERVOS_H
#define SERVOS_H


#define N_SERVOS 6

// This delay in milliseconds determines the pause 
// between each one degree step the servo travels.  Increasing 
// this number will make the servo sweep more slowly.  
// Decreasing this number will make the servo sweep more quickly.
// Zero is the default number and will make the servos spin at
// full speed.  150 ms makes them spin very slowly.
int stepDelay [N_SERVOS] = { 15,15,15,15,15,15 }; // ms

// Pins

//byte servoPins [N_SERVOS] = {11,13,12,15,8,14,50,52,53};
byte servoPins [N_SERVOS] = {2,3,4,5,6,7};


// !!!!!!!!!!!!!!!!!!!frond --> behind && left--> right!!!!!!!!!!!!!!!!!!!!

// 11 base plate 0，60，180
// 13 left shoulder 0 ，130 ，130
// 12 right shoulder 0 100 ，125 
// 15 neck near shoulder  vetical / horizontal 5 ，5，130
// 8 neck near clow up/down 0, 0, 180
// 14 clow 1 ,1,110
// 50 souna  left right 0, 80, 180
// 52 yuntai left right 0 ,90 ,180
// 53,yuntai up down 0,40,150


// Initial Position
byte servoInitPosition [N_SERVOS] = {90,120,90,90,90,60}; // [0, 180] degrees


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

