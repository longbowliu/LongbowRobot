#include <Servo.h>
int servo_speed = 15;
Servo t ;
void setup()
{
  Serial.begin(9600);

}

void loop()
{
  String comdata = "";
  int numdata[6] = {0},  mark = 0;
  int j = 0;
  while (Serial.available() > 0)
  {
    comdata += char(Serial.read());
    delay(2);
    mark = 1;
  }
  if (mark == 1)
  {
    for (int i = 0; i < comdata.length() ; i++)
    {
      if (comdata[i] == ',')
      {
        j++;
      }
      else
      {
        if (comdata[i] != ' ') {
          numdata[j] = numdata[j] * 10 + (comdata[i] - '0');
        }
      }
    }
    int servoNum = numdata[0];
    int startAng = numdata[1];
    int stopAng = numdata[2];

    int angle;
    
   t.attach(servoNum);

    int temp = t.read();
    Serial.print("current position 1: ");
    Serial.println(temp);
    if (startAng > temp) {
      for (angle = temp; angle < startAng; angle++) {
        t.write(angle);
        delay(servo_speed);
      }
    }
    if (startAng < temp) {
      for (angle = temp; angle > startAng; angle--) {

        delay(servo_speed);
      }
    }
    temp = t.read();
    Serial.print("current position 2: ");
    Serial.println(temp);

    if (servoNum < 90 ) {
      rotate(t, startAng, stopAng, servo_speed, 1);
      Serial.print("current position 3: ");
      Serial.println(t.read());
      rotate(t, stopAng, startAng, servo_speed, 0);
      Serial.print("current position 4: ");
      Serial.println(t.read());
       t.detach();
      Serial.print("current position 5: ");
      Serial.println(t.read());
    }
    // reset to null for next time use.
    comdata = String("");
    for (int i = 0; i < 6; i++)
    {
      numdata[i] = 0;
    }
    mark = 0;

  }

}

void rotate(Servo t , int startAngle, int stopAngle, int speed , int direction ) {
  int angle;
  Serial.print("  from ");
  Serial.print(startAngle);
  Serial.print(" to ");
  Serial.print(stopAngle);
  Serial.print(" direction ");
  Serial.println(direction);
  for (angle = startAngle; direction == 1 ? angle<stopAngle : angle>stopAngle; direction == 1 ? angle++ : angle--) {
    t.write(angle);
    //      Serial.print("current position : ");
    //      Serial.println(t.read());
    delay(speed);
  }
}


