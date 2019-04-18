#ifndef MOTOR_H
#define MOTOR_H

class Motor
{
  public:
    Motor();
    Motor(int pinPWM, int pinBrake,int pinDirection, bool inverted=false);
    void setSpeed(int);
    int getSpeed();
    void ARU();
  private:
    int pins [3];
    int speed=0;
    bool disabled_=false;
    bool inverted_;
};

#endif
