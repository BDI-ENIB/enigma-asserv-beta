#ifndef MOTOR_H
#define MOTOR_H

class Motor
{
  public:
    Motor();
    Motor(int,int,int);
    void setSpeed(int);
    int getSpeed();
    void ARU();
  private:
    int pins [3];
    int speed=0;
    bool disabled=false;
};

#endif
