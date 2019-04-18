#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Motor.hpp"

/**
 *  PWM:
 *  Brake:      ralentissement? HIGH pour activer le moteur
 *  Direction:
 */
Motor::Motor(int pinPWM,int pinBrake,int pinDirection, bool inverted){
  pins[0]=pinPWM;
  pins[1]=pinBrake;
  pins[2]=pinDirection;
  inverted_=inverted;
  for(int i=0;i<3;i++){
    pinMode(pins[i],OUTPUT);
    digitalWrite(pins[i],(i==1?HIGH:LOW));
  }
}

void Motor::ARU(){
  speed=0;
  disabled_=true;
  for(int i=0;i<3;i++){
    digitalWrite(pins[i],LOW);
  }
}

void Motor::setSpeed(int spd){
  if(disabled_) return;
  speed=spd;
  if(inverted_?speed<0:speed>0){
    digitalWrite(pins[2], HIGH);
  }else{
    digitalWrite(pins[2], LOW);
  }
  analogWrite(pins[0],abs(speed));
}

int Motor::getSpeed(){
  return speed;
}
