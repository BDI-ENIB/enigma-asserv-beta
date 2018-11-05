#include "Arduino.h"
#include "Odometry.hpp"

const float opi = 3.141592;

double ocircstrain(double c){
  while(c<-opi)c+=2*opi;
  while(c>opi)c-=2*opi;
  return c;
}

Odometry::Odometry(double x0,double y0,double a0,double L,double R,int S){
  x_=x0;
  y_=y0;
  a_=a0;
  l_=L;
  r_=R;
  s_=S;
}
void Odometry::move(int cl,int cr){
  double dr=2*opi*r_*((double)cr)/((double)s_);
  double dl=2*opi*r_*((double)cl)/((double)s_);
  a_ = ocircstrain(a_ + (dr-dl)/l_);
  double z=(dr+dl)/2;
  x_+=cos(a_)*z;
  y_+=sin(a_)*z;
}
double Odometry::getX(){
  return x_;
}
double Odometry::getY(){
  return y_;
}
double Odometry::getA(){
  return a_;
}
void Odometry::set(double x0,double y0,double a0){
  x_=x0;
  y_=y0;
  a_=a0;
}
