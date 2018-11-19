#include "Coders.hpp"
#include <Encoder.h>
Coders::Coders(int pl1,int pl2,int pr1,int pr2){
  lc_=new Encoder(pl1, pl2);
  rc_=new Encoder(pr1, pr2);
}
Coders::~Coders(){
  delete lc_;
  delete rc_;
}
int Coders::left(){
  int a = lc_->read();
  lc_->write(0);
  return a;
}
int Coders::right(){
  int a = rc_->read();
  rc_->write(0);
  return a;
}
void Coders::reset(){
  lc_->write(0);
  rc_->write(0);
}
