#ifndef CODERS_H
#define CODERS_H 1
#include <Encoder.h>
class Coders{
  public:
    Coders(int pLeftB,int pLeftA,int pRightA,int pRightB);
    ~Coders();
    int left();
    int right();
    void reset();

  private:
    Encoder *lc_;
    Encoder *rc_;
};
#endif
