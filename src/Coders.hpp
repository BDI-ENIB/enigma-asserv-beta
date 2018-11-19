#ifndef CODERS_H
#define CODERS_H 1
#include <Encoder.h>
class Coders{
  public:
    Coders(int p11,int p12,int p21,int p22);
    ~Coders();
    int left();
    int right();
    void reset();

  private:
    Encoder *lc_;
    Encoder *rc_;
};
#endif
