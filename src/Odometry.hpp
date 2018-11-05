#ifndef ODOMETRY_H
#define ODOMETRY_H 1

class Odometry{
  public:
    Odometry(double x0,double y0,double a0,double L,double R,int S);
    void move(int dl,int dr);
    double getX();
    double getY();
    double getA();
    void set(double x0,double y0,double a0);
  private:
    double x_,y_,a_,l_,r_;
    int s_;
};
#endif
