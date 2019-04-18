#ifndef ODOMETRY_H
#define ODOMETRY_H 1

class Odometry{
  public:
    /**
     * @param:
     *    {xO, y0, a0}:   base coords
     *    L:              half-interwheel
     *    R:              wheel radius
     *    S:              coder steps per revolution
     */
    Odometry(double x0,double y0,double a0,double L,double R,int S);
    /**
     * @param:
     *    cl:     left counter delta
     *    cr:     right counter delta
     */
    void move(int cl,int cr);
    double getX();
    double getY();
    double getA();
    void set(double x0,double y0,double a0);
  private:
    double x_,y_,a_,l_,r_;
    int s_;
};
#endif
