#ifndef PID_H
#define PID_H 1

class PID{
  public:
    PID(double P, double I, double D, double target);
    void setTarget(double target);
    void update(double input);
    double getCommand();
  private:

    double P, I, D, target;
    double lastCommand, lastInput, lastDelta, cumulatedError;
    long lastUpdate;
};

#endif
