#ifndef CONTROLLER_H
#define CONTROLLER_H 1

#include "PID.hpp"
#include "Arduino.h"

#define MAX_ACCELERATION 0.2
#define MAX_ACCELERATION_ROTATION 0.1
#define MAX_PWM 255
#define PRECISION_DISTANCE 10
#define PRECISION_ANGLE 0.2
#define DEMI_ECART_ROUES 160

typedef struct {double x,y;} Point;

class Controller{
  public:
    Controller(double P, double I, double D);

    /** Commandes de création de trajectoire
     * En gros l'idée c'est de décomposer nos trajectoires en clothoïdes.
     */
     void setTarget(Point *checkpoints, int checkpointAmount, double targetedAngle);

    /**
     * Commandes de génération de commande
     */
    void update(double posX, double posY, double currentAngle);
    int getLCommand();
    int getRCommand();

  private:
    // Définition de la trajectoire
    Point *checkpoints;
    double targetedAngle;
    double targetedSpeed;

    long lastUpdate;
    double lastPosX, lastPosY, lastAngle;

    int currentCheckpoint;
    int checkpointAmount;

    bool rotationOnly;

    // Sous-éléments du controller & autre
    PID *leftMotor, *rightMotor;

};

#endif
