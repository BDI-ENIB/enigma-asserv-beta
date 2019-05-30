#ifndef CONTROLLER_H
#define CONTROLLER_H 1

#include "PID.hpp"
#include "Arduino.h"

#define MAX_ACCELERATION 0.0003
#define MAX_ACCELERATION_ROTATION 12
#define MAX_PWM 50
#define PRECISION_DISTANCE 20
#define PRECISION_ANGLE 0.025
#define THRESHOLD_ROTATION_SPEED 0.01
#define OUTPUT_PRECISON_MODE 25
#define DEMI_ECART_ROUES 160
#define MAX_CHECKPOINT_AMOUNT 18

typedef struct {double x=0; double y=0;} Point;

class Controller{
  public:
    Controller(double P, double I, double D);

    /**
     * Commandes de création de trajectoire
     */
     void setTarget(Point checkpoints[], int checkpointAmount, double targetedAngle);

    /**
     * Commandes de génération de commande
     */
    void update(double posX, double posY, double currentAngle);
    int getLCommand();
    int getRCommand();

    void log();

  private:
    // Définition de la trajectoire
    Point checkpoints[MAX_CHECKPOINT_AMOUNT];
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
