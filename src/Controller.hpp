#ifndef CONTROLLER_H
#define CONTROLLER_H 1

#include "PID.hpp"

#include <vector>

#define MAX_ACCELERATION 0.2
#define PRECISION 10

typedef struct {double x,y;} Point;

class Controller{
  public:
    Controller(double P, double I, double D);

    /** Commandes de création de trajectoire
     * En gros l'idée c'est de décomposer nos trajectoires en clothoïdes.
     */
     void setTarget(std::vector<Point> checkpoints, double targetedAngle);

    /**
     * Commandes de génération de commande
     */
    void update(double posX, double posY, double currentAngle);
    int[] getCommand();

  private:
    // Définition de la trajectoire
    std::vector<Point> checkpoints;
    double targetedAngle;
    double targetedSpeed;

    long lastUpdate;
    double lastPosX, lastPosY;

    int currentCheckpoint;
    int checkpointAmount;

    bool isRotating;

    // Sous-éléments du controller & autre
    PID leftMotor, rightMotor;

};

#endif
