#ifndef CONTROLLER_H
#define CONTROLLER_H 1

#include "PID.hpp"

#include <vector>

class Controller{
  public:
    Controller(double P, double I, double D);
    /** Commandes de création de trajectoire
     * En gros l'idée c'est de décomposer nos trajectoires en clothoïdes.
     */
     typedef struct {double x,y;} Point;
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

    // Sous-éléments du controller & autre
    PID leftMotor, rightMotor;

};

#endif
