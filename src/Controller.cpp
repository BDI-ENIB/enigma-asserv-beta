/*
class Controller{
  public:
    Controller(double P, double I, double D);
    ** Commandes de création de trajectoire
     * En gros l'idée c'est de décomposer nos trajectoires en clothoïdes.
     *
     typedef struct {double x,y;} Point;
     void setTarget(Vector<Point> checkpoints, double targetAngle);

    **
     * Commandes de génération de commande
     *
    void update(double posX, double posY, double currentAngle);
    int[] getCommand();

  private:
    // Définition de la trajectoire
    std::vector<Point> checkpoints;
    double targetedAngle;

    // Sous-éléments du controller & autre
    PID leftMotor, rightMotor;
};
*/

#include "Controller.hpp"

Controller::Controller(double P, double I, double D){
  // Asserv en vitesse => commande de base à 0, robot immobile
  leftMotor = new PID(P, I, D, 0);
  rightMotor = new PID(P, I, D, 0);
}

void Controller::setTarget(std::vector<Point> checkpoints, double targetedAngle){
  this->checkpoints = checkpoints;
  this->targetedAngle = targetedAngle;
}

void Controller::update(double posX, double posY, double currentAngle){
  leftMotor.update(currentSpeedL);
  rightMotor.update(currentSpeedR);
}

int[] Controller::getCommand(){
  return [leftMotor.getCommand(), rightMotor.getCommand()];
}
