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

  lastUpdate=micros();
  lastPosY = 0.0;
  lastPosX = 0.0;

  checkpointAmount = 0; //Car la liste checkpoints[] n'est pas initialisée
  currentCheckpoint = 1; //Par défaut, il est considéré que nous sommes arrivés à destination.... oui, sérieusement x)
}

void Controller::setTarget(Point[] checkpoints, int checkpointAmount, double targetedAngle){
  this->checkpoints = checkpoints;
  this->targetedAngle = targetedAngle;

  this->checkpointAmount = checkpointAmount;
  currentCheckpoint = 0;
  isRotating = true;
}

double targetedAngle, distanceToNode, currentSpeed, idealSpeed;
void Controller::update(double posX, double posY, double currentAngle){

  currentSpeed = sqrt(pow(lastPosX-posX,2)+pow(lastPosY-posY, 2))/(micros()-lastUpdate);
  lastUpdate=micros();
  lastPosY = posY;
  lastPosX = posX;

  targetSelection:;

  // Si on est arrivé à l'objectif, on ne fait plus rien
  if(currentCheckpoint>checkpointAmount){
    return;
  }

  // calcul de la distance et de l'angle à l'objectif:
  distanceToNode=sqrt(pow(checkpoints[currentCheckpoint].x-x,2)+pow(checkpoints[currentCheckpoint].y-y,2));
  targetedAngle=atan2(ty-y,tx-x);

  if(distanceToNode<PRECISION){
    currentCheckpoint++;
    isRotating = true;
    goto targetSelection;
  }

  if(isRotating){
    
  }else{
    // Calcul de la consigne en vitesse:
    idealSpeed=distanceToNode/MAX_ACCELERATION;
    if(abs(currentSpeed-idealSpeed)<MAX_ACCELERATION){
      targetedSpeed = idealSpeed;
    }else if(currentSpeed<idealSpeed){
      targetedSpeed += MAX_ACCELERATION;
    }else{
      targetedSpeed -= MAX_ACCELERATION;
    }
  }

  leftMotor.update(currentSpeedL);
  rightMotor.update(currentSpeedR);
}

int[] Controller::getCommand(){
  return [leftMotor.getCommand(), rightMotor.getCommand()];
}
