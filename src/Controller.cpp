#include "Controller.hpp"

Controller::Controller(double P, double I, double D){
  // Asserv en vitesse => commande de base à 0, robot immobile
  leftMotor = new PID(P, I, D, 0);
  rightMotor = new PID(P, I, D, 0);
  curvature = new PID(P, I, D, 0);

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

double targetAngle, distanceToNode, currentSpeed, idealSpeed, idealRotation, rotationSpeed, targetedRotationSpeed;
void Controller::update(double posX, double posY, double currentAngle){

  currentSpeed = sqrt(pow(lastPosX-posX,2)+pow(lastPosY-posY, 2))/(micros()-lastUpdate);
  rotationSpeed = (currentAngle-lastAngle)*ECART_ROUES/2*MAX_ACCELERATION;
  lastUpdate=micros();
  lastPosY = posY;
  lastPosX = posX;
  lastAngle = currentAngle;

  targetSelection:;

  // Si on est arrivé à l'objectif, on ne fait plus rien
  if(currentCheckpoint>checkpointAmount){
    return;
  }

  // calcul de la distance et de l'angle à l'objectif:
  distanceToNode=sqrt(pow(checkpoints[currentCheckpoint].x-x,2)+pow(checkpoints[currentCheckpoint].y-y,2));
  targetAngle=atan2(ty-y,tx-x); // cet angle doit tendre vers 0

  if(distanceToNode<PRECISION){
    currentCheckpoint++;
    isRotating = true;
    goto targetSelection;
  }


  // Calcul de la consigne en vitesse:
  idealSpeed=distanceToNode/MAX_ACCELERATION;
  if(abs(currentSpeed-idealSpeed)<MAX_ACCELERATION){
    targetedSpeed = idealSpeed;
  }else if(currentSpeed<idealSpeed){
    targetedSpeed += MAX_ACCELERATION;
  }else{
    targetedSpeed -= MAX_ACCELERATION;
  }

  if(isRotating){
    //commande de rotation
    idealRotation = targetAngle*ECART_ROUES/2*MAX_ACCELERATION_ROTATION;

    if(abs(rotationSpeed-idealSpeed)<MAX_ACCELERATION_ROTATION){
      targetedRotationSpeed = idealSpeed;
    }else if(rotationSpeed<idealSpeed){
      targetedRotationSpeed += MAX_ACCELERATION_ROTATION;
    }else{
      targetedRotationSpeed -= MAX_ACCELERATION_ROTATION;
    }

    leftMotor.setTarget(targetedSpeed+targetRotation);
    rightMotor.setTarget(targetedSpeed-targetRotation);
  }else{
    leftMotor.setTarget(targetedSpeed);
    rightMotor.setTarget(targetedSpeed);
  }

  curvature.update(targetAngle);
  leftMotor.update(currentSpeedL);
  rightMotor.update(currentSpeedR);
}

int[] Controller::getCommand(){
  return [max(-MAX_PWM, min(MAX_PWM, leftMotor.getCommand())), max(-MAX_PWM, min(MAX_PWM, rightMotor.getCommand()))];
}
