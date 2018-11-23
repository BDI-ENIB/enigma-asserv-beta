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

void Controller::setTarget(Point *checkpoints, int checkpointAmount, double targetedAngle){
  this->checkpoints = checkpoints;
  this->targetedAngle = targetedAngle;

  this->checkpointAmount = checkpointAmount;
  currentCheckpoint = 0;
  rotationOnly = true;
}

double circstrain(double c){
  while(c<-3.1415)c+=2*3.1415;
  while(c>3.1415)c-=2*3.1415;
  return c;
}

// Cache
double angleToTarget, distanceToNode, currentSpeed, idealSpeed, idealRotationSpeed, currentRotationSpeed, targetedRotationSpeed;

void Controller::update(double posX, double posY, double currentAngle){

  circstrain(currentAngle);

  currentSpeed = sqrt(pow(lastPosX-posX,2)+pow(lastPosY-posY, 2))/(micros()-lastUpdate);
  currentRotationSpeed = (currentAngle-lastAngle)*DEMI_ECART_ROUES;
  lastUpdate=micros();
  lastPosY = posY;
  lastPosX = posX;
  lastAngle = currentAngle;

  //targetSelection:;

  // Si on est arrivé à l'objectif, on ne fait plus rien
  if(currentCheckpoint>checkpointAmount){
    return;
  }

  // calcul de la distance et de l'angle à l'objectif:
  distanceToNode=sqrt(pow(checkpoints[currentCheckpoint].x-posX,2)+pow(checkpoints[currentCheckpoint].y-posY,2));
  angleToTarget=atan2(checkpoints[currentCheckpoint].y-posY,checkpoints[currentCheckpoint].x-posX); // cet angle doit tendre vers 0

  /*if(distanceToNode<PRECISION_DISTANCE){
    currentCheckpoint++;
    rotationOnly = true;
    goto targetSelection;
  }*/

  /*if(angleToTarget<PRECISION_ANGLE){
    rotationOnly = false;
  }*/

  if(rotationOnly){
    targetedSpeed = 0;
  }else{
    // Calcul de la consigne en vitesse:
    idealSpeed = distanceToNode/MAX_ACCELERATION;
    if(abs(currentSpeed-idealSpeed)<MAX_ACCELERATION){
      targetedSpeed = idealSpeed;
    }else if(currentSpeed<idealSpeed){
      targetedSpeed += MAX_ACCELERATION;
    }else{
      targetedSpeed -= MAX_ACCELERATION;
    }
  }

  //commande de rotation
  idealRotationSpeed = angleToTarget*DEMI_ECART_ROUES*MAX_ACCELERATION_ROTATION;
  if(abs(currentRotationSpeed-idealRotationSpeed)<MAX_ACCELERATION_ROTATION){
    targetedRotationSpeed = idealRotationSpeed;
  }else if(currentRotationSpeed<idealRotationSpeed){
    targetedRotationSpeed = currentRotationSpeed + MAX_ACCELERATION_ROTATION;
  }else{
    targetedRotationSpeed = currentRotationSpeed - MAX_ACCELERATION_ROTATION;
  }

  leftMotor->setTarget(targetedSpeed+targetedRotationSpeed);
  rightMotor->setTarget(targetedSpeed-targetedRotationSpeed);

  curvature->update(angleToTarget);
  leftMotor->update(currentSpeed-currentRotationSpeed*DEMI_ECART_ROUES);
  rightMotor->update(currentSpeed+currentRotationSpeed*DEMI_ECART_ROUES);
}

int Controller::getLCommand(){
  return 0;
  return max(-MAX_PWM, min(MAX_PWM, leftMotor->getCommand()));
}

int Controller::getRCommand(){
  return 0;
  return max(-MAX_PWM, min(MAX_PWM, rightMotor->getCommand()));
}
