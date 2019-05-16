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

void Controller::setTarget(Point checkpoints[], int checkpointAmount, double targetedAngle){
  for(int i = 0; i<checkpointAmount;++i){
    this->checkpoints[i]=checkpoints[i];
  }
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
double dt, angleToTarget, distanceToNode, currentSpeed, idealSpeed, idealRotationSpeed, currentRotationSpeed, targetedRotationSpeed;

void Controller::update(double posX, double posY, double currentAngle){

  circstrain(currentAngle);

  currentSpeed = sqrt(pow(lastPosX-posX,2)+pow(lastPosY-posY, 2))/(micros()-lastUpdate);
  currentRotationSpeed = (currentAngle-lastAngle)*DEMI_ECART_ROUES;

  dt = (lastUpdate-micros())/1000;
  lastUpdate=micros();
  lastPosY = posY;
  lastPosX = posX;
  lastAngle = currentAngle;

  // La relecture pique les yeux. Qui a mit un goto ici?? (oups) #doWhile
  //TODO: refaire. Avec un switch case clair: Mode maintien de position, mode rotation, mode déplacement.
  targetSelection:;

  // Si on est arrivé à l'objectif, on ne fait plus rien
  if(currentCheckpoint>=checkpointAmount){
    return;
  }

  // calcul de la distance et de l'angle à l'objectif:
  distanceToNode=sqrt(pow(checkpoints[currentCheckpoint].x-posX,2)+pow(checkpoints[currentCheckpoint].y-posY,2));
  angleToTarget=circstrain(atan2(checkpoints[currentCheckpoint].y-posY,checkpoints[currentCheckpoint].x-posX)-currentAngle); // cet angle doit tendre vers 0
  //log();

  if(distanceToNode<PRECISION_DISTANCE){
    //Serial.println("#######Changement de cible!#######");
    currentCheckpoint++;
    rotationOnly = true;
    goto targetSelection;
  }

  if(abs(angleToTarget)<PRECISION_ANGLE){
    rotationOnly = false;
  }

  // Calcul de la consigne en vitesse:
  if(rotationOnly){
    targetedSpeed = 0;
  }else{
    idealSpeed = distanceToNode/MAX_ACCELERATION;
    if(abs(currentSpeed-idealSpeed)<MAX_ACCELERATION*dt){
      targetedSpeed = idealSpeed;
    }else if(currentSpeed<idealSpeed){
      targetedSpeed = currentSpeed + MAX_ACCELERATION*dt;
    }else{
      targetedSpeed = currentSpeed - MAX_ACCELERATION*dt;
    }
  }

  //commande de rotation
  idealRotationSpeed = angleToTarget*DEMI_ECART_ROUES*MAX_ACCELERATION_ROTATION;
  if(abs(currentRotationSpeed-idealRotationSpeed)<MAX_ACCELERATION_ROTATION*dt){
    targetedRotationSpeed = idealRotationSpeed;
  }else if(currentRotationSpeed<idealRotationSpeed){
    targetedRotationSpeed = currentRotationSpeed + MAX_ACCELERATION_ROTATION*dt;
  }else{
    targetedRotationSpeed = currentRotationSpeed - MAX_ACCELERATION_ROTATION*dt;
  }

  leftMotor->setTarget(targetedSpeed+targetedRotationSpeed);
  rightMotor->setTarget(targetedSpeed-targetedRotationSpeed);

  leftMotor->update(currentSpeed-currentRotationSpeed*DEMI_ECART_ROUES);
  rightMotor->update(currentSpeed+currentRotationSpeed*DEMI_ECART_ROUES);
}

int Controller::getLCommand(){
  return (currentCheckpoint>=checkpointAmount)?0:max(-MAX_PWM, min(MAX_PWM, leftMotor->getCommand()));
}

int Controller::getRCommand(){
  return (currentCheckpoint>=checkpointAmount)?0:max(-MAX_PWM, min(MAX_PWM, rightMotor->getCommand()));
}


bool finished=false;
void Controller::log(){
  if(currentCheckpoint<=checkpointAmount){
    Serial.println("");
    Serial.println("dt: "+(String)dt);
    Serial.println("Distance to target " + (String)distanceToNode);
    Serial.println("Angle to target " + (String)(angleToTarget/PI*180));
    Serial.println("Commands: L"+(String)leftMotor->getCommand()+" R"+(String)rightMotor->getCommand()+" (T"+(String)targetedSpeed+", RS"+(String)targetedRotationSpeed+")");
    Serial.println("Target index: "+(String)currentCheckpoint);
    Serial.println("Target:" +(String)checkpoints[currentCheckpoint].x +';'+ (String)checkpoints[currentCheckpoint].y);
    Serial.println("Position:" +(String)lastPosX +';'+ lastPosY);
    Serial.println("Angle: "+(String)lastAngle);
  }
}
