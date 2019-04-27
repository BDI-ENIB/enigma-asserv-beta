/***
class PID{
  public:
    PID(double P, double I, double D, double target);
    void setTarget(double target);
    double update(double input);
  private:
    double P, I, D, target;
    double lastCommand, lastInput, cumulatedError;
    long lastUpdate;
}
*/

#include "PID.hpp"
#include "Arduino.h"

PID::PID(double P, double I, double D, double target){
  this->P = P;
  this->I = I;
  this->D = D;
  this->target = target;

  lastCommand=0;
  lastDelta=0;
  lastUpdate=micros();
}

void PID::setTarget(double target){
  this->target = target;
}

double delta;
void PID::update(double input){
  delta=target-input;

  // On incrémente l'erreur cumulée
  cumulatedError+=delta*(micros()-lastUpdate)/1000;
  lastUpdate=micros();

  // TODO: Jarter cette variable quand on a un changement de target. En fait, créer une fonction pour reset le I, et laisser le controller gérer

  // Le proportionnel:
  lastCommand = delta*P;

  // La partie intégrée:
  lastCommand += cumulatedError*I;

  // La partie dérivée:
  lastCommand += (delta-lastDelta)*D;

  // On met à jour la var de sauvegarde de l'entrée
  lastInput = input;
  lastDelta = 0.2*lastDelta+0.8*delta;
}

double PID::getCommand(){
  return lastCommand;
}
