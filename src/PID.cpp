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

  lastCommand=0; lastUpdate=micros();
}

void PID::setTarget(double target){
  this->target = target;
}

void PID::update(double input){
  // On incrémente l'erreur cumulée
  cumulatedError+=input-target;
  // TODO: Jarter cette variable quand on a un changement de target. En fait, créer une fonction pour reset le I, et laisser le controller gérer

  // Le proportionnel:
  lastCommand = (input-target)*P;

  // La partie intégrée:
  lastCommand += cumulatedError*I;

  // La partie dérivée:
  lastCommand += (input-target)*D;

  // On met à jour la var de sauvegarde de l'entrée
  // TODO: repenser cette var
  lastInput = input;
}

double PID::getCommand(){
  return lastCommand;
}
