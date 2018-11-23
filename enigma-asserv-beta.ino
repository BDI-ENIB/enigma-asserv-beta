#include "src/Coders.hpp"
#include "src/Odometry.hpp"
#include "src/Controller.hpp"
#include "src/Motor.hpp"
#include <Arduino.h>

#define LED 13

IntervalTimer controlTimer;
Coders coders(33,34,35,36);
Odometry odometry(1000,1000,0,265.0,16.0,20000);
Controller controller(0,0,0);
Motor leftMotor(2,3,4);
Motor rightMotor(5,6,7);

void setup(){
  // On ouvre la connexion
  Serial.begin(250000);
  delay(1000);

  // On paramètre les pins
  pinMode(LED, OUTPUT);

  // On redéfinit la target
  Point checkpoints[3] = {{150,0}, {150, 150}, {0, 150}};
  controller.setTarget(std::move(checkpoints), 3, 0); // checkpints, nb de checkpoints, angle à l'arrivée

  // On lance l'asservissement
  controlTimer.begin(mainLoop, 4166);
  controlTimer.priority(129);

}

void loop(){
  delay(500);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
}

void mainLoop(){
    odometry.move(coders.left(),coders.right());
    controller.update(odometry.getX(),odometry.getY(),odometry.getA());

    leftMotor.setSpeed(controller.getLCommand());
    rightMotor.setSpeed(controller.getRCommand());
}
