#include "src/Coders.hpp"
#include "src/Odometry.hpp"
#include "src/Controller.hpp"
#include "src/Motor.hpp"

#include <Arduino.h>

#define LED 13

IntervalTimer controlTimer;
Coders coders(36,35,33,34);
Odometry odometry(1000,1000,0,265.0,16.0,20000); // X Y Alpha
Controller controller(0.02,0.00,0.00); // PID
Motor leftMotor(3,4,5); // aka motor 1
Motor rightMotor(10,26,27,true); // PWM, brake, direction

void setup(){
  // On ouvre la connexion
  Serial.begin(250000);
  delay(3000);

  // On paramètre les pins
  pinMode(LED, OUTPUT);

  // On redéfinit la target
  Point checkpoints[] = {{1000,1000},{1200,1000},{1000,1000}};
  controller.setTarget(checkpoints, 3, -PI/2); // checkpints, nb de checkpoints, angle à l'arrivée

  // On lance l'asservissement
  controlTimer.begin(mainLoop, 4166);
  controlTimer.priority(129);
}

void loop(){
  //Serial.println((String)odometry.getX()+" "+odometry.getY());
  controller.log();
  delay(50);
  digitalWrite(LED, HIGH);
  delay(50);
  digitalWrite(LED, LOW);
}

void mainLoop(){
    odometry.move(coders.right(),coders.left());
    controller.update(odometry.getX(),odometry.getY(),odometry.getA()); // cm -> mm

    leftMotor.setSpeed(controller.getLCommand());
    rightMotor.setSpeed(controller.getRCommand());
}
