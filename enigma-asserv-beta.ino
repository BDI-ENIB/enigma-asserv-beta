#include "src/Coders.hpp"
#include "src/Odometry.hpp"
#include "src/Controller.hpp"
#include "src/Motor.hpp"

#include <Arduino.h>

#define LED 13

IntervalTimer controlTimer;
Coders coders(35,36,34,33);
Odometry odometry(1000,1000,0,260.0,16.0,4096); // X Y Alpha
Controller controller(0.02,0.00,0.00); // PID
Motor leftMotor(3,4,5,true); // aka motor 1
Motor rightMotor(10,26,27,false); // PWM, brake, direction

void setup(){
  // On ouvre la connexion
  Serial.begin(250000);
  delay(3000);

  // On paramètre les pins
  pinMode(LED, OUTPUT);

  // On redéfinit la target
  Point checkpoints[] = {{1000,1000},{1400,1000},{1400,-1400},{1000,1000}};
  controller.setTarget(checkpoints, 4, -PI/2); // checkpints, nb de checkpoints, angle à l'arrivée

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
