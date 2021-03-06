#include "src/Coders.hpp"
#include "src/Odometry.hpp"
#include "src/Controller.hpp"
#include "src/Motor.hpp"

#include <Arduino.h>

#define LED 13

IntervalTimer controlTimer;
Coders coders(35,36,34,33);
Odometry odometry(1000,1000,0,260.0,16.0,4096); // X Y Alpha
Controller controller(0.045,0.0,0.6); // PID
Motor leftMotor(3,4,5,true); // aka motor 1
Motor rightMotor(10,26,27,false); // PWM, brake, direction
Point path[16]{};
volatile bool pause = false;
volatile bool enabled = false;

void setup(){
  // On ouvre la connexion
  Serial.begin(250000);
  delay(3000);

  // On paramètre les pins
  pinMode(LED, OUTPUT);

  // On redéfinit la target
  Point checkpoints[] = {{1000,1000}};
  controller.setTarget(checkpoints, 1, 0); // checkpints, nb de checkpoints, angle à l'arrivée
  // former: targetedAngle -> -PI/2

  // On lance l'asservissement
  controlTimer.begin(mainLoop, 4166);
  controlTimer.priority(129);
}

void loop(){
  #ifdef DEBUG
  controller.log();
  delay(50);
  digitalWrite(LED, HIGH);
  delay(50);
  digitalWrite(LED, LOW);
  #else
  digitalWrite(LED, HIGH);
  String s = Serial.readStringUntil(';');
  if(s.equals("")) return;
  for(unsigned int i = 0; i<s.length(); i++){
      if(s.charAt(i)=='\n') return;
  }
  if(s.startsWith("forward:")){
      // Commande possible: "forward:100"
      int command = s.substring(8).toInt();
      path[0]={odometry.getX()+command*cos(controller.getTargetedAngle()), odometry.getY()+command*sin(controller.getTargetedAngle())};
      controller.setTarget(path,1,(command>0)?controller.getTargetedAngle():PI-controller.getTargetedAngle());
  }else if(s.startsWith("rotate:")){
      // Commande possible: "rotate:3.1415"
      double command = s.substring(7).toFloat();
      // Serial.print(String(command));
      path[0]={odometry.getX(), odometry.getY()};
      controller.setTarget(path,0,controller.getTargetedAngle()+command);
  }else if(s.startsWith("whois")){
      Serial.print("MotionBase;");
  }else if(s.startsWith("pause")){
    pause = true;
  }else if(s.startsWith("resume")){
    pause = false;
}else if(s.startsWith("activate")){
    enabled = true;
}else if(s.startsWith("deactivate")){
    enabled = false;
  }else{
      Serial.print("Undefined Command: '"+s+"';");
  }
  #endif
}

void mainLoop(){
    odometry.move(coders.right(),coders.left());

    if(!pause && enabled){
        controller.update(odometry.getX(),odometry.getY(),odometry.getA()); // cm -> mm
        leftMotor.setSpeed(controller.getLCommand());
        rightMotor.setSpeed(controller.getRCommand());
    }else{
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
    }
}
