#include "src/Coders.hpp"
#include "src/Odometry.hpp"
#include "src/Controller.hpp"
#include "src/Motor.hpp"

#include <Arduino.h>

#define LED 13

IntervalTimer controlTimer;
Coders coders(35,36,34,33);
Odometry odometry(1000,1000,0,260.0,16.0,4096); // X Y Alpha
Controller controller(0.045,0.0,0.2); // PID
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
      path[0]={odometry.getX()+command*cos(odometry.getA()), odometry.getY()+command*sin(odometry.getA())};
      controller.setTarget(path,1,odometry.getA());
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
    controller.update(odometry.getX(),odometry.getY(),odometry.getA()); // cm -> mm

    if(!pause && enabled){
        leftMotor.setSpeed(controller.getLCommand());
        rightMotor.setSpeed(controller.getRCommand());
    }else{
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
    }
}
