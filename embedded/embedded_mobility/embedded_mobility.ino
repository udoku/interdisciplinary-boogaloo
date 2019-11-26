#include <ArduinoHardware.h>
#include <ros.h>
#include <robot_pkg/ServoCommand.h>

#include <Servo.h>

ros::NodeHandle handle;

Servo servos[6];

void msg_cb(const robot_pkg::ServoCommand msg){
  if (0 <= msg.servo_id && msg.servo_id < 6) {
    servos[msg.servo_id].write(msg.value);
  }
}

ros::Subscriber<robot_pkg::ServoCommand> sub("servo_command", msg_cb);

void setup() {
  // put your setup code here, to run once:
  handle.initNode(); 

  // TODO: setup pins
  servos[0].attach(5);
  servos[1].attach(6);
  servos[2].attach(7);
  servos[3].attach(8);
  servos[4].attach(9);
  servos[5].attach(10);
  
  handle.subscribe(sub);
}

void loop() {
  handle.spinOnce();
}
