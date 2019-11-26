#include <ArduinoHardware.h>
#include <ros.h>
#include <robot_pkg/ServoCommand.h>
#include <robot_pkg/LedCommand.h>

#include <Servo.h>

#define DISABLE_LED 2
#define ARMED_LED 3
#define COMPLETE_LED 4

#define NUM_SERVOS 7

ros::NodeHandle nh;
int led_state;

Servo servos[NUM_SERVOS];

// Pass servo commands
void servo_callback(const robot_pkg::ServoCommand msg){
  if (0 <= msg.servo_id && msg.servo_id < NUM_SERVOS) {
    servos[msg.servo_id].write(msg.value);
  }
}

// Update LED intended state
void led_callback(const robot_pkg::LedCommand msg) {
  led_state = msg.state;
}

ros::Subscriber<robot_pkg::ServoCommand> servo_sub("servo_command", servo_callback);
ros::Subscriber<robot_pkg::LedCommand> led_sub("led_command", led_callback);

// Update LED actual state
void led_update() {
  if (led_state == robot_pkg::LedCommand::DISABLED) {
    digitalWrite(DISABLE_LED, HIGH);
    digitalWrite(ARMED_LED, LOW);
    digitalWrite(COMPLETE_LED, LOW);
  }
  else if (led_state == robot_pkg::LedCommand::ARMED) {
    digitalWrite(DISABLE_LED, LOW);
    // Hacky way of blinking
    if (millis() % 1300 <= 100) {
      digitalWrite(ARMED_LED, HIGH);
    }
    else {
      digitalWrite(ARMED_LED, LOW);
    }
    digitalWrite(COMPLETE_LED, LOW);
  }
  else {
    digitalWrite(DISABLE_LED, LOW);
    digitalWrite(ARMED_LED, LOW);
    if (millis() % 2000 < 1000) {
      digitalWrite(COMPLETE_LED, HIGH);
    }
    else {
      digitalWrite(COMPLETE_LED, LOW);
    }
    
  }
}

void setup() {
  // put your setup code here, to run once:
  nh.initNode(); 

  // TODO: setup pins
  servos[0].attach(11);
  servos[1].attach(9);
  servos[2].attach(7);
  servos[3].attach(12);
  servos[4].attach(10);
  servos[5].attach(8);
  servos[6].attach(6);

  led_state = robot_pkg::LedCommand::DISABLED;

  pinMode(DISABLE_LED, OUTPUT);
  pinMode(ARMED_LED, OUTPUT);
  pinMode(COMPLETE_LED, OUTPUT);
  
  nh.subscribe(servo_sub);
  nh.subscribe(led_sub);
}

void loop() {
  nh.spinOnce();
  led_update();
}
