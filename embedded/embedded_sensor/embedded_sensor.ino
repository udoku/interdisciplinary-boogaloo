#include <ArduinoHardware.h>
#include <ros.h>
#include <robot_pkg/ServoCommand.h>
#include <robot_pkg/UltrasonicPing.h>

#include <Servo.h>

// echo of all 6 ultrasound sensors must be attached to the same interrupt pin,
// since there are only 2 interrupt pins
const int INTERRUPT_PIN = 2; // must be 2 or 3 (I think)

// how many ultrasonic sensors to use
const int NUM_US = 6;
// the start of the 6 pins used for the ultrasound sensors
const int US_START_PIN = 6;
// maximum ultrasonic echo time (in microseconds)
const long US_RANGE_TIMEOUT = 3000;

void servo_msg_cb(const robot_pkg::ServoCommand msg);

// ros state
ros::NodeHandle handle;
robot_pkg::UltrasonicPing range_msg;
ros::Publisher range_pub("us_range", &range_msg);
ros::Subscriber<robot_pkg::ServoCommand> servo_sub("servo_command", servo_msg_cb);

// the time the trigger pulse was sent
long echo_start_time = 0;
// the echo time in microseconds
long echo_time = 0;
// the index of the ultrasonic sensor that is currently being used
int current_us = 0;
// a measurement has been detected and should be sent
bool range_ready = false;

Servo dropper_servo;

void servo_msg_cb(const robot_pkg::ServoCommand msg){
  // only care about last servo
  if (msg.servo_id == 6) {
    dropper_servo.write(msg.value);
  }
}


// short callback measuring the echo time and telling the main loop a measurement has been made
void us_cb() {
  // try to make sure we avoid counting reflections
  if (!range_ready) {
    echo_time = micros() - echo_start_time;
  
    range_ready = true;
  }
}

void setup() {
  // put your setup code here, to run once:
  handle.initNode(); 

  // TODO: setup pins
  dropper_servo.attach(5);

  handle.subscribe(servo_sub);
  handle.advertise(range_pub);

  // attach a callback to any echo from an ultrasonic sensor
  pinMode(INTERRUPT_PIN, INPUT);
  digitalWrite(INTERRUPT_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), us_cb, RISING);
}

const float M_PER_US = 3.4027;

// post a range message on the "range" (?) topic
void send_range_message(long us, int sensor) {
  range_msg.sensor_id = current_us;
  range_msg.distance = 0.5 * M_PER_US * us;
  range_pub.publish(&range_msg);
}

void loop() {

  // timeout if we don't receive an echo
  if (micros() - echo_start_time > US_RANGE_TIMEOUT) {
    echo_time = US_RANGE_TIMEOUT;
    range_ready = true;
  }
  
  // if a new measurement has been made recently, send it via ros
  if (range_ready) {
    digitalWrite(US_START_PIN + current_us, LOW);
    
    send_range_message(echo_time, current_us);

    current_us = (current_us + 1) % NUM_US;

    // start the measurement on the next sensor
    echo_start_time = micros();
    range_ready = false;
    digitalWrite(US_START_PIN + current_us, HIGH);
  }


  handle.spinOnce();
}
