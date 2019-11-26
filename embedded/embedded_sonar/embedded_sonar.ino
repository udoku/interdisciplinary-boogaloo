#include <ArduinoHardware.h>
#include <ros.h>
#include <robot_pkg/ServoCommand.h>
#include <robot_pkg/UltrasonicPing.h>

#include <Servo.h>

#define NUM_SENSORS 6

#define ECHO_PIN 7
#define SERVO_PIN 8

#define METERS_PER_US 0.000343
#define PING_WAIT 10000

const int sensor_pins[NUM_SENSORS] = {1, 2, 3, 4, 5, 6};

ros::NodeHandle nh;
robot_pkg::UltrasonicPing ping_msg;
int active_sensor;

Servo servo;

// Pass servo commands
void servo_callback(const robot_pkg::ServoCommand msg){
  if (msg.servo_id == 7) {
    servo.write(msg.value);
  }
}

ros::Subscriber<robot_pkg::ServoCommand> servo_sub("servo_command", servo_callback);
ros::Publisher ping_pub("ultrasonic_ping", &ping_msg);

void read_sensor() {
  int pin = sensor_pins[active_sensor];

  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);

  double duration = pulseIn(ECHO_PIN, HIGH, PING_WAIT);
  double distance = duration * METERS_PER_US / 2;

  ping_msg.sensor_id = active_sensor;
  ping_msg.distance = distance;
  ping_pub.publish(&ping_msg);
  
  active_sensor = (active_sensor + 1) % NUM_SENSORS;
}

void setup() {
  // put your setup code here, to run once:
  nh.initNode(); 

  // TODO: setup pins
  servo.attach(SERVO_PIN);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensor_pins[i], OUTPUT);
  }
  pinMode(ECHO_PIN, INPUT);

  active_sensor = 0;
  
  nh.subscribe(servo_sub);
  nh.advertise(ping_pub);
}

void loop() {
  nh.spinOnce();
  read_sensor();
}
