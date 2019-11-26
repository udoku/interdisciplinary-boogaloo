#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <robot_pkg/UltrasonicPing.h>

#define NUM_SENSORS 6

#define KILL_PIN 4
#define ECHO_PIN 3

#define METERS_PER_US 0.000343
#define PING_WAIT 10000

const int sensor_pins[NUM_SENSORS] = {12, 11, 10, 9, 8, 7};

ros::NodeHandle nh;

robot_pkg::UltrasonicPing ping_msg;
std_msgs::Bool kill_msg;

int active_sensor;


ros::Publisher killswitch_pub("killswitch", &kill_msg);
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

// Update killswitch
void kill_update() {
  kill_msg.data = digitalRead(KILL_PIN);
  killswitch_pub.publish(&kill_msg);
}

void setup() {
  // put your setup code here, to run once:
  nh.initNode(); 

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensor_pins[i], OUTPUT);
  }
  pinMode(ECHO_PIN, INPUT);
  pinMode(KILL_PIN, INPUT);

  active_sensor = 0;
  
  nh.advertise(ping_pub);
  nh.advertise(killswitch_pub);
}

void loop() {
  nh.spinOnce();
  read_sensor();
  kill_update();
}
