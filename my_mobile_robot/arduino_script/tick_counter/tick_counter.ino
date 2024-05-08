#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 8

#define WHEEL_DISTANCE 0.201
#define mps_ticks 224.299082191

float left_speed_setpoint = 0;
float right_speed_setpoint = 0;

 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 32-bit integers
const long encoder_minimum = -2147483648;
const long encoder_maximum = 2147483647;


void cmdVelCallback(const geometry_msgs::Twist &msg) 
{

  float linearVel = msg.linear.x;
  float angularVel = msg.angular.z;
  
  // Calculate motor speeds based on the robot's wheel distance
  left_speed_setpoint = (linearVel - (WHEEL_DISTANCE * angularVel) / 2)*mps_ticks;
  right_speed_setpoint = (linearVel + (WHEEL_DISTANCE * angularVel) / 2)*mps_ticks;

}


// Keep track of the number of wheel ticks
std_msgs::Int32 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int32 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

//Subscribe to cmdvel topic to control robot with rqt or other via cmd_vel
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", cmdVelCallback); 

// One-second interval for measurements
int interval = 100;
long previousMillis = 0;
long currentMillis = 0;


// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}


void setup() 
{
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(cmdVelSub);
}

//Doesn't finish yet
void loop() 
{
  // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of ticks
  if (currentMillis - previousMillis > interval) 
  {
    previousMillis = currentMillis;
     
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
    nh.spinOnce();
  }
  delay(1);
}
