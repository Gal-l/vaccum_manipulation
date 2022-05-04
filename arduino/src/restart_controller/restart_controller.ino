#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <PID_v1.h>
#include <Encoder.h>

ros::NodeHandle  nh;

//Define Variables we'll be connecting to
double Setpoint = 0, Input = 0, Output, switch_val;


//Specify the links and initial tuning parameters
double Kp=0.5, Ki=0.1, Kd=0.1;

PID motorPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Encoder motorEnc(2, 3);

// motor pins
int motor_pin = 6;
int motor_dir = 5;
int limit_switch = 4;

// Define Ecoder PPR
const double encoder_ratio = 360 / 1288.8;        // gear ration base one 11*6 PPR encoder 
double angle2mm = 640.0/3500.0 * 360.0 / 4096.0;

void motor_dis_callback( const std_msgs::Float64& msg){
  Setpoint = msg.data + (double)(motorEnc.read() * encoder_ratio);
}


std_msgs::Float64 pid_error_msg;
std_msgs::Float64 encoder_read_msg;
std_msgs::Float64 switch_read_msg;

ros::Subscriber<std_msgs::Float64> sub_motor_move("/restart_controller/motor_move", &motor_dis_callback );
ros::Publisher publish_pid_error("/restart_controller/pid_error", &pid_error_msg);
ros::Publisher publish_encoder_read("/restart_controller/encoder_read", &encoder_read_msg);
ros::Publisher publish_switch_read("/restart_controller/switch_read", &switch_read_msg);

void setup()
{
  nh.initNode();
  nh.subscribe(sub_motor_move);
  nh.advertise(publish_pid_error);
  nh.advertise(publish_encoder_read);
  nh.advertise(publish_switch_read);
  
  //turn the PID on
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-120, 120);

  pinMode(motor_pin, OUTPUT);
  pinMode(motor_dir, OUTPUT);
  pinMode(limit_switch, INPUT_PULLUP);
}


void loop()
{
  // Pirint motor position
  Input = (double)(motorEnc.read() * encoder_ratio);
  switch_val = digitalRead(limit_switch);

  double error = Setpoint-Input;
  
  pid_error_msg.data = error;
  encoder_read_msg.data = Input;
  switch_read_msg.data = switch_val;
  
  publish_pid_error.publish(&pid_error_msg);
  publish_encoder_read.publish(&encoder_read_msg);
  publish_switch_read.publish(&switch_read_msg);
  
  if (abs(error ) > 5.0){ 
    motorPID.Compute();
  
    if (Output > 0){
      digitalWrite(motor_dir, LOW);
    }else{
      digitalWrite(motor_dir, HIGH);
    }
  
    // Send PWN signal to the motor according to the PID
    analogWrite(motor_pin, abs(Output));
  }
  
  else
  {
    analogWrite(motor_pin, 0);
  }
  
  nh.spinOnce();
  delay(1);

}
