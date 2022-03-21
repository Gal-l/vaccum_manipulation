#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <PID_v1.h>
#include <Encoder.h>

ros::NodeHandle  nh;

//Define Variables we'll be connecting to
double Setpoint = 0, Input, Output;

//Specify the links and initial tuning parameters
double Kp=1.0, Ki=0.0, Kd=0.12;

PID motorPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Encoder motorEnc(2, 3);

// motor pins
int motor_pin = 6;
int motor_dir = 5;
double angle2mm = 640.0/3500.0 * 360.0 / 4096.0;

void conv_dis_callback( const std_msgs::Float64& msg){
  Setpoint = msg.data + (double)(motorEnc.read() * angle2mm);
}


std_msgs::Float64 pid_error_msg;

ros::Subscriber<std_msgs::Float64> sub_conveyor_move("/conveyor_controller/conveyor_move", &conv_dis_callback );
ros::Publisher publish_pid_error("/conveyor_controller/pid_error", &pid_error_msg);

void setup()
{
  nh.initNode();
  nh.subscribe(sub_conveyor_move);
  nh.advertise(publish_pid_error);

  Input = 0;
  
  //turn the PID on
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-70, 70);

  pinMode(motor_pin, OUTPUT);
  pinMode(motor_dir, OUTPUT);
}


void loop()
{
  // Pirint motor position
  Input = (double)(motorEnc.read() * angle2mm);

  double error = Setpoint-Input;
  pid_error_msg.data = error;
  publish_pid_error.publish(&pid_error_msg);
  
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
  
  nh.spinOnce();
  delay(1);

}
