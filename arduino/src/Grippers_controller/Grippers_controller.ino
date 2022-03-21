#include <Servo.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <PID_v1.h>


ros::NodeHandle  nh;

int tool_changer = 1; //0 is to release gripper and 1 is to lock the gripper
int tool_changer_pin = 5 ;
int grasp_object = 162;// 0 to open finger gripper or to grasp with glue gripper or to suck with vaccum gripper. 255 to lock with finger gripper or to release glue gripper  . 146 release with vaccum gripper
int grasp_object_pin = 3;
int glue_servo_spin = 90; // 90 stop and 0 is to spin
int pot_pin = A1;
int pid_finger_mode = 0;
int fingers_force = 0;
int main_valve_pin = 6;
int main_valve = 1 ;// 0 propotional valve close. 1 - propotional valve open

Servo glue_servo;
std_msgs::Int16 pot_msg;
std_msgs::Int16 pwm_msg;


int C_output = 0;
int pot_value = 0;

double current_pose = 0, pwmVal = 0.0, fingers_target = 20.0;
double Kp=0.3, Ki=0.000001, Kd=0.02;
PID fingersPID(&current_pose, &pwmVal, &fingers_target, Kp, Ki, Kd, DIRECT);



void gripper_change_callback( const std_msgs::Int8& msg){
  tool_changer = msg.data;
  digitalWrite(tool_changer_pin, tool_changer);
}


void grasp_object_callback( const std_msgs::Int16& msg){
  grasp_object = msg.data;
  analogWrite(grasp_object_pin, grasp_object);
}

void glue_spin_callback( const std_msgs::Int16& msg){
  glue_servo_spin = msg.data;
  glue_servo.write(glue_servo_spin);
}


void fingers_target_callback( const std_msgs::Int16& msg){
  fingers_target = msg.data;
}

void pid_finger_mode_callback( const std_msgs::Int8& msg){
  pid_finger_mode = msg.data;
}

void fingers_force_callback( const std_msgs::Int8& msg){
  fingers_force = msg.data;
  Fingers_gripper_force(fingers_force);
}

void main_valve_callback( const std_msgs::Int8& msg){
  main_valve = msg.data;
  digitalWrite(main_valve_pin , main_valve);
}




ros::Subscriber<std_msgs::Int8> sub_tool_changer("tool_changer", &gripper_change_callback );
ros::Subscriber<std_msgs::Int8> sub_main_valve("main_valve", &main_valve_callback );
ros::Subscriber<std_msgs::Int8> sub_fingers_pid_mode("pid_finger_mode", &pid_finger_mode_callback );
ros::Subscriber<std_msgs::Int8> sub_fingers_force("fingers_force", &fingers_force_callback );
ros::Subscriber<std_msgs::Int16> sub_grasp_object("grasp_object", &grasp_object_callback );
ros::Subscriber<std_msgs::Int16> sub_glue_spin("glue_spin", &glue_spin_callback );
ros::Subscriber<std_msgs::Int16> sub_fingers_target("fingers_gripper_target", &fingers_target_callback );
ros::Publisher publish_pot("pid_error", &pot_msg);
ros::Publisher publish_pid_pwm("pwm_value", &pwm_msg);



void setup()
{ 
  pinMode(tool_changer_pin, OUTPUT);
  pinMode(grasp_object_pin, OUTPUT);
  pinMode(pot_pin,INPUT);
  pinMode(main_valve_pin,OUTPUT);
  glue_servo.attach(10);
  nh.initNode();
  nh.subscribe(sub_tool_changer);
  nh.subscribe(sub_grasp_object);
  nh.subscribe(sub_glue_spin);
  nh.subscribe(sub_fingers_target);
  nh.subscribe(sub_fingers_pid_mode);
  nh.subscribe(sub_fingers_force);
  nh.subscribe(sub_main_valve);
  nh.advertise(publish_pot);
  nh.advertise(publish_pid_pwm);
  digitalWrite(tool_changer_pin, tool_changer);
  analogWrite(grasp_object_pin, grasp_object);
  digitalWrite(main_valve_pin , main_valve);
  glue_servo.write(glue_servo_spin);

  //turn the PID on
  fingersPID.SetMode(AUTOMATIC);
  fingersPID.SetOutputLimits(-10, 10);
}

void loop()
{  
  nh.spinOnce();
  if(pid_finger_mode==1){
  pot_value=analogRead(pot_pin);
  publish_pot.publish( &pot_msg );
  P_controller_fingers_gripper(pot_value,fingers_target);
  }
  delay(1);
}



float pot_value2fingers_gripper_pos(int pot_value)
{
  return -0.0456*(pot_value)+58;
}

float fingers_gripper_pos2pot_value(int fingers_gripper_pos)
{
  return -0.0456*(pot_value)+58;
}



void P_controller_fingers_gripper(int pot_value,int target)
{
  float ui = 0;
  float u = 0;
  int e = 999;
  int C_output = 0;

  pot_value=analogRead(pot_pin);
  e = pot_value2fingers_gripper_pos(pot_value) - target;
  current_pose = pot_value2fingers_gripper_pos(pot_value);
  pot_msg.data = e;
  publish_pot.publish( &pot_msg );
  grasp_object = 157;
  
  if(abs(e)>5){
    fingersPID.Compute();
    grasp_object -= pwmVal;
//    grasp_object = 162; /
  }
  pwm_msg.data = (int)grasp_object;
  publish_pid_pwm.publish(&pwm_msg);
  analogWrite(grasp_object_pin, grasp_object);
 }



void Fingers_gripper_force(int fingers_force){
  analogWrite(grasp_object_pin,fingers_force + grasp_object);
}
