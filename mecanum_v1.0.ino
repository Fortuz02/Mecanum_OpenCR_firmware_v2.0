/*
 * EZ MŰKÖDIK 
 * 
 * roscore
 * roslaunch mecanum mecanum_port.launch
 * rostopic pub toggle std_msgs/Empty --once
 * 
 * Toggle módba bekapcsolja és kikapcsolja az előre menetelt
 */

#include "mecanum_config.h"

void setup()
{
  DEBUG_SERIAL.begin(57600);

  nh.initNode();
  nh.getHardware()->setBaud(115200);
  
  // Subscribers
  nh.subscribe(sub_toggle);
  nh.subscribe(sub_cmd);
  
  // Publishers
  nh.advertise(pub_voltage);

  // Setting for Dynamixel motors
  motor_driver.init();
  
  pinMode(13, OUTPUT);

  setup_end = true;
}

void loop()
{
  t = millis();

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY)){
    if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT) {
      goal_linear_x_velocity  = 0.0;
      goal_linear_y_velocity  = 0.0;
      goal_angular_velocity   = 0.0;
    } 
    controlMecanum();
    tTime[0] = t;
  }

  if ((t-tTime[1]) >= (1000 / VOLTAGE_PUBLISH_FREQUENCY )){
    voltage_msg.data = getPowerInVoltage();
    pub_voltage.publish(&voltage_msg);
    tTime[1] = millis();
  }
  
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
  
}

/*******************************************************************************
* Callback function for cmd_vel
*******************************************************************************/
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel){
  goal_linear_x_velocity  = cmd_vel.linear.x;
  goal_linear_y_velocity  = cmd_vel.linear.y;
  goal_angular_velocity   = cmd_vel.angular.z;
  tTime[6] = millis();
}

/*******************************************************************************
* Callback function for LED toggle blink
*******************************************************************************/
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

/*******************************************************************************
* Function for motor controll 
*******************************************************************************/
void controlMecanum()
{
  bool dxl_comm_result = false;

  int64_t wheel_value[MECANUMWHEEL_NUM] = {0, 0, 0, 0};
  double wheel_angular_velocity[MECANUMWHEEL_NUM] = {0.0, 0.0, 0.0, 0.0};

  wheel_angular_velocity[0] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity - goal_linear_y_velocity - (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  wheel_angular_velocity[1] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity + goal_linear_y_velocity + (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  wheel_angular_velocity[2] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity + goal_linear_y_velocity - (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  wheel_angular_velocity[3] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity - goal_linear_y_velocity + (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);

  for (int id = 0; id < MECANUMWHEEL_NUM; id++)
  {
    wheel_value[id] = wheel_angular_velocity[id] * 9.54 / RPM_CONSTANT_VALUE;

    if (wheel_value[id] > LIMIT_X_MAX_VALUE)       wheel_value[id] =  LIMIT_X_MAX_VALUE;
    else if (wheel_value[id] < -LIMIT_X_MAX_VALUE) wheel_value[id] = -LIMIT_X_MAX_VALUE;
  }

  dxl_comm_result = motor_driver.controlMotor(-(int64_t)wheel_value[0], -(int64_t)wheel_value[1], (int64_t)wheel_value[2], (int64_t)wheel_value[3]);
  if (dxl_comm_result == false)
    return;
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}
