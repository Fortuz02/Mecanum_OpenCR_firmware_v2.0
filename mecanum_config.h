#ifndef MECANUM_CONFIG_H_
#define MECANUM_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include <math.h>

#include "mecanum_motor_driver.h"

// #define DEBUG                            
#define DEBUG_SERIAL                     SerialBT2
#define FIRMWARE_VER                     "1.1"

/*******************************************************************************
* Update frequencies and Timing
*******************************************************************************/
#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VOLTAGE_PUBLISH_FREQUENCY              50   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

/*******************************************************************************
* Robot parameters
*******************************************************************************/
#define NAME                            "Mecanum"

#define WHEEL_RADIUS                    0.03      // meter
#define WHEEL_SEPARATION_X              0.1005    // 170mm/2 + 31mm/2
#define WHEEL_SEPARATION_Y              0.085     // 200mm/2 - 30mm/2
#define DISTANCE_CENTER_TO_WHEEL        0.165     // meter

#define ENCODER_MIN                     -2147483648     // raw
#define ENCODER_MAX                     2147483648      // raw

#define RPM_CONSTANT_VALUE              0.229

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_LINEAR_Y               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_LINEAR_Y         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define MECANUMWHEEL_NUM                4
#define LIMIT_X_MAX_VALUE               480

#define CONTROL_PERIOD                  8000

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI
#define TICK2RAD                        0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
MecanumMotorDriver motor_driver;
double goal_linear_x_velocity  = 0.0;
double goal_linear_y_velocity  = 0.0;
double goal_angular_velocity   = 0.0;

static uint32_t tTime[10];                // Software timer
uint32_t t = 0;

bool setup_end        = false;
uint8_t battery_state = 0;

/*******************************************************************************
* Callback function prototypes
*******************************************************************************/
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);
void messageCb(const std_msgs::Empty& toggle_msg);

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<std_msgs::Empty> sub_toggle("toggle", &messageCb);
ros::Subscriber<geometry_msgs::Twist> sub_cmd("cmd_vel", &cmdVelCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
std_msgs::Float32 voltage_msg;
ros::Publisher pub_voltage("voltage", &voltage_msg);

/*******************************************************************************
* Function prototypes
*******************************************************************************/
void controlMecanum();

#endif // MECANUM_CONFIG_H_

// Ref : H.Taheri, B.Qiao, N.Ghaeminezhad, "Kinematic Model of a Four Mecanum Wheeled Mobile Robot",
//       International Journal of Computer Applications, 3 March 2015
