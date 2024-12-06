#ifndef CORE_BASE_H_
#define CORE_BASE_H_

#include <micro_ros_arduino.h>
#include "car_info.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/timer.h>
#include <rcutils/logging_macros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

// ros/time.h
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/twist.h>
#include <tf2_msgs/msg/tf_message.h>
#include <std_msgs/msg/float64.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <nav_msgs/msg/odometry.h>

#include <stdio.h>
#include <math.h>

#define FIRMWARE_VER "1.0.0"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define WHEEL_NUM                        4
#define LEFT                             0
#define RIGHT                            1
#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI
#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
#define M_PI 3.14159265358979323846

#define FILTER_WINDOW_SIZE               10         // Hall Sensor window size

// #define DEBUG
#define SERIAL_DEBUG                     1

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// PIN Define
#define LED_ERROR                        22
#define LED_WORKING_CHECK                23
#define BLDC                              5
#define SERVO                             6
#define HALLA                             2
#define HALLB                            17
#define HALLC                            18

// Callback function prototypes
void commandVelocityCallback(const void *cmd_vel_msg);
void imuCallback(const void *imu_msg);

// Function prototypes - publish
void publishCmdVelEchMsg(void);

// Function prototypes - update robot states
void updateOdometry(void);
void updateTF();
void updateGoalVelocity(void);
//void updateTime(void);

// Init

bool calcOdometry(double diff_time);
void hallSeonsor_ISR(void);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
rcl_node_t node;
rcl_time_point_value_t current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char* get_tf_prefix = "my_robot";

char odom_header_frame_id[30];
char odom_child_frame_id[30];

/*******************************************************************************
* Subscriber
*******************************************************************************/
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_sub_msg;

/*******************************************************************************
* Publisher
*******************************************************************************/
rcl_publisher_t linx_echo_pub;
std_msgs__msg__Float64 linx_echo_msg;

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
uint32_t tTime[10];

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
volatile float deltaT_buffer[FILTER_WINDOW_SIZE] = {0};
unsigned long lastTimeA = 0;
volatile int buffer_index         = 0; // Index for writing to the buffer
volatile int sample_count         = 0; // Total number of samples in the buffer
volatile int motor_rpm = 0;
volatile int wheel_rpm = 0;
double current_linear_velocity = 0;

/*******************************************************************************
* Declaration for controllersr
*******************************************************************************/
// Turtlebot3Controller controllers;
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;

/*******************************************************************************
* Update control message from RC controller
*******************************************************************************/
void getDataFromRemoteController(void);
void controlServoMotor(void);
void controlBLDCMotor(void);

// etc
rcl_subscription_t subscriber;
rclc_executor_t executor;   // 콜백 실행, 메시지 처리
rcl_allocator_t allocator;  // 메모리 관리
rclc_support_t support;     // uros 초기화, 관리
uint32_t t;

bool mode_rc = false; // false 일 경우 자율주행모드, true일 경우 rc카 모드

void error_loop();

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

#endif // CORE_BASE_H_

