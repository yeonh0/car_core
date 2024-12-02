#include "core_base.h"

void error_loop() 
{
  while(1){
    digitalWrite(LED_ERROR, !digitalRead(LED_ERROR));
    delay(100);
  }
}

void setup() 
{
 /*******************************************************************************
  * Initialize Micro ROS
  *******************************************************************************/
  set_microros_transports();                                                // Init transport
  allocator = rcl_get_default_allocator();                                  // Init allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));                // Init support
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));  // Create executor

  /*******************************************************************************
  * ROS NodeHandle
  *******************************************************************************/
  RCCHECK(rclc_node_init_default(&node, "opencr_node", "", &support));

  /*******************************************************************************
  * Subscriber
  *******************************************************************************/
  RCCHECK(rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_sub_msg, &commandVelocityCallback, ON_NEW_DATA));

  RCCHECK(rclc_subscription_init_default(&imu_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));
  RCCHECK(rclc_executor_add_subscription(&executor, &imu_sub, &imu_sub_msg, &imuCallback, ON_NEW_DATA));

  /*******************************************************************************
  * Publisher
  *******************************************************************************/
  RCCHECK(rclc_publisher_init_default(&cmd_vel_echo_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel_rc100"));

  RCCHECK(rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

  RCCHECK(rclc_publisher_init_default(&tf_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf_broadcaster"));

  /*******************************************************************************
  * Setting for SLAM and Navigation
  *******************************************************************************/
  initOdom();
  initPrefix();
  
  /*******************************************************************************
  * Pin mode set (INPUT/OUTPUT, Interrupt)
  *******************************************************************************/
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_WORKING_CHECK, OUTPUT);
  pinMode(HALLA, INPUT);
  pinMode(HALLB, INPUT);
  pinMode(HALLC, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALLA), hallSeonsor_ISR, RISING);

  prev_update_time = millis();
  Serial.setTimeout(1);

  #if SERIAL_DEBUG
    Serial.begin(115200);
  #endif
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop() 
{
  t = millis();
  initOdom();
  // updateTime();

  /* motor contorol section : tTime[0] - MOTOR SIGNAL SEQUNCE */
  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();     // 특정 주기마다 goal velocity 업데이트

    if (!mode_rc && (t-tTime[6]) > CONTROL_MOTOR_TIMEOUT)     // 일정시간 동안 입력이 없을 시, 속도 값을 0으로 설정
    {
      // motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, zero_velocity);
    } 
    else {
      // motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
    }
    tTime[0] = t;
  }

  /* publish cmd_vel_echo section - 특정 주기마다 echo cmd_vel 데이터 퍼블리시 : tTime[1] - CMD VEL ECHO MSG SIGNAL SEQUENCE */
  if ((t-tTime[1]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY))
  {
    publishCmdVelEchMsg();
    tTime[1] = t;
  }

    /* publish odom, tf section - 특정 주기마다 odom, tf 데이터 퍼블리시 : tTime[2] - ODOM, TF MSG SIGNAL SEQUENCE*/
  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    updateMotorInfo();
    publishDriveInformation();
    tTime[2] = t;
  }

    /* RC mode check section : tTime[3] - MOTOR MODE CHECK SEQUENCE */
    getDataFromRemoteController(); // RC컨트롤러로부터 제어 정보 지속적으로 업데이트
    if (mode_rc)
    {
        tTime[3] = t;
    }

  // Call all the callbacks waiting to be called at that point in time
  RCCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));
}

/*******************************************************************************
* Publish msgs (CMD Velocity data from RC100 : angular velocity, linear velocity)
*******************************************************************************/
void publishCmdVelEchMsg(void)
{
  cmd_vel_echo_msg.linear.x  = round(goal_velocity[LINEAR]);
  cmd_vel_echo_msg.angular.z = round(goal_velocity[ANGULAR]);

  RCSOFTCHECK(rcl_publish(&cmd_vel_echo_pub, &cmd_vel_echo_msg, NULL));
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const void *cmd_vel_msg) 
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)cmd_vel_msg;

  goal_velocity_from_cmd[LINEAR]  = msg->linear.x;
  goal_velocity_from_cmd[ANGULAR] = msg->angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}

/*******************************************************************************
 * get controller data from RC Controller
 *******************************************************************************/
void getDataFromRemoteController(void)
{
  
}

/*******************************************************************************
* Callback function for imu msg
*******************************************************************************/
void imuCallback(const void *imu_msg)
{
  // Quat to Euler -> Extract yaw data
  const sensor_msgs__msg__Imu * msg = (const sensor_msgs__msg__Imu *)imu_msg;

  double x = msg->orientation.x;
  double y = msg->orientation.y;
  double z = msg->orientation.z;
  double w = msg->orientation.w;

  ahrs_yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  if (mode_rc) // rc 모드일 경우 goal velocity를 RC컨트롤러 값으로 업데이트
    {
        goal_velocity[LINEAR] = goal_velocity_from_rc[LINEAR];
        goal_velocity[ANGULAR] = goal_velocity_from_rc[ANGULAR];
    }
    else // 자율주행 모드일 경우 cmd 데이터로 부터 goal velocity 업데이트
    {
        goal_velocity[LINEAR] = goal_velocity_from_cmd[LINEAR];
        goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
    }

  // sensors.setLedPattern(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
}

/*******************************************************************************
* Update Motor Information (Hall sensor data)
*******************************************************************************/
void updateMotorInfo(void)
{
  
}

/*******************************************************************************
* Publish msgs (odometry, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // calculate Quaternion from yaw
  double yaw = odom_pose[2];
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  qx = 0.0;
  qy = 0.0;
  qz = sy;
  qw = cy;

  // odometry
  updateOdometry();
  odom_msg->header.stamp.nanosec = tv.tv_nsec;
  odom_msg->header.stamp.sec = tv.tv_sec;
  RCSOFTCHECK(rcl_publish(&odom_pub, odom_msg, NULL));

  // odometry tf
  updateTF();
  tf_msg->transforms.data[0].header.stamp.nanosec = tv.tv_nsec;
	tf_msg->transforms.data[0].header.stamp.sec = tv.tv_sec;
  RCSOFTCHECK(rcl_publish(&tf_pub, tf_msg, NULL));
}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom_msg->pose.pose.position.x = odom_pose[0];
  odom_msg->pose.pose.position.y = odom_pose[1];
  odom_msg->pose.pose.position.z = 0;

  odom_msg->pose.pose.orientation.x = qx;
  odom_msg->pose.pose.orientation.y = qy;
  odom_msg->pose.pose.orientation.z = qz;
  odom_msg->pose.pose.orientation.w = qw;

  odom_msg->twist.twist.linear.x  = odom_vel[0];
  odom_msg->twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Update the tf 
*******************************************************************************/
void updateTF()
{
  tf_msg->transforms.data[0].transform.translation.x = odom_pose[0];
  tf_msg->transforms.data[0].transform.translation.y = odom_pose[1];
  tf_msg->transforms.data[0].transform.translation.z = 0;

  tf_msg->transforms.data[0].transform.rotation.x    = qx;
  tf_msg->transforms.data[0].transform.rotation.y    = qy;
  tf_msg->transforms.data[0].transform.rotation.z    = qz;
  tf_msg->transforms.data[0].transform.rotation.w    = qw;
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
  //orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}

/*******************************************************************************
* Initialize message's frame id
*******************************************************************************/
void initPrefix(void)
{
  char log_msg[50];
  char string1[] = "odom";
  char string2[] = "base_footprint";
  
  tf_msg = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&tf_msg->transforms, 1);

  tf_msg->transforms.data[0].header.frame_id.data = (char*)malloc(100*sizeof(char));
  memcpy(tf_msg->transforms.data[0].header.frame_id.data, string1, strlen(string1) + 1);
  tf_msg->transforms.data[0].header.frame_id.size = strlen(tf_msg->transforms.data[0].header.frame_id.data);
  tf_msg->transforms.data[0].header.frame_id.capacity = 100;

  tf_msg->transforms.data[0].child_frame_id.data =  (char*)malloc(100*sizeof(char));
  memcpy(tf_msg->transforms.data[0].child_frame_id.data, string2, strlen(string2) + 1);
  tf_msg->transforms.data[0].child_frame_id.size = strlen(tf_msg->transforms.data[0].child_frame_id.data);
  tf_msg->transforms.data[0].child_frame_id.capacity = 100;

  odom_msg->header.frame_id.data = (char*)malloc(100*sizeof(char));
  memcpy(odom_msg->header.frame_id.data, string1, strlen(string1) + 1);
  odom_msg->header.frame_id.size = strlen(odom_msg->header.frame_id.data);
  odom_msg->header.frame_id.capacity = 100;

  odom_msg->child_frame_id.data =  (char*)malloc(100*sizeof(char));
  memcpy(odom_msg->child_frame_id.data, string2, strlen(string2) + 1);
  odom_msg->child_frame_id.size = strlen(odom_msg->child_frame_id.data);
  odom_msg->child_frame_id.capacity = 100;

  sprintf(odom_header_frame_id, string1);
  sprintf(odom_child_frame_id, string2);
  sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
  RCUTILS_LOG_INFO_NAMED("opencr_node", log_msg);
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom_msg->pose.pose.position.x = 0.0;
  odom_msg->pose.pose.position.y = 0.0;
  odom_msg->pose.pose.position.z = 0.0;

  odom_msg->pose.pose.orientation.x = 0.0;
  odom_msg->pose.pose.orientation.y = 0.0;
  odom_msg->pose.pose.orientation.z = 0.0;
  odom_msg->pose.pose.orientation.w = 0.0;

  odom_msg->twist.twist.linear.x  = 0.0;
  odom_msg->twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Hall Sensor ISR
*******************************************************************************/
void hallSeonsor_ISR(void)
{
  unsigned long currentTime = micros();  // 현재 시간(us) 단위로 읽기
  unsigned long timeDiff = currentTime - lastTimeA;  // 두 이벤트 간 시간 차 계산
  lastTimeA = currentTime;
  
  // RPM 계산: (60 * 1,000,000) / (한 바퀴 당 이벤트 수 * 시간 차)
  rpm = (60 * 1000000L) / (6 * timeDiff);  // 6은 한 바퀴에 발생하는 이벤트 수
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
// ros::Time rosNow() { }