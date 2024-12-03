#include "core_base.h"

void error_loop() 
{
  while(1){
    digitalWrite(LED_ERROR, !digitalRead(LED_ERROR));
    delay(100);
  }
}
rcl_publisher_t helohelo;
std_msgs__msg__Int32 msg;

void setup() 
{
 /*******************************************************************************
  * Initialize Micro ROS
  *******************************************************************************/
  set_microros_transports();                                                // Init transport
  delay(2000);
  allocator = rcl_get_default_allocator();                                  // Init allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));                // Init support

  /*******************************************************************************
  * ROS NodeHandle
  *******************************************************************************/
  RCCHECK(rclc_node_init_default(&node, "opencr_node", "", &support));

  /*******************************************************************************
  * Publisher
  *******************************************************************************/
  RCCHECK(rclc_publisher_init_default(&cmd_vel_echo_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel_rc100"));
  RCCHECK(rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
  RCCHECK(rclc_publisher_init_default(&tf_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf_broadcaster"));
  RCCHECK(rclc_publisher_init_default(&helohelo, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "helohelo"));

  /*******************************************************************************
  * Subscriber
  *******************************************************************************/
  RCCHECK(rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&imu_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));  // Create executor
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_sub_msg, &commandVelocityCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &imu_sub, &imu_sub_msg, &imuCallback, ON_NEW_DATA));

  /*******************************************************************************
  * Setting for SLAM and Navigation
  *******************************************************************************/
  initOdom();
  initPrefix();

  /*******************************************************************************
  * Motors setup (BLDC, Servo)
  *******************************************************************************/
  drv_pwm_set_freq(SERVO, 500);
  drv_pwm_setup(SERVO);
  drv_pwm_set_duty(SERVO, 10, 512);

  drv_pwm_set_freq(BLDC, 100);
  drv_pwm_setup(BLDC);
  drv_pwm_set_duty(BLDC, 10, 512);
  
  /*******************************************************************************
  * Pin mode set (INPUT/OUTPUT, Interrupt)
  *******************************************************************************/
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_WORKING_CHECK, OUTPUT);
  pinMode(HALLA, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALLA), hallSeonsor_ISR, RISING);

  prev_update_time = millis();
  Serial.setTimeout(1);

  Serial.begin(115200);
}

/*******************************************************************************
* Pin mode set (INPUT/OUTPUT, Interrupt)
*******************************************************************************/
void controlServoMotor()
{
  // Kinematic Bicycle Model에 의해 앞바퀴 각도를 계산
  float linear_x = goal_velocity[LINEAR], angular_z = goal_velocity[ANGULAR];
  float duty = (MIN_DUTY + MAX_DUTY)/2;
  if (angular_z || linear_x) {
    float steering_angle_radians = atan(WHEEL_SEPARATION * angular_z / linear_x);

    // 라디안 값을 도(degree) 단위로 변환
    float steering_angle_degrees = steering_angle_radians * (180.0 / M_PI);

    // 각도가 -60도에서 60도 사이로 제한되도록 클리핑
    if (steering_angle_degrees < MIN_ANGLE) steering_angle_degrees = MIN_ANGLE;
    if (steering_angle_degrees > MAX_ANGLE) steering_angle_degrees = MAX_ANGLE;

    // 각도와 duty cycle을 선형적으로 변환
    duty = (steering_angle_degrees - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE) * (MAX_DUTY - MIN_DUTY) + MIN_DUTY;
  }
  int pwmval = map(duty, 0, 100, 0, 1023);     // scale it for use with the servo (value between 0 and 180)
  drv_pwm_set_duty(SERVO, 10, pwmval);
}

void controlBLDCMotor()
{
  // Compute velocity using buffered deltaT values
  float avgDeltaT = 0;
  int currentSampleCount = sample_count;
  for (int i = 0; i < currentSampleCount; i++) {
    avgDeltaT += deltaT_buffer[i];
  }

  if (currentSampleCount > 0) {
    avgDeltaT /= currentSampleCount; // Average of deltaT values
  } else {
    avgDeltaT = 1.0; // Prevent division by zero
  }
  motor_rpm = 60.0 / avgDeltaT; // Convert average deltaT to velocity
  wheel_rpm = motor_rpm / DRIVE_RATIO;
  current_linear_velocity = (wheel_rpm * 2.0 * M_PI * WHEEL_RADIUS) / 60.0;

///////////////// //////////////////////////
  float linear_x = goal_velocity[LINEAR], angular_z = goal_velocity[ANGULAR];
  
  if (linear_x < -1.0) linear_x = -1.0;
  else if (linear_x > 1.0) linear_x = 1.0;

  // linear_x를 -1.0에서 1.0 → 13에서 17로 선형 변환
  float linxduty = 13 + (linear_x + 1.0) * (17.0 - 13.0) / 2.0;

  // duty를 0~100 → 0~1023으로 변환
  int pwmVal = (int)(linxduty * 1023.0 / 100.0);

  drv_pwm_set_duty(BLDC, 10, pwmVal);
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
      controlServoMotor();
      controlBLDCMotor();
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
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

/*******************************************************************************
* Publish msgs (CMD Velocity data from RC100 : angular velocity, linear velocity)
*******************************************************************************/
void publishCmdVelEchMsg(void)
{
  cmd_vel_echo_msg.linear.x  = goal_velocity[LINEAR];
  cmd_vel_echo_msg.angular.z = goal_velocity[ANGULAR];

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

  // IMU orientation
  imu_orientation[0] = msg->orientation.x;
  imu_orientation[1] = msg->orientation.y;
  imu_orientation[2] = msg->orientation.z;
  imu_orientation[3] = msg->orientation.w;

  // IMU angular covariance
  for (int i = 0; i < 9; ++i) {
    imu_angular_covariance[i] = msg->angular_velocity_covariance[i];
  }

  // IMU angular_velocity
  imu_angular_velocity[0] = msg->angular_velocity.x;
  imu_angular_velocity[1] = msg->angular_velocity.y;
  imu_angular_velocity[2] = msg->angular_velocity.z;

  // IMU linear_acceleration
  imu_linear_acceleration[0] = msg->linear_acceleration.x;
  imu_linear_acceleration[1] = msg->linear_acceleration.y;
  imu_linear_acceleration[2] = msg->linear_acceleration.z;

  double x = imu_orientation[0], y = imu_orientation[1], z = imu_orientation[2], w = imu_orientation[3];
  ahrs_yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  // if (mode_rc) // rc 모드일 경우 goal velocity를 RC컨트롤러 값으로 업데이트
  //   {
  //       goal_velocity[LINEAR] = goal_velocity_from_rc[LINEAR];
  //       goal_velocity[ANGULAR] = goal_velocity_from_rc[ANGULAR];
  //   }
  //   else // 자율주행 모드일 경우 cmd 데이터로 부터 goal velocity 업데이트
  //   {
        goal_velocity[LINEAR] = goal_velocity_from_cmd[LINEAR];
        goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
    // }

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

  odom_msg->pose.pose.orientation.x = imu_orientation[0];
  odom_msg->pose.pose.orientation.y = imu_orientation[1];
  odom_msg->pose.pose.orientation.z = imu_orientation[2];
  odom_msg->pose.pose.orientation.w = imu_orientation[3];

  odom_msg->twist.twist.linear.x  = current_linear_velocity;
  odom_msg->twist.twist.angular.z = imu_angular_velocity[2];

  for (int i = 0; i < 9; ++i) {
    odom_msg->twist.covariance[i] = imu_angular_covariance[i];
  }
}

/*******************************************************************************
* Update the tf 
*******************************************************************************/
void updateTF()
{
  tf_msg->transforms.data[0].transform.translation.x = odom_pose[0];
  tf_msg->transforms.data[0].transform.translation.y = odom_pose[1];
  tf_msg->transforms.data[0].transform.translation.z = 0;

  tf_msg->transforms.data[0].transform.rotation.x    = imu_orientation[0];
  tf_msg->transforms.data[0].transform.rotation.y    = imu_orientation[1];
  tf_msg->transforms.data[0].transform.rotation.z    = imu_orientation[2];
  tf_msg->transforms.data[0].transform.rotation.w    = imu_orientation[3];
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  if (diff_time == 0)
    return false;
  
  // compute odometric pose
  odom_pose[0] += current_linear_velocity * cos(ahrs_yaw) * diff_time;
  odom_pose[1] += current_linear_velocity * sin(ahrs_yaw) * diff_time;
  odom_pose[2] = ahrs_yaw;

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
  unsigned long timeDiff = (currentTime - lastTimeA) / 1.0e6;;  // 두 이벤트 간 시간 차 계산
  lastTimeA = currentTime;
  
  // Store deltaT in the buffer
  deltaT_buffer[buffer_index] = timeDiff;
  buffer_index = (buffer_index + 1) % FILTER_WINDOW_SIZE;

  // Update the sample count (capped at the buffer size)
  if (sample_count < FILTER_WINDOW_SIZE) {
    sample_count++;
  }
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
// ros::Time rosNow() { }