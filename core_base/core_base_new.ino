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
  RCCHECK(rclc_publisher_init_default(&linx_echo_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "linx"));

  /*******************************************************************************
  * Subscriber
  *******************************************************************************/
  RCCHECK(rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));  // Create executor
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_sub_msg, &commandVelocityCallback, ON_NEW_DATA));

  /*******************************************************************************
  * Setting for SLAM and Navigation
  *******************************************************************************/

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
  // Compute velocity using buffered deltaT value

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
  // updateTime();

  /* motor contorol section : tTime[0] - MOTOR SIGNAL SEQUNCE */
  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();     // 특정 주기마다 goal velocity 업데이트

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

    double my_vel = (wheel_rpm * 2.0 * M_PI * WHEEL_RADIUS) / 60.0;


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
    linx_echo_msg.data  = current_linear_velocity;

    RCSOFTCHECK(rcl_publish(&linx_echo_pub, &linx_echo_msg, NULL));
  }

  /* publish odom, tf section - 특정 주기마다 odom, tf 데이터 퍼블리시 : tTime[2] - ODOM, TF MSG SIGNAL SEQUENCE*/
  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
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


/*******************************************************************************
* Publish msgs (odometry, tf)
*******************************************************************************/


/*******************************************************************************
* Update the odometry
*******************************************************************************/


/*******************************************************************************
* Update the tf 
*******************************************************************************/


/*******************************************************************************
* Calculate the odometry
*******************************************************************************/

/*******************************************************************************
* Initialize message's frame id
*******************************************************************************/

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/


/*******************************************************************************
* Hall Sensor ISR
*******************************************************************************/
void hallSeonsor_ISR(void)
{
  long currentTime = micros();  // 현재 시간(us) 단위로 읽기
  float timeDiff = ((float)currentTime - lastTimeA) / 1.0e6;;  // 두 이벤트 간 시간 차 계산
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