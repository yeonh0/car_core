#ifndef CAR_INFO_H_
#define CAR_INFO_H_

#define NAME                             "MYCAR"

#define WHEEL_RADIUS                     0.028           // meter (2.2'' / 2 ~= 28mm)
#define WHEEL_SEPARATION                 0.281           // meter (11'' ~= 281mm)
#define TURNING_RADIUS                   0.080           // meter - 계산 필요
#define ROBOT_RADIUS                     0.268           // meter (차량 길이 = 21'' -> 21''/2 ~= 267.5mm)
#define DRIVE_RATIO                      19.69           // 차량 기어비

// #define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_LINEAR_VELOCITY              1.5
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#define MIN_ANGLE                        -60.0          // 최소 각도
#define MAX_ANGLE                        60.0           // 최대 각도
#define MIN_DUTY                         53             // 최소 duty cycle (%)
#define MAX_DUTY                         98             // 최대 duty cycle (%)
#define MID_DUTY                         0              // 0도에 해당하는 duty cycle (%)

#endif  //CAR_INFO_H_