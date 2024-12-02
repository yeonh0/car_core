#ifndef CAR_INFO_H_
#define CAR_INFO_H_

#define NAME                             "MYCAR"

#define WHEEL_RADIUS                     0.028           // meter (2.2'' / 2 ~= 28mm)
#define WHEEL_SEPARATION                 0.281           // meter (11'' ~= 281mm)
#define TURNING_RADIUS                   0.080           // meter - 계산 필요
#define ROBOT_RADIUS                     0.268           // meter (차량 길이 = 21'' -> 21''/2 ~= 267.5mm)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#endif  //CAR_INFO_H_