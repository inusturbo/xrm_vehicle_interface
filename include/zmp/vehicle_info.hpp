#ifndef VEHCILE_INFO_HPP
#define VEHCILE_INFO_HPP

#define ZMP_LOOP_RATE 50

#define WHEEL_BASE 2.7                                           // tire-to-tire size of Prius.//车轮轴距的大小
#define WHEEL_ANGLE_MAX 31.28067                                 // max angle of front tires. //前轮转向角度的最大值
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX / WHEEL_ANGLE_MAX) // 转向角度与前轮转向角度的比值
#define STEERING_ANGLE_MAX 666                                   // max angle of steering  转向角度的最大值

#endif // VEHCILE_INFO_HPP