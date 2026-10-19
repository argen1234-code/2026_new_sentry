#ifndef _GRAVITY_H
#define _GRAVITY_H

typedef struct {
    float k;             // 弹道系数（空气阻力系数）
    float current_v;     // 当前弹速（m/s）
    float s_bias;        // 枪口水平偏移修正（m）
    float z_bias;        // yaw轴到枪口水平面的垂直修正（m）
} BallisticParams;

#define GRAVITY         9.80665f    // 使用更精确的重力加速度
#define AIR_DENSITY     1.225f      // 空气密度（kg/m3）
#define PROJECTILE_MASS 0.0032f      // 弹丸质量（kg）
#define Initial_velocity 16.2

extern BallisticParams params;
void init_BallisticParams(BallisticParams* params);
float calculateCompensationAngle(float shoot_angle, float distance, float initial_velocity, BallisticParams params);
#endif
