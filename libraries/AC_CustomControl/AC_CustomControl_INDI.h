#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_P.h>
#include <Filter/LowPassFilter.h>

#include "AC_CustomControl_Backend.h"

class AC_CustomControl_INDI : public AC_CustomControl_Backend {
public:
    AC_CustomControl_INDI(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);

    Vector3f update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    float _dt;

    // Attitude (Angle) Loop Controllers
    AC_P _p_angle_roll;
    AC_P _p_angle_pitch;
    AC_P _p_angle_yaw;

    // INDI Rate P Gains
    AP_Float _k_rate_p_roll;
    AP_Float _k_rate_p_pitch;
    AP_Float _k_rate_p_yaw;

    // INDI Control Effectiveness (G)
    AP_Float _g_roll;
    AP_Float _g_pitch;
    AP_Float _g_yaw;

    // Filter for the differentiated gyro
    AP_Float _accel_filter_hz;

    // INDI Internal States
    Vector3f _last_gyro;
    Vector3f _last_motor_out;
    LowPassFilterVector3f _ang_accel_filter;
};

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
