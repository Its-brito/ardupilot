#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include "AC_CustomControl_INDI.h"
#include "AC_AttitudeControl/AC_AttitudeControl_Multi.h"

const AP_Param::GroupInfo AC_CustomControl_INDI::var_info[] = {
    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain. Converts roll angle error to target roll rate.
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", 1, AC_CustomControl_INDI, AC_P),

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain. Converts pitch angle error to target pitch rate.
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 2, AC_CustomControl_INDI, AC_P),

    // @Param: ANG_YAW_P
    // @DisplayName: Yaw axis angle controller P gain
    // @Description: Yaw axis angle controller P gain. Converts yaw angle error to target yaw rate.
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 6.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_yaw, "ANG_YAW_", 3, AC_CustomControl_INDI, AC_P),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis INDI rate P gain
    // @Description: Roll axis proportional gain from body rate error to desired angular acceleration.
    // @Range: 0.100 20.000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RAT_RLL_P", 4, AC_CustomControl_INDI, _k_rate_p_roll, 8.0f),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis INDI rate P gain
    // @Description: Pitch axis proportional gain from body rate error to desired angular acceleration.
    // @Range: 0.100 20.000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RAT_PIT_P", 5, AC_CustomControl_INDI, _k_rate_p_pitch, 8.0f),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis INDI rate P gain
    // @Description: Yaw axis proportional gain from body rate error to desired angular acceleration.
    // @Range: 0.100 20.000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RAT_YAW_P", 6, AC_CustomControl_INDI, _k_rate_p_yaw, 4.0f),

    // @Param: G_RLL
    // @DisplayName: Roll effectiveness gain
    // @Description: Roll axis control effectiveness estimate used to convert desired acceleration to control increment.
    // @Range: 0.100 100.000
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("G_RLL", 7, AC_CustomControl_INDI, _g_roll, 20.0f),

    // @Param: G_PIT
    // @DisplayName: Pitch effectiveness gain
    // @Description: Pitch axis control effectiveness estimate used to convert desired acceleration to control increment.
    // @Range: 0.100 100.000
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("G_PIT", 8, AC_CustomControl_INDI, _g_pitch, 20.0f),

    // @Param: G_YAW
    // @DisplayName: Yaw effectiveness gain
    // @Description: Yaw axis control effectiveness estimate used to convert desired acceleration to control increment.
    // @Range: 0.100 100.000
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("G_YAW", 9, AC_CustomControl_INDI, _g_yaw, 10.0f),

    // @Param: ACC_FLT
    // @DisplayName: Angular acceleration filter cutoff
    // @Description: Cutoff frequency of the measured angular acceleration low-pass filter.
    // @Range: 1.0 100.0
    // @Increment: 1
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("ACC_FLT", 10, AC_CustomControl_INDI, _accel_filter_hz, 15.0f),

    AP_GROUPEND
};

AC_CustomControl_INDI::AC_CustomControl_INDI(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _p_angle_roll(AC_ATTITUDE_CONTROL_ANGLE_P),
    _p_angle_pitch(AC_ATTITUDE_CONTROL_ANGLE_P),
    _p_angle_yaw(AC_ATTITUDE_CONTROL_ANGLE_P)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);
    _ang_accel_filter.set_cutoff_frequency(_accel_filter_hz.get());
}

Vector3f AC_CustomControl_INDI::update(void)
{
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            reset();
            return Vector3f{0.0f, 0.0f, 0.0f};
        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            break;
    }

    _ang_accel_filter.set_cutoff_frequency(_accel_filter_hz.get());

    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);
    attitude_target = _att_control->get_attitude_target_quat();

    Vector3f attitude_error;
    float thrust_angle_rad;
    float thrust_error_angle_rad;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, thrust_angle_rad, thrust_error_angle_rad);

    Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();

    Vector3f target_rate;
    target_rate.x = _p_angle_roll.kP() * attitude_error.x + ang_vel_body_feedforward.x;
    target_rate.y = _p_angle_pitch.kP() * attitude_error.y + ang_vel_body_feedforward.y;
    target_rate.z = _p_angle_yaw.kP() * attitude_error.z + ang_vel_body_feedforward.z;

    Vector3f current_gyro = _ahrs->get_gyro_latest();
    Vector3f ang_accel_raw = (current_gyro - _last_gyro) / _dt;

    Vector3f ang_accel_meas = _ang_accel_filter.apply(ang_accel_raw, _dt);
    _last_gyro = current_gyro;

    Vector3f rate_error = target_rate - current_gyro;

    Vector3f ang_accel_des;
    ang_accel_des.x = rate_error.x * _k_rate_p_roll.get();
    ang_accel_des.y = rate_error.y * _k_rate_p_pitch.get();
    ang_accel_des.z = rate_error.z * _k_rate_p_yaw.get();

    Vector3f delta_u;
    float g_r = is_zero(_g_roll.get()) ? 1.0f : _g_roll.get();
    float g_p = is_zero(_g_pitch.get()) ? 1.0f : _g_pitch.get();
    float g_y = is_zero(_g_yaw.get()) ? 1.0f : _g_yaw.get();

    delta_u.x = (ang_accel_des.x - ang_accel_meas.x) / g_r;
    delta_u.y = (ang_accel_des.y - ang_accel_meas.y) / g_p;
    delta_u.z = (ang_accel_des.z - ang_accel_meas.z) / g_y;

    Vector3f motor_out = _last_motor_out + delta_u;
    motor_out.x = constrain_float(motor_out.x, -1.0f, 1.0f);
    motor_out.y = constrain_float(motor_out.y, -1.0f, 1.0f);
    motor_out.z = constrain_float(motor_out.z, -1.0f, 1.0f);

    _last_motor_out = motor_out;

    return motor_out;
}

void AC_CustomControl_INDI::reset(void)
{
    _last_gyro = _ahrs->get_gyro_latest();
    _last_motor_out.zero();
    _ang_accel_filter.reset(Vector3f{0.0f, 0.0f, 0.0f});
}

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
