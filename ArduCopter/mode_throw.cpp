#include "Copter.h"

#if MODE_THROW_ENABLED == ENABLED

// throw_init - initialise throw controller
bool ModeThrow::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to use throw to start
    return false;
#endif

    // do not enter the mode when already armed or when flying
    if (motors->armed()) {
        return false;
    }

    // init state
    stage = Throw_Disarmed;
    nextmode_attempted = false;
    detect_start_ms = 0;
    upright_start_ms = 0;
    height_start_ms = 0;

    // initialise pos controller speed and acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    return true;
}

// runs the throw to start controller
// should be called at 100hz or more
void ModeThrow::run()
{
    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    uint32_t now = AP_HAL::millis();

    if (!motors->armed()) {
        // state machine entry is always from a disarmed state
        stage = Throw_Disarmed;

    } else if (stage == Throw_Disarmed && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO,"waiting for throw");
        stage = Throw_Detecting;

    } else if (stage == Throw_Detecting && throw_detected()){
        gcs().send_text(MAV_SEVERITY_INFO,"throw detected - spooling motors");
        copter.set_land_complete(false);
        stage = Throw_Wait_Throttle_Unlimited;

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        gcs().send_text(MAV_SEVERITY_INFO,"throttle is unlimited - uprighting");
        stage = Throw_Uprighting;
        upright_start_ms = now;
    } else if (stage == Throw_Uprighting && throw_attitude_good() && ((now - upright_start_ms) > (uint32_t)g2.lb_upright_ms.get())) {
        gcs().send_text(MAV_SEVERITY_INFO,"uprighted - controlling height");
        stage = Throw_HgtStabilise;
        height_start_ms = now;

        // initialise the z controller
        pos_control->init_z_controller_no_descent();

        // initialise the demanded height to 3m above the throw height
        // we want to rapidly clear surrounding obstacles
        if (g2.throw_type == ThrowType::Drop) {
            pos_control->set_pos_target_z_cm(inertial_nav.get_position_z_up_cm() - 100);
        } else {
            pos_control->set_pos_target_z_cm(inertial_nav.get_position_z_up_cm() + 300);
        }

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);

    } else if (stage == Throw_HgtStabilise && throw_height_good() && ((now - height_start_ms) > (uint32_t)g2.lb_height_ms.get())) {
        gcs().send_text(MAV_SEVERITY_INFO,"height achieved - controlling position");
        stage = Throw_PosHold;

        // initialise position controller
        pos_control->init_xy_controller();

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);
    } else if (stage == Throw_PosHold && throw_position_good()) {
        if (!nextmode_attempted) {
            switch ((Mode::Number)g2.throw_nextmode.get()) {
                case Mode::Number::ALT_HOLD:
                case Mode::Number::AUTO:
                case Mode::Number::GUIDED:
                case Mode::Number::RTL:
                case Mode::Number::LAND:
                case Mode::Number::BRAKE:
                case Mode::Number::LOITER:
                    set_mode((Mode::Number)g2.throw_nextmode.get(), ModeReason::THROW_COMPLETE);
                    break;
                default:
                    // do nothing
                    break;
            }
        }
    }

    // Throw State Processing
    switch (stage) {

    case Throw_Disarmed:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pos_control->standby_xyz_reset();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);

        copter.arming.arm(AP_Arming::Method::UNKNOWN,false);

        break;

    case Throw_Detecting:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // Hold throttle at zero during the throw and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pos_control->standby_xyz_reset();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);

        // Play the waiting for throw tone sequence to alert the user
        AP_Notify::flags.waiting_for_throw = true;

        break;

    case Throw_Wait_Throttle_Unlimited:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        break;

    case Throw_Uprighting:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // demand a level roll/pitch attitude with zero yaw rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // output 50% throttle and turn off angle boost to maximise righting moment
        attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);

        break;

    case Throw_HgtStabilise:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // call height controller
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();

        break;

    case Throw_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // use position controller to stop
        Vector2f vel;
        Vector2f accel;
        pos_control->input_vel_accel_xy(vel, accel);
        pos_control->update_xy_controller();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

        // call height controller
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();

        break;
    }

    // log at 10hz or if stage changes
    now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 100) {
        prev_stage = stage;
        last_log_ms = now;
        const float velocity = inertial_nav.get_velocity_neu_cms().length();
        const float velocity_z = inertial_nav.get_velocity_z_up_cms();
        const float accel = copter.ins.get_accel().length();
        const float ef_accel_z = ahrs.get_accel_ef().z;
        const bool throw_detect = (stage > Throw_Detecting) || throw_detected();
        const bool attitude_ok = (stage > Throw_Uprighting) || throw_attitude_good();
        const bool height_ok = (stage > Throw_HgtStabilise) || throw_height_good();
        const bool pos_ok = (stage > Throw_PosHold) || throw_position_good();
        
// @LoggerMessage: THRO
// @Description: Throw Mode messages
// @URL: https://ardupilot.org/copter/docs/throw-mode.html
// @Field: TimeUS: Time since system startup
// @Field: Stage: Current stage of the Throw Mode
// @Field: Vel: Magnitude of the velocity vector
// @Field: VelZ: Vertical Velocity
// @Field: Acc: Magnitude of the vector of the current acceleration
// @Field: AccEfZ: Vertical earth frame accelerometer value
// @Field: Throw: True if a throw has been detected since entering this mode
// @Field: AttOk: True if the vehicle is upright 
// @Field: HgtOk: True if the vehicle is within 50cm of the demanded height
// @Field: PosOk: True if the vehicle is within 50cm of the demanded horizontal position
        
        AP::logger().WriteStreaming(
            "THRO",
            "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk,HgtOk,PosOk",
            "s-nnoo----",
            "F-0000----",
            "QBffffbbbb",
            AP_HAL::micros64(),
            (uint8_t)stage,
            (double)velocity,
            (double)velocity_z,
            (double)accel,
            (double)ef_accel_z,
            throw_detect,
            attitude_ok,
            height_ok,
            pos_ok);
    }
}

bool ModeThrow::throw_detected()
{
    uint32_t now = AP_HAL::millis();

    if(detect_start_ms < 1 && stage == Throw_Detecting) {  // The timer has not been set
        if(!copter.button.get_button_state(1)) // Simple limit switch check
            detect_start_ms = now;
        return false;
    }
    
    // The timer was set before, Return if the timer has expired
    return now > detect_start_ms + (uint32_t)g2.lb_detect_ms.get();
}

bool ModeThrow::throw_attitude_good()
{
    // Check that we have uprighted the copter
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    if (rotMat.c.z > 0.866f)
        return true; // is_upright
    upright_start_ms = AP_HAL::millis();
    return false;
}

bool ModeThrow::throw_height_good() const
{
    // Check that we are within 0.5m of the demanded height
    return (pos_control->get_pos_error_z_cm() < 50.0f);
}

bool ModeThrow::throw_position_good() const
{
    // check that our horizontal position error is within 50cm
    return (pos_control->get_pos_error_xy_cm() < 50.0f);
}

#endif
