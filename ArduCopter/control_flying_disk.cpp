#include "Copter.h"
#include "defines.h"

/*
 * Init and run calls for Gamera flight mode
 */

float target_yaw_rate;
int16_t dif_ctrl_in, trim_yaw;
float sin_yaw_org, cos_yaw_org, asin_sin_yaw, acos_cos_yaw, yaw_heading;

// gamera_init - initialise Gamera controller
bool Copter::flying_disk_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    //show message when Gamera mode was called
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_DEBUG, "GAMERA MODE!");

    target_yaw_rate = 0;
    set_simple_mode(1);
    trim_yaw = channel_yaw->get_control_in();
    sin_yaw_org = simple_sin_yaw;
    cos_yaw_org = simple_cos_yaw;
    //initial_armed_bearing = 0;
    //simple_cos_yaw = 1;;
    //simple_sin_yaw = 0;
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::flying_disk_run()
{
    float target_roll, target_pitch;
    //float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // change yaw rate (and rotation speed of flying saurcer)
    if (read_3pos_switch(CH_11)==AUX_SWITCH_HIGH)
        //target_yaw_rate += 0.001*get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        target_yaw_rate += 10;
    if (read_3pos_switch(CH_11)==AUX_SWITCH_MIDDLE)
        //target_yaw_rate -= 0.001*get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        target_yaw_rate -= 10;
    if (read_3pos_switch(CH_11)==AUX_SWITCH_LOW){
        target_yaw_rate = 0;
        simple_sin_yaw = sin_yaw_org;
        simple_cos_yaw = cos_yaw_org;
    }
    // limitation of target_yaw_rate
    if(fabs(target_yaw_rate) > 10000) target_yaw_rate = (int)(target_yaw_rate)/(int)(fabs(target_yaw_rate))*10000;

    // calc yaw angle[deg] from sin_yaw and cos_yaw
    acos_cos_yaw = acos(simple_cos_yaw);
    asin_sin_yaw = asin(simple_sin_yaw);
    if (simple_sin_yaw>=0) yaw_heading = acos_cos_yaw/(4*atan(1.0))*180;
    else if (simple_cos_yaw<=0) yaw_heading = 180 - asin_sin_yaw/(4*atan(1.0))*180;
    else yaw_heading = 360 - acos_cos_yaw/(4*atan(1.0))*180;
    // read yaw-input from controller
    dif_ctrl_in = channel_yaw->get_control_in() - trim_yaw;
    // change yaw angle of heading
    yaw_heading += (double)dif_ctrl_in * 0.00002;
    simple_cos_yaw = cos(yaw_heading/180*(4*atan(1.0)));
    simple_sin_yaw = sin(yaw_heading/180*(4*atan(1.0)));


    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
