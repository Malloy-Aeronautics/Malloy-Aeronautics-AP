#include "Copter.h"

/*
  Ground speed limit (SPD_LIM_*): in STABILIZE and ALT_HOLD, going over the limit sends the
  vehicle home. The other modes fly their own WPNAV/LOIT/RTL speeds and are not checked. Runs
  at 10Hz from the scheduler.
*/
void Copter::speed_limit_check()
{
    const Mode::Number mode = flightmode->mode_number();
    const bool pilot_mode = mode == Mode::Number::STABILIZE || mode == Mode::Number::ALT_HOLD;

    // nothing to check on the ground or while the motors are still spooling up
    const bool flying = motors->armed() && !ap.land_complete &&
                        motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED;

    Vector3f vel_ned;
    const bool have_speed = ahrs.get_velocity_NED(vel_ned);
    const float ground_speed = have_speed ? vel_ned.xy().length() : 0.0f;

    if (g2.speed_limit.update(pilot_mode && flying && have_speed, ground_speed)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Speed limit: %.2f m/s over %.2f, RTL",
                        (double)ground_speed, (double)g2.speed_limit.limit());
        set_mode_RTL_or_land_with_pause(ModeReason::SPEED_LIMIT);
    }
}
