#include "Copter.h"

/*
  Ground speed limit: in STABILIZE and ALT_HOLD, going over the limit sends the
  vehicle home. The other modes fly their own WPNAV/LOIT/RTL speeds and are not checked. Off
  while the EKF isn't using GPS.
*/
void Copter::speed_limit_check()
{
    const Mode::Number mode = flightmode->mode_number();
    const bool pilot_mode = mode == Mode::Number::STABILIZE || mode == Mode::Number::ALT_HOLD;

    // nothing to check on the ground or while the motors are still spooling up
    const bool flying = motors->armed() && !ap.land_complete &&
                        motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED;

    // the speed is only trusted while the EKF is navigating on GPS. It stops within a few seconds
    // of losing the GPS, sooner than it gives up on the velocity it's dead reckoning
    nav_filter_status filt_status {};
    const bool using_gps = ahrs.get_filter_status(filt_status) && filt_status.flags.using_gps;
    switch (g2.speed_limit.update_gps(using_gps, AP_HAL::millis())) {
    case -1:
        if (g2.speed_limit.enabled()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Speed limit off: no GPS");
        }
        break;
    case 1:
        if (g2.speed_limit.enabled()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Speed limit on: GPS back");
        }
        break;
    default:
        break;
    }

    Vector3f vel_ned;
    const bool have_speed = ahrs.get_velocity_NED(vel_ned);
    const float ground_speed = have_speed ? vel_ned.xy().length() : 0.0f;

    if (g2.speed_limit.update(pilot_mode && flying && have_speed, ground_speed)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Speed limit: %.2f m/s over %.2f, RTL",
                        (double)ground_speed, (double)g2.speed_limit.limit());
        set_mode_RTL_or_land_with_pause(ModeReason::SPEED_LIMIT);
    }
}
