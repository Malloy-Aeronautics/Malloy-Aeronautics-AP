#include "Copter.h"

/*
  Ground speed limit (SPD_LIM_*): in STABILIZE and ALT_HOLD, going over the limit sends the
  vehicle home. The other modes fly their own WPNAV/LOIT/RTL speeds and are not checked. Off
  while there's no usable GPS. Runs at 10Hz from the scheduler.
*/
void Copter::speed_limit_check()
{
    const Mode::Number mode = flightmode->mode_number();
    const bool pilot_mode = mode == Mode::Number::STABILIZE || mode == Mode::Number::ALT_HOLD;

    // nothing to check on the ground or while the motors are still spooling up
    const bool flying = motors->armed() && !ap.land_complete &&
                        motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED;

    // the speed is only trusted while the EKF is navigating on GPS and at least one GPS that is
    // still sending sees enough satellites. With two GPS fitted, losing one isn't a GPS loss
    const uint32_t now_ms = AP_HAL::millis();
    uint8_t sats = 0;
    for (uint8_t i = 0; i < gps.num_sensors(); i++) {
        if (now_ms - gps.last_message_time_ms(i) < AC_SpeedLimit::GPS_TIMEOUT_MS) {
            sats = MAX(sats, gps.num_sats(i));
        }
    }
    nav_filter_status filt_status {};
    const bool using_gps = ahrs.get_filter_status(filt_status) && filt_status.flags.using_gps;
    const bool gps_usable = using_gps && sats >= AC_SpeedLimit::GPS_MIN_SATS;
    switch (g2.speed_limit.update_gps(gps_usable, now_ms)) {
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

    const bool applies = pilot_mode && flying && have_speed;
    const bool trigger = g2.speed_limit.update(applies, ground_speed);
    if (trigger) {
        // RTL only. If RTL is refused (no home, no position) the vehicle stays in the pilot's mode
        // rather than landing wherever it is
        if (set_mode(Mode::Number::RTL, ModeReason::SPEED_LIMIT)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Speed limit: %.2f m/s over %.2f, RTL",
                            (double)ground_speed, (double)g2.speed_limit.limit());
        } else {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Speed limit: %.2f m/s over %.2f, RTL refused",
                            (double)ground_speed, (double)g2.speed_limit.limit());
        }
    }

    if (!g2.speed_limit.enabled()) {
        return;
    }
// @LoggerMessage: SPDL
// @Description: Ground speed limit, written at 10Hz while SPD_LIM_EN is set. The RTL itself is in MODE (reason 51)
// @Field: TimeUS: Time since system startup
// @Field: Spd: horizontal ground speed from the EKF
// @Field: Lim: speed limit (SPD_LIM_VAL)
// @Field: App: 1 if the limit applies right now (STABILIZE or ALT_HOLD, flying, EKF speed available)
// @Field: Ovr: 1 if the pilot override switch has turned the limit off
// @Field: EKF: 1 if the EKF is using GPS
// @Field: NSat: most satellites seen by any GPS that is still sending
// @Field: On: 1 if the limit is live (not overridden, GPS not lost or still recovering)
// @Field: Trig: 1 on the update that asked for the RTL
    AP::logger().Write("SPDL", "TimeUS,Spd,Lim,App,Ovr,EKF,NSat,On,Trig", "snn------", "F00------", "QffBBBBBB",
                       AP_HAL::micros64(),
                       (double)ground_speed,
                       (double)g2.speed_limit.limit(),
                       (uint8_t)applies,
                       (uint8_t)g2.speed_limit.get_override(),
                       (uint8_t)using_gps,
                       sats,
                       (uint8_t)(g2.speed_limit.active() && g2.speed_limit.gps_ok()),
                       (uint8_t)trigger);
}
