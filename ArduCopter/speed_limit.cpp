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

    // the ground speed is only trusted while the EKF is navigating on GPS with an absolute position
    // (not dead reckoning or relative only) and the receivers pass speed_limit_gps's checks
    const uint32_t now_ms = AP_HAL::millis();
    nav_filter_status filt_status {};
    const bool ekf_ok = ahrs.get_filter_status(filt_status) && filt_status.flags.using_gps &&
                        ekf_has_absolute_position();
    speed_limit_gps.update(gps, ekf_ok, now_ms);
    const AP_GPSCheck::Status &gps_status = speed_limit_gps.status();
    const bool gps_was_ok = g2.speed_limit.gps_ok();
    g2.speed_limit.set_gps_ok(speed_limit_gps.ok());
    if (g2.speed_limit.enabled() && g2.speed_limit.gps_ok() != gps_was_ok) {
        gcs().send_text(MAV_SEVERITY_NOTICE, g2.speed_limit.gps_ok() ? "Speed limit on: GPS back" :
                                                                       "Speed limit off: no GPS");
    }

    Vector3f vel_ned;
    const bool have_speed = ahrs.get_velocity_NED(vel_ned);
    const float ground_speed = have_speed ? vel_ned.xy().length() : 0.0f;

    // the speed is checked in the pilot modes while flying
    const bool should_check = pilot_mode && flying && have_speed;
    // on the way home after an overspeed: keep telling the pilot until it has slowed down
    const bool returning = mode == Mode::Number::RTL && control_mode_reason == ModeReason::SPEED_LIMIT &&
                           flying && have_speed;
    const float limit = g2.speed_limit.limit();
    const AC_SpeedLimit::Notice notice = g2.speed_limit.notice(should_check, returning, ground_speed, now_ms);
    const bool trigger = g2.speed_limit.update(should_check, ground_speed, now_ms);
    if (trigger) {
        // RTL only. If RTL is refused (no home, no position) the vehicle stays in the pilot's mode
        // rather than landing wherever it is, and the RTL is asked for again every second while it
        // is still over the limit
        const bool rtl = set_mode(Mode::Number::RTL, ModeReason::SPEED_LIMIT);
        gcs().send_text(MAV_SEVERITY_ALERT, "Speed limit: %.2f m/s over %.2f, %s",
                        (double)ground_speed, (double)limit, rtl ? "RTL" : "RTL refused");
    } else if (notice == AC_SpeedLimit::Notice::OVER) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Speed over limit: %.1f m/s, limit %.1f",
                        (double)ground_speed, (double)limit);
    } else if (notice == AC_SpeedLimit::Notice::WARN) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Speed warning: %.1f m/s, limit %.1f",
                        (double)ground_speed, (double)limit);
    }

    if (!g2.speed_limit.enabled()) {
        return;
    }
// @LoggerMessage: SPDL
// @Description: Ground speed limit, written at 10Hz while SPD_LIM_EN is set. The RTL itself is in MODE (reason 51)
// @Field: TimeUS: Time since system startup
// @Field: Spd: horizontal ground speed from the EKF
// @Field: Lim: speed limit (SPD_LIM_VAL)
// @Field: Chk: 1 if the speed is checked right now (STABILIZE or ALT_HOLD, flying, EKF speed available)
// @Field: Ovr: 1 if the pilot override switch has turned the limit off
// @Field: EKF: 1 if the EKF is using GPS and has an absolute position
// @Field: NSat: most satellites on any GPS still sending with a 3D fix
// @Field: HAcc: best horizontal accuracy reported by those GPS, -1 if none reports one
// @Field: SAcc: best speed accuracy reported by those GPS, -1 if none reports one
// @Field: On: 1 if the limit is live (not overridden, GPS not lost or still recovering)
// @Field: Trig: 1 on an update that asked for the RTL
// @Field: Ntc: message sent to the ground station this update: 0 none, 1 warning, 2 over the limit
    AP::logger().Write("SPDL", "TimeUS,Spd,Lim,Chk,Ovr,EKF,NSat,HAcc,SAcc,On,Trig,Ntc", "snn----mn---", "F00----00---", "QffBBBBffBBB",
                       AP_HAL::micros64(),
                       (double)ground_speed,
                       (double)limit,
                       (uint8_t)should_check,
                       (uint8_t)g2.speed_limit.get_override(),
                       (uint8_t)gps_status.ekf_ok,
                       gps_status.sats,
                       (double)gps_status.hacc,
                       (double)gps_status.sacc,
                       (uint8_t)g2.speed_limit.is_active(),
                       (uint8_t)trigger,
                       (uint8_t)(trigger ? AC_SpeedLimit::Notice::OVER : notice));
}
