#include "Copter.h"

/*
  Ground speed limit (SPD_LIM_*): in STABILIZE and ALT_HOLD, going over the limit sends the
  vehicle home. The other modes fly their own WPNAV/LOIT/RTL speeds and are not checked. Off
  while there's no usable GPS. Runs at 10Hz from the scheduler.
*/

void Copter::speed_limit_check()
{
    const uint32_t now_ms = AP_HAL::millis();
    speed_limit_update_gps(now_ms);

    if (motors->armed() && !speed_limit_was_armed) {
        speed_limit_warn_speeds();
    }
    speed_limit_was_armed = motors->armed();

    const Mode::Number mode = flightmode->mode_number();
    const bool pilot_mode = mode == Mode::Number::STABILIZE || mode == Mode::Number::ALT_HOLD;

    // nothing to check on the ground or while the motors are still spooling up
    const bool flying = motors->armed() && !ap.land_complete &&
                        motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED;

    Vector3f vel_ned;
    const bool have_speed = ahrs.get_velocity_NED(vel_ned);
    const float ground_speed = have_speed ? vel_ned.xy().length() : 0.0f;

    // the speed is checked in the pilot modes while flying
    const bool should_check = pilot_mode && flying && have_speed;
    // on the way home after an overspeed: keep telling the pilot until it has slowed down
    const bool returning = mode == Mode::Number::RTL && control_mode_reason == ModeReason::SPEED_LIMIT &&
                           flying && have_speed;

    const AC_SpeedLimit::Notice notice = g2.speed_limit.notice(should_check, returning, ground_speed, now_ms);
    const bool trigger = g2.speed_limit.update(should_check, ground_speed, now_ms);
    speed_limit_notify(trigger, notice, ground_speed, now_ms);
    speed_limit_log(should_check, ground_speed, trigger, notice);
}

/*
  The ground speed is only trusted while the EKF is navigating on GPS with an absolute position
  (not dead reckoning or relative only) and the receivers pass speed_limit_gps's checks. Tells
  the pilot when that turns the limit off or back on.
*/
void Copter::speed_limit_update_gps(uint32_t now_ms)
{
    nav_filter_status filt_status {};
    const bool ekf_ok = ahrs.get_filter_status(filt_status) && filt_status.flags.using_gps &&
                        ekf_has_absolute_position();
    speed_limit_gps.update(gps, ekf_ok, now_ms);
    g2.speed_limit.set_gps_ok(speed_limit_gps.ok(), speed_limit_gps.recovering());

    switch (g2.speed_limit.gps_notice(now_ms)) {
    case AC_SpeedLimit::GpsNotice::INACTIVE:
        gcs().send_text(MAV_SEVERITY_NOTICE, "Speed limit inactive: no GPS");
        break;
    case AC_SpeedLimit::GpsNotice::ACTIVE:
        gcs().send_text(MAV_SEVERITY_NOTICE, "Speed limit active");
        break;
    case AC_SpeedLimit::GpsNotice::NONE:
        break;
    }
}

/*
  Acts on what the limit decided this update: the RTL when it asked for one, otherwise the
  message for the pilot, if any.
*/
void Copter::speed_limit_notify(bool trigger, AC_SpeedLimit::Notice notice, float ground_speed, uint32_t now_ms)
{
    const float limit = g2.speed_limit.limit();
    if (trigger) {
        // RTL only. If RTL is refused (no home, no position) the vehicle stays in the pilot's mode
        // rather than landing wherever it is, and the RTL is asked for again every second while it
        // is still over the limit
        const bool rtl = set_mode(Mode::Number::RTL, ModeReason::SPEED_LIMIT);
        gcs().send_text(MAV_SEVERITY_ALERT, "Speed limit: %.2f m/s over %.2f, %s",
                        (double)ground_speed, (double)limit, rtl ? "RTL" : "RTL refused");
        return;
    }

    switch (notice) {
    case AC_SpeedLimit::Notice::OVER:
        gcs().send_text(MAV_SEVERITY_ALERT, "Speed over limit: %.2f m/s, limit %.2f",
                        (double)ground_speed, (double)limit);
        break;
    case AC_SpeedLimit::Notice::WARN:
        gcs().send_text(MAV_SEVERITY_WARNING, "Speed warning: %.2f m/s, limit %.2f",
                        (double)ground_speed, (double)limit);
        break;
    case AC_SpeedLimit::Notice::GPS_BACK: {
        const uint32_t secs = (speed_limit_gps.ms_until_ok(now_ms) + 999) / 1000;
        gcs().send_text(MAV_SEVERITY_WARNING, "GPS back: slow down, speed limit active in %us", (unsigned)secs);
        break;
    }
    case AC_SpeedLimit::Notice::NONE:
        break;
    }
}

/*
  On arming, with the limit on: RTL, LOITER and the auto modes fly their own speeds and aren't
  checked, so the pilot is warned about any set faster than the limit. RTL is where the limit sends
  the vehicle; at RTL_SPEED 0 it flies at WPNAV_SPEED, which the WPNAV_SPEED warning covers.
*/
void Copter::speed_limit_warn_speeds()
{
    if (!g2.speed_limit.enabled()) {
        return;
    }
    const float limit = g2.speed_limit.limit();
    const struct {
        const char *name;
        float speed;    // m/s
    } speeds[] {
        { "RTL_SPEED", g.rtl_speed_cms * 0.01f },
        { "WPNAV_SPEED", wp_nav->get_default_speed_xy() * 0.01f },
        { "LOIT_SPEED", loiter_nav->get_speed_max_cms() * 0.01f },
    };
    for (const auto &s : speeds) {
        if (s.speed > limit) {
            gcs().send_text(MAV_SEVERITY_WARNING, "%s %.1f m/s over speed limit %.1f",
                            s.name, (double)s.speed, (double)limit);
        }
    }
}

void Copter::speed_limit_log(bool should_check, float ground_speed, bool trigger, AC_SpeedLimit::Notice notice)
{
    if (!g2.speed_limit.enabled()) {
        return;
    }
    const AP_GPSCheck::Status &gps_status = speed_limit_gps.status();
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
// @Field: Ntc: message sent to the ground station this update: 0 none, 1 warning, 2 over the limit, 3 GPS back while over the limit
    AP::logger().Write("SPDL", "TimeUS,Spd,Lim,Chk,Ovr,EKF,NSat,HAcc,SAcc,On,Trig,Ntc", "snn----mn---", "F00----00---", "QffBBBBffBBB",
                       AP_HAL::micros64(),
                       (double)ground_speed,
                       (double)g2.speed_limit.limit(),
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

/*
  The pilot switch (RC option 199), or a radio failsafe putting the limit back on. Says what the
  limit is doing now, so the pilot isn't told it's active while it's still waiting for the GPS.
*/
void Copter::speed_limit_set_override(bool override_on, const char *why)
{
    AC_SpeedLimit &speed_limit = g2.speed_limit;
    speed_limit.set_override(override_on);
    if (!speed_limit.enabled()) {
        return;
    }
    if (override_on) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Speed limit inactive: %s", why);
    } else if (speed_limit.gps_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Speed limit active: %s", why);
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Speed limit waiting for GPS: %s", why);
    }
}
