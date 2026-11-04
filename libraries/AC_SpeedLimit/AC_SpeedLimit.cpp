#include "AC_SpeedLimit.h"

#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AC_SpeedLimit::var_info[] = {

    // @Param: EN
    // @DisplayName: Speed limit enable
    // @Description: Switches to RTL as soon as the horizontal ground speed goes over SPD_LIM_VAL in STABILIZE or ALT_HOLD. The other flight modes fly their own speed settings and are not checked.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("EN", 1, AC_SpeedLimit, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: VAL
    // @DisplayName: Speed limit
    // @Description: Horizontal ground speed above which the vehicle is sent home
    // @Units: m/s
    // @Range: 1 50
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("VAL", 2, AC_SpeedLimit, _limit, 20.0f),

    // index 3 was SPD_LIM_TIME in an earlier version, don't reuse it

    // @Param: WARN
    // @DisplayName: Speed limit warning
    // @Description: Sends a warning to the ground station when the horizontal ground speed gets within this percentage of SPD_LIM_VAL, in the modes the limit covers. 0 turns the warning off
    // @Units: %
    // @Range: 0 99
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("WARN", 4, AC_SpeedLimit, _warn_pct, 90),

    AP_GROUPEND
};

AC_SpeedLimit::AC_SpeedLimit()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_SpeedLimit::set_override(bool override_on)
{
    _override = override_on;
    _triggered = false;
}

int8_t AC_SpeedLimit::update_gps(bool gps_usable, uint32_t now_ms)
{
    if (!gps_usable) {
        _gps_back_ms = 0;
        if (!_gps_lost) {
            _gps_lost = true;
            _triggered = false;
            return -1;
        }
        return 0;
    }
    if (!_gps_lost) {
        return 0;
    }
    if (_gps_back_ms == 0) {
        _gps_back_ms = now_ms;
        return 0;
    }
    if (now_ms - _gps_back_ms < GPS_RECOVER_MS) {
        return 0;
    }
    _gps_lost = false;
    _gps_back_ms = 0;
    return 1;
}

bool AC_SpeedLimit::update(bool applies, float ground_speed)
{
    // a limit of zero or less would send the vehicle home as soon as it moved, treat it as off
    if (!active() || !applies || _gps_lost || !isfinite(ground_speed) || !is_positive(_limit.get()) ||
        ground_speed <= _limit) {
        _triggered = false;
        return false;
    }
    if (_triggered) {
        return false;
    }
    _triggered = true;
    return true;
}

float AC_SpeedLimit::warn_speed() const
{
    if (_warn_pct <= 0 || _warn_pct >= 100 || !is_positive(_limit.get())) {
        return 0.0f;
    }
    return _limit * _warn_pct * 0.01f;
}

AC_SpeedLimit::Notice AC_SpeedLimit::notice(bool applies, bool returning, float ground_speed, uint32_t now_ms)
{
    if (!active() || _gps_lost || !(applies || returning) || !isfinite(ground_speed) ||
        !is_positive(_limit.get())) {
        _over = false;
        _warned = false;
        return Notice::NONE;
    }

    // over the limit: straight away, then every OVER_NOTICE_MS for as long as it stays over
    if (ground_speed > _limit) {
        if (!_over || now_ms - _last_over_ms >= OVER_NOTICE_MS) {
            _over = true;
            _last_over_ms = now_ms;
            return Notice::OVER;
        }
        return Notice::NONE;
    }
    _over = false;

    // getting close, only in the modes the limit covers (not on the way home)
    const float warn = warn_speed();
    if (!applies || !is_positive(warn)) {
        _warned = false;
        return Notice::NONE;
    }
    if (ground_speed >= warn) {
        if (!_warned || now_ms - _last_warn_ms >= WARN_NOTICE_MS) {
            _warned = true;
            _last_warn_ms = now_ms;
            return Notice::WARN;
        }
        return Notice::NONE;
    }
    if (ground_speed < warn - WARN_HYSTERESIS * _limit) {
        _warned = false;
    }
    return Notice::NONE;
}
