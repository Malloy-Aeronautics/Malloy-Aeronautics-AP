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

void AC_SpeedLimit::set_gps_ok(bool gps_ok)
{
    _gps_lost = !gps_ok;
    if (_gps_lost) {
        _triggered = false;
    }
}

bool AC_SpeedLimit::can_check(float ground_speed) const
{
    // a limit of zero or less would send the vehicle home as soon as it moved, treat it as off
    return is_active() && isfinite(ground_speed) && is_positive(_limit.get());
}

bool AC_SpeedLimit::due(bool &started, uint32_t &last_ms, uint32_t now_ms, uint32_t interval_ms)
{
    if (started && now_ms - last_ms < interval_ms) {
        return false;
    }
    started = true;
    last_ms = now_ms;
    return true;
}

bool AC_SpeedLimit::update(bool should_check, float ground_speed, uint32_t now_ms)
{
    if (!should_check || !can_check(ground_speed) || ground_speed <= _limit) {
        _triggered = false;
        return false;
    }
    // still checked and still over after asking: the RTL didn't happen, ask again
    return due(_triggered, _last_trigger_ms, now_ms, RTL_RETRY_MS);
}

float AC_SpeedLimit::warn_speed() const
{
    if (_warn_pct <= 0 || _warn_pct >= 100 || !is_positive(_limit.get())) {
        return 0.0f;
    }
    return _limit * _warn_pct * 0.01f;
}

AC_SpeedLimit::Notice AC_SpeedLimit::notice(bool should_check, bool returning, float ground_speed, uint32_t now_ms)
{
    if (!(should_check || returning) || !can_check(ground_speed)) {
        _over = false;
        _warned = false;
        return Notice::NONE;
    }

    // over the limit: straight away, then every OVER_NOTICE_MS for as long as it stays over
    if (ground_speed > _limit) {
        return due(_over, _last_over_ms, now_ms, OVER_NOTICE_MS) ? Notice::OVER : Notice::NONE;
    }
    _over = false;

    // getting close, only in the modes the limit covers (not on the way home)
    const float warn = warn_speed();
    if (!should_check || !is_positive(warn)) {
        _warned = false;
        return Notice::NONE;
    }
    if (ground_speed >= warn) {
        return due(_warned, _last_warn_ms, now_ms, WARN_NOTICE_MS) ? Notice::WARN : Notice::NONE;
    }
    if (ground_speed < warn - WARN_HYSTERESIS * _limit) {
        _warned = false;
    }
    return Notice::NONE;
}
