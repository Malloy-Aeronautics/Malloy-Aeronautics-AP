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

bool AC_SpeedLimit::update(bool applies, float ground_speed)
{
    // a limit of zero or less would send the vehicle home as soon as it moved, treat it as off
    if (!active() || !applies || !isfinite(ground_speed) || !is_positive(_limit.get()) ||
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
