#pragma once

#include <AP_Param/AP_Param.h>

/*
  Ground speed limit that sends the vehicle home.

  The vehicle calls update() at a steady rate with its ground speed and whether the limit
  applies right now (armed, flying, in a mode it covers). As soon as the speed goes over
  SPD_LIM_VAL, update() returns true on that one call and the vehicle switches to RTL. Nothing
  else is changed: the attitude and the navigation speeds are left alone.

  The pilot can turn it off with an RC switch (override); a radio failsafe turns it back on.
*/
class AC_SpeedLimit {
public:
    AC_SpeedLimit();

    CLASS_NO_COPY(AC_SpeedLimit);

    // returns true on the call where the vehicle should start an RTL
    bool update(bool applies, float ground_speed);

    // SPD_LIM_EN is set (whether or not the pilot has overridden it)
    bool enabled() const { return _enabled != 0; }

    // enabled and not overridden by the pilot
    bool active() const { return enabled() && !_override; }

    void set_override(bool override_on);
    bool get_override() const { return _override; }

    float limit() const { return _limit; }          // m/s

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8 _enabled;
    AP_Float _limit;

    // set once it has asked for an RTL, until the speed is back under the limit or the limit
    // stops applying (the vehicle is in RTL by then), so one overspeed sends it home once.
    // Initialised here so it also behaves on the stack (the unit tests make it there)
    bool _triggered = false;
    bool _override = false;
};
