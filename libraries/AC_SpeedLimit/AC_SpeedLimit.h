#pragma once

#include <AP_Param/AP_Param.h>

/*
  Ground speed limit that sends the vehicle home.

  The vehicle calls update() at a steady rate with its ground speed and whether the limit
  applies right now (armed, flying, in a mode it covers). As soon as the speed goes over
  SPD_LIM_VAL, update() returns true on that one call and the vehicle switches to RTL. Nothing
  else is changed: the attitude and the navigation speeds are left alone.

  The pilot can turn it off with an RC switch (override); a radio failsafe turns it back on.

  Without GPS the ground speed can't be trusted (the EKF carries on with a velocity that drifts),
  so the limit is switched off as soon as the EKF stops using GPS, and back on once it has been
  using it again for GPS_RECOVER_MS. This goes on the EKF, not on the GPS fix type.
*/
class AC_SpeedLimit {
public:
    static constexpr uint32_t GPS_RECOVER_MS = 3000;    // EKF back on GPS this long: limit on again

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

    // tell it whether the EKF is using GPS. Returns +1 when that turns the limit back on, -1 when
    // it turns it off, 0 otherwise
    int8_t update_gps(bool using_gps, uint32_t now_ms);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8 _enabled;
    AP_Float _limit;

    // set once it has asked for an RTL, until the speed is back under the limit or the limit
    // stops applying (the vehicle is in RTL by then), so one overspeed sends it home once.
    // Initialised here so it also behaves on the stack (the unit tests make it there)
    bool _triggered = false;
    bool _override = false;
    bool _gps_lost = false;
    uint32_t _gps_back_ms = 0;      // when the EKF started using GPS again, 0 = not yet
};
