#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_GPSCheck/AP_GPSCheck.h>

/*
  Ground speed limit that sends the vehicle home.

  The vehicle calls update() at a steady rate with its ground speed and whether the speed should
  be checked right now (armed, flying, in a mode it covers). As soon as the speed goes over
  SPD_LIM_VAL, update() returns true and the vehicle switches to RTL. If it is still being checked
  and still over the limit a second later (the RTL was refused), update() asks again, once every
  RTL_RETRY_MS.

  notice() says what to tell the pilot on each update: a warning when the speed gets within
  SPD_LIM_WARN percent of the limit, and an alert for as long as it stays over the limit.

  The pilot can turn it off with an RC switch (override); a radio failsafe turns it back on.

  Without GPS the ground speed can't be trusted (the EKF carries on with a velocity that drifts),
  so the limit is off while the vehicle's AP_GPSCheck, built with gps_criteria(), says the GPS isn't
  ok. The vehicle passes that on with set_gps_ok().
*/
class AC_SpeedLimit {
public:
    // what the GPS has to meet for the ground speed to be trusted
    static constexpr uint8_t GPS_MIN_SATS = 4;          // the minimum for a 3D fix
    static constexpr float GPS_MAX_HACC = 10.0f;        // m
    static constexpr float GPS_MAX_SACC = 2.0f;         // m/s
    static constexpr uint32_t GPS_TIMEOUT_MS = 1000;    // a GPS silent this long doesn't count
    static constexpr uint32_t GPS_RECOVER_MS = 3000;    // GPS usable again this long: limit on again
    static AP_GPSCheck::Criteria gps_criteria() {
        return AP_GPSCheck::Criteria { GPS_MIN_SATS, GPS_MAX_HACC, GPS_MAX_SACC, GPS_TIMEOUT_MS, GPS_RECOVER_MS };
    }

    static constexpr uint32_t RTL_RETRY_MS = 1000;      // RTL refused: ask again this often
    static constexpr uint32_t OVER_NOTICE_MS = 1000;    // over the limit: alert this often
    static constexpr uint32_t WARN_NOTICE_MS = 3000;    // close to the limit: warn this often
    static constexpr float WARN_HYSTERESIS = 0.05f;     // of the limit, before a new warning

    AC_SpeedLimit();

    CLASS_NO_COPY(AC_SpeedLimit);

    // returns true when the vehicle should start an RTL: straight away on going over the limit,
    bool update(bool should_check, float ground_speed, uint32_t now_ms);

    enum class Notice : uint8_t {
        NONE = 0,
        WARN = 1,   // within SPD_LIM_WARN percent of the limit
        OVER = 2,   // over the limit
    };

    // what to tell the pilot this update. `should_check` as for update(); `returning` while the
    // vehicle is in the RTL the limit started, so the alerts carry on until it has slowed down
    Notice notice(bool should_check, bool returning, float ground_speed, uint32_t now_ms);

    // whether the GPS can be trusted, from the vehicle's AP_GPSCheck, before each update()
    void set_gps_ok(bool gps_ok);

    // SPD_LIM_EN is set (whether or not the pilot has overridden it or the GPS is lost)
    bool enabled() const { return _enabled != 0; }

    // the limit is live: enabled, not overridden, and the GPS is usable
    bool is_active() const { return enabled() && !_override && !_gps_lost; }

    // as last given to set_gps_ok()
    bool gps_ok() const { return !_gps_lost; }

    void set_override(bool override_on);
    bool get_override() const { return _override; }

    float limit() const { return _limit; }          // m/s

    // speed at which the warning starts, 0 when SPD_LIM_WARN is off
    float warn_speed() const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    bool can_check(float ground_speed) const;
    static bool due(bool &started, uint32_t &last_ms, uint32_t now_ms, uint32_t interval_ms);

    AP_Int8 _enabled;
    AP_Float _limit;
    AP_Int8 _warn_pct;

    // set once it has asked for an RTL, until the speed is back under the limit or it stops being
    // checked (the vehicle is in RTL by then). Initialised here so it also behaves on the stack
    // (the unit tests make it there)
    bool _triggered = false;
    uint32_t _last_trigger_ms = 0;
    bool _override = false;
    bool _gps_lost = false;

    // notice() state
    bool _over = false;
    bool _warned = false;
    uint32_t _last_over_ms = 0;
    uint32_t _last_warn_ms = 0;
};
