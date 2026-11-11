#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_GPSCheck/AP_GPSCheck.h>

/*
  Ground speed limit that sends the vehicle home.

Ground speed limiter that triggers RTL when the vehicle exceeds the configured speed limit. update() is called regularly with the current
ground speed and whether limit checking is active; it requests RTL whenever the speed exceeds SPD_LIM_VAL and retries at RTL_RETRY_MS intervals if
the violation continues. notice() provides pilot warnings when approaching the limit and alerts while over it, with logic to avoid repeated alerts
during the RTL that was triggered by the limit. It also warns when GPS recovery will reactivate limit enforcement. gps_notice() reports when the
limit becomes inactive or active due to GPS state, rate-limiting reactivation messages to avoid noise from intermittent GPS loss. The limiter can
be disabled by RC override and is automatically re-enabled by radio failsafe. Because reliable ground speed depends on GPS, enforcement is disabled
whenever GPS validity reported through set_gps_ok() is not confirmed.
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
    static constexpr float WARN_HYSTERESIS = 0.05f;     // of the limit, to drop under before a new warning
    static constexpr uint32_t GPS_NOTICE_MS = 10000;    // at most one "active again" message this often

    AC_SpeedLimit();

    CLASS_NO_COPY(AC_SpeedLimit);

    // returns true when the vehicle should start an RTL: straight away on going over the limit,
    bool update(bool should_check, float ground_speed, uint32_t now_ms);

    enum class Notice : uint8_t {
        NONE = 0,
        WARN = 1,       // within SPD_LIM_WARN percent of the limit
        OVER = 2,       // over the limit
        GPS_BACK = 3,   // over the limit with the GPS coming back: the RTL is about to follow
    };

    // what to tell the pilot this update. `should_check` as for update(); `returning` while the
    // vehicle is in the RTL the limit started, so the alerts carry on until it has slowed down
    Notice notice(bool should_check, bool returning, float ground_speed, uint32_t now_ms);

    // whether the GPS can be trusted, from the vehicle's AP_GPSCheck, before each update().
    // coming_back: not ok yet, but usable again and waiting out GPS_RECOVER_MS
    void set_gps_ok(bool gps_ok, bool coming_back = false);

    enum class GpsNotice : uint8_t {
        NONE,
        INACTIVE,   // the limit has gone inactive, no GPS
        ACTIVE,     // the limit is active again
    };

    // what to tell the pilot about the GPS this update. Only while enabled and not overridden: the
    // pilot switch has its own messages
    GpsNotice gps_notice(uint32_t now_ms);

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

    // the three parts of notice(), in the order it asks them
    bool gps_back_notice(bool should_check, float ground_speed);
    bool over_notice(uint32_t now_ms);
    bool warn_notice(bool should_check, float ground_speed);

    AP_Int8 _enabled;
    AP_Float _limit;
    AP_Int8 _warn_pct;

    // set once it has asked for an RTL, until the speed is back under the limit or it stops being
    // checked (the vehicle is in RTL by then). Initialised here so it also behaves on the stack
    // (the unit tests make it there)
    bool _triggered = false;
    uint32_t _last_trigger_ms = 0;
    bool _override = false;
    bool _gps_lost = true;
    bool _gps_coming_back = false;

    // notice() state
    bool _over = false;
    bool _returned_under = false;       // in the RTL the limit started, the speed has been under the limit
    float _returned_under_limit = 0.0f; // the limit it was under
    bool _warned = false;
    bool _told_gps_back = false;
    uint32_t _last_over_ms = 0;

    // gps_notice() state: what the pilot was last told, and when
    bool _told_active = false;
    bool _gps_told_once = false;
    uint32_t _last_gps_notice_ms = 0;
};
