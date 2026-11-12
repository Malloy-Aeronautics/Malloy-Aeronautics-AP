#pragma once

#include <AP_Common/AP_Common.h>
#include <stdint.h>

class AP_GPS;

/*
  Decides whether the GPS is good enough to trust, for a feature that depends on it (the ground
  speed limit, for one). Each feature keeps its own AP_GPSCheck with its own thresholds.

  The GPS is usable on an update when the caller says the EKF is navigating on it and at least one
  receiver passes receiver_ok(), so with two receivers fitted losing one isn't a GPS loss. ok()
  goes false on the first update that isn't usable, and only goes true again once the GPS has been
  usable for recover_ms without a break, so a GPS that keeps dropping in and out stays off. It
  starts off false too: after boot the GPS has to earn it the same way.
*/
class AP_GPSCheck {
public:
    struct Criteria {
        uint8_t min_sats;
        float max_hacc;         // m
        float max_sacc;         // m/s
        uint32_t timeout_ms;    // a receiver silent this long doesn't count
        uint32_t recover_ms;    // usable this long again before ok() comes back
    };

    // what is known about one receiver
    struct Receiver {
        uint32_t age_ms;    // since its last message
        bool healthy;       // the GPS library's own health check (messages arriving on time)
        bool fix_3d;        // 3D fix or better
        uint8_t sats;
        bool have_hacc;     // false when the receiver doesn't report horizontal accuracy
        float hacc;         // m
        bool have_sacc;     // false when the receiver doesn't report speed accuracy
        float sacc;         // m/s
    };

    // the latest update, for logging and messages
    struct Status {
        bool ekf_ok;        // as given by the caller
        bool usable;        // on this update, before the recovery wait
        uint8_t sats;       // most satellites on any receiver still sending with a 3D fix
        float hacc;         // best horizontal accuracy among those, -1 if none reports one
        float sacc;         // best speed accuracy among those, -1 if none reports one
    };

    explicit AP_GPSCheck(const Criteria &criteria) : _criteria(criteria) {}

    CLASS_NO_COPY(AP_GPSCheck);

    // true when this receiver passes every check. An accuracy the receiver doesn't report isn't
    // held against it
    static bool receiver_ok(const Receiver &rx, const Criteria &criteria);

    static Receiver read_receiver(const AP_GPS &gps, uint8_t instance, uint32_t now_ms);

    // read every receiver and update, at a steady rate. ekf_ok: the EKF is navigating on GPS
    void update(const AP_GPS &gps, bool ekf_ok, uint32_t now_ms);

    // the same with the receivers already read
    void update(const Receiver *receivers, uint8_t count, bool ekf_ok, uint32_t now_ms);

    // the GPS can be trusted: usable for at least recover_ms
    bool ok() const { return !_lost; }

    // usable again, but not for long enough yet
    bool recovering() const { return _recovering; }

    // how long until ok() comes back, while recovering(). 0 otherwise
    uint32_t ms_until_ok(uint32_t now_ms) const;

    const Status &status() const { return _status; }

private:
    void debounce(bool usable, uint32_t now_ms);

    const Criteria _criteria;
    Status _status {};
    bool _lost = true;
    bool _recovering = false;
    uint32_t _back_ms = 0;      // when it became usable again, while _recovering
};
