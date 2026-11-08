#include "AP_GPSCheck.h"

#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>

bool AP_GPSCheck::receiver_ok(const Receiver &rx, const Criteria &criteria)
{
    return rx.age_ms < criteria.timeout_ms && rx.healthy && rx.fix_3d &&
           rx.sats >= criteria.min_sats &&
           (!rx.have_hacc || rx.hacc <= criteria.max_hacc) &&
           (!rx.have_sacc || rx.sacc <= criteria.max_sacc);
}

AP_GPSCheck::Receiver AP_GPSCheck::read_receiver(const AP_GPS &gps, uint8_t instance, uint32_t now_ms)
{
    Receiver rx {};
    rx.age_ms = now_ms - gps.last_message_time_ms(instance);
    rx.healthy = gps.is_healthy(instance);
    rx.fix_3d = gps.status(instance) >= AP_GPS::GPS_OK_FIX_3D;
    rx.sats = gps.num_sats(instance);
    rx.have_hacc = gps.horizontal_accuracy(instance, rx.hacc);
    rx.have_sacc = gps.speed_accuracy(instance, rx.sacc);
    return rx;
}

void AP_GPSCheck::update(const AP_GPS &gps, bool ekf_ok, uint32_t now_ms)
{
    Receiver receivers[GPS_MAX_RECEIVERS];
    const uint8_t count = MIN(gps.num_sensors(), (uint8_t)GPS_MAX_RECEIVERS);
    for (uint8_t i = 0; i < count; i++) {
        receivers[i] = read_receiver(gps, i, now_ms);
    }
    update(receivers, count, ekf_ok, now_ms);
}

void AP_GPSCheck::update(const Receiver *receivers, uint8_t count, bool ekf_ok, uint32_t now_ms)
{
    bool receiver_good = false;
    _status.ekf_ok = ekf_ok;
    _status.sats = 0;
    _status.hacc = -1.0f;
    _status.sacc = -1.0f;
    for (uint8_t i = 0; i < count; i++) {
        const Receiver &rx = receivers[i];
        if (rx.age_ms < _criteria.timeout_ms && rx.fix_3d) {
            _status.sats = MAX(_status.sats, rx.sats);
            if (rx.have_hacc && (_status.hacc < 0 || rx.hacc < _status.hacc)) {
                _status.hacc = rx.hacc;
            }
            if (rx.have_sacc && (_status.sacc < 0 || rx.sacc < _status.sacc)) {
                _status.sacc = rx.sacc;
            }
        }
        if (receiver_ok(rx, _criteria)) {
            receiver_good = true;
        }
    }
    _status.usable = ekf_ok && receiver_good;
    debounce(_status.usable, now_ms);
}

void AP_GPSCheck::debounce(bool usable, uint32_t now_ms)
{
    if (!usable) {
        _lost = true;
        _recovering = false;
        return;
    }
    if (!_lost) {
        return;
    }
    if (!_recovering) {
        _recovering = true;
        _back_ms = now_ms;
        return;
    }
    if (now_ms - _back_ms >= _criteria.recover_ms) {
        _lost = false;
        _recovering = false;
    }
}
