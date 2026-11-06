#include "AircraftInfo.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// how often counters are written to storage while armed
#define AIRCRAFT_INFO_SAVE_INTERVAL_MS 60000U

const AP_Param::GroupInfo AircraftInfo::var_info[] = {
    // @Param: CHS
    // @DisplayName: Chassis number
    // @Description: Aircraft chassis number. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("CHS", 1, AircraftInfo, chassis_number, 0),

    // @Param: MOT1
    // @DisplayName: Motor 1 serial number
    // @Description: Serial number of motor 1. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MOT1", 2, AircraftInfo, motor_sn[0], 0),

    // @Param: MOT2
    // @DisplayName: Motor 2 serial number
    // @Description: Serial number of motor 2. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MOT2", 3, AircraftInfo, motor_sn[1], 0),

    // @Param: MOT3
    // @DisplayName: Motor 3 serial number
    // @Description: Serial number of motor 3. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MOT3", 4, AircraftInfo, motor_sn[2], 0),

    // @Param: MOT4
    // @DisplayName: Motor 4 serial number
    // @Description: Serial number of motor 4. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MOT4", 5, AircraftInfo, motor_sn[3], 0),

    // @Param: MOT5
    // @DisplayName: Motor 5 serial number
    // @Description: Serial number of motor 5. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MOT5", 6, AircraftInfo, motor_sn[4], 0),

    // @Param: MOT6
    // @DisplayName: Motor 6 serial number
    // @Description: Serial number of motor 6. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MOT6", 7, AircraftInfo, motor_sn[5], 0),

    // @Param: MOT7
    // @DisplayName: Motor 7 serial number
    // @Description: Serial number of motor 7. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MOT7", 8, AircraftInfo, motor_sn[6], 0),

    // @Param: MOT8
    // @DisplayName: Motor 8 serial number
    // @Description: Serial number of motor 8. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MOT8", 9, AircraftInfo, motor_sn[7], 0),

    // @Param: MM1
    // @DisplayName: MM1 serial number
    // @Description: Serial number of MM1. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MM1", 10, AircraftInfo, mm_sn[0], 0),

    // @Param: MM2
    // @DisplayName: MM2 serial number
    // @Description: Serial number of MM2. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MM2", 11, AircraftInfo, mm_sn[1], 0),

    // @Param: MM3
    // @DisplayName: MM3 serial number
    // @Description: Serial number of MM3. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MM3", 12, AircraftInfo, mm_sn[2], 0),

    // @Param: MM4
    // @DisplayName: MM4 serial number
    // @Description: Serial number of MM4. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("MM4", 13, AircraftInfo, mm_sn[3], 0),

    // @Param: ARM1
    // @DisplayName: Arm 1 serial number
    // @Description: Serial number of arm 1. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ARM1", 14, AircraftInfo, arm_sn[0], 0),

    // @Param: ARM2
    // @DisplayName: Arm 2 serial number
    // @Description: Serial number of arm 2. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ARM2", 15, AircraftInfo, arm_sn[1], 0),

    // @Param: ARM3
    // @DisplayName: Arm 3 serial number
    // @Description: Serial number of arm 3. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ARM3", 16, AircraftInfo, arm_sn[2], 0),

    // @Param: ARM4
    // @DisplayName: Arm 4 serial number
    // @Description: Serial number of arm 4. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ARM4", 17, AircraftInfo, arm_sn[3], 0),

    // @Param: ESC1
    // @DisplayName: ESC 1 serial number
    // @Description: Serial number of ESC 1. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ESC1", 18, AircraftInfo, esc_sn[0], 0),

    // @Param: ESC2
    // @DisplayName: ESC 2 serial number
    // @Description: Serial number of ESC 2. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ESC2", 19, AircraftInfo, esc_sn[1], 0),

    // @Param: ESC3
    // @DisplayName: ESC 3 serial number
    // @Description: Serial number of ESC 3. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ESC3", 20, AircraftInfo, esc_sn[2], 0),

    // @Param: ESC4
    // @DisplayName: ESC 4 serial number
    // @Description: Serial number of ESC 4. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ESC4", 21, AircraftInfo, esc_sn[3], 0),

    // @Param: ESC5
    // @DisplayName: ESC 5 serial number
    // @Description: Serial number of ESC 5. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ESC5", 22, AircraftInfo, esc_sn[4], 0),

    // @Param: ESC6
    // @DisplayName: ESC 6 serial number
    // @Description: Serial number of ESC 6. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ESC6", 23, AircraftInfo, esc_sn[5], 0),

    // @Param: ESC7
    // @DisplayName: ESC 7 serial number
    // @Description: Serial number of ESC 7. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ESC7", 24, AircraftInfo, esc_sn[6], 0),

    // @Param: ESC8
    // @DisplayName: ESC 8 serial number
    // @Description: Serial number of ESC 8. Stored for identification only and not used by the firmware.
    // @Range: 0 16777216
    // @User: Advanced
    AP_GROUPINFO("ESC8", 25, AircraftInfo, esc_sn[7], 0),

    // @Param: FLT_HRS
    // @DisplayName: Flight hours
    // @Description: Hours the aircraft has been flying since this was last set, using the same land detector as STAT_FLTTIME. Set to zero to reset, and it counts up again from there.
    // @User: Advanced
    AP_GROUPINFO("FLT_HRS", 26, AircraftInfo, flt_hrs, 0),

    AP_GROUPEND
};

AircraftInfo::AircraftInfo()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AircraftInfo::update()
{
    constrain_serials();
    update_flight_time();

    // save periodically while flying, set_flying() saves on landing
    const uint32_t now_ms = AP_HAL::millis();
    if (flying_ms && now_ms - last_save_ms >= AIRCRAFT_INFO_SAVE_INTERVAL_MS) {
        flt_hrs.save();
        last_save_ms = now_ms;
    }
}

// called by the land detector, the same signal AP_Stats uses for STAT_FLTTIME
void AircraftInfo::set_flying(const bool is_flying)
{
    if (is_flying) {
        if (!flying_ms) {
            flying_ms = AP_HAL::millis();
            last_save_ms = flying_ms;
        }
    } else {
        update_flight_time();
        flying_ms = 0;
        flt_hrs.save();
    }
}

void AircraftInfo::update_flight_time()
{
    // hold the count as whole seconds, the parameter shows it as hours.
    // FLT_HRS may be set to any value (e.g. zero to reset), then keeps counting from it
    if (!is_equal(flt_hrs.get(), flt_s / 3600.0f)) {
        const float hrs = flt_hrs.get();
        flt_s = hrs > 0 ? uint32_t(hrs * 3600.0f + 0.5f) : 0;
    }
    if (flying_ms) {
        const uint32_t delta_s = (AP_HAL::millis() - flying_ms) / 1000U;
        flt_s += delta_s;
        flying_ms += delta_s * 1000U;
    }
    flt_hrs.set(flt_s / 3600.0f);
}

// keep serial numbers within 0 to AIRCRAFT_INFO_SN_MAX
void AircraftInfo::constrain_serials()
{
    AP_Int32 *serials[] = {
        &chassis_number,
        &motor_sn[0], &motor_sn[1], &motor_sn[2], &motor_sn[3],
        &motor_sn[4], &motor_sn[5], &motor_sn[6], &motor_sn[7],
        &mm_sn[0], &mm_sn[1], &mm_sn[2], &mm_sn[3],
        &arm_sn[0], &arm_sn[1], &arm_sn[2], &arm_sn[3],
        &esc_sn[0], &esc_sn[1], &esc_sn[2], &esc_sn[3],
        &esc_sn[4], &esc_sn[5], &esc_sn[6], &esc_sn[7],
    };
    for (AP_Int32 *sn : serials) {
        const int32_t v = sn->get();
        if (v < 0 || v > AIRCRAFT_INFO_SN_MAX) {
            sn->set_and_save(constrain_int32(v, 0, AIRCRAFT_INFO_SN_MAX));
            gcs().send_text(MAV_SEVERITY_WARNING, "SN_ serial out of range, limited to 0 to %u", unsigned(AIRCRAFT_INFO_SN_MAX));
        }
    }
}
