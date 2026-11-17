#pragma once

#include <AP_Param/AP_Param.h>

#define AIRCRAFT_INFO_NUM_MOTORS 8
#define AIRCRAFT_INFO_NUM_MM 4
#define AIRCRAFT_INFO_NUM_ARMS 4
#define AIRCRAFT_INFO_NUM_ESCS 8

// largest integer a ground station can send exactly, since parameters
// travel as 32 bit floats
#define AIRCRAFT_INFO_SN_MAX 16777216

/*
  identification parameters: chassis number, motor, MM, arm and ESC
  serial numbers, plus a resettable flight hours timer.
 */
class AircraftInfo {
public:
    AircraftInfo();

    // call at 1Hz
    void update();

    // call when the land detector changes state
    void set_flying(bool is_flying);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int32 chassis_number;
    AP_Int32 motor_sn[AIRCRAFT_INFO_NUM_MOTORS];
    AP_Int32 mm_sn[AIRCRAFT_INFO_NUM_MM];
    AP_Int32 arm_sn[AIRCRAFT_INFO_NUM_ARMS];
    AP_Int32 esc_sn[AIRCRAFT_INFO_NUM_ESCS];
    AP_Float flt_hrs;

    // flight seconds held exactly here, the parameter shows them as hours
    uint32_t flt_s;

    // start of the flight second being counted, 0 when not flying
    uint32_t flying_ms;
    uint32_t last_save_ms;

    void constrain_serials();
    void update_flight_time();
};
