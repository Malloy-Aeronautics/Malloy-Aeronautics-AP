/*
  Tests for the ground speed limit that sends the vehicle home (AC_SpeedLimit).

  update() is fed a ground speed at 10Hz, the rate Copter calls it at, and the tests check
  when it asks for an RTL. What counts as a usable GPS is tested with AP_GPSCheck.

  ./waf configure --board sitl
  ./waf --target tests/test_speed_limit
  ./build/sitl/tests/test_speed_limit
*/

#include <AP_gtest.h>

#include <AC_SpeedLimit/AC_SpeedLimit.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

struct Limiter {
    AC_SpeedLimit limit;
    uint32_t t_ms = 1000;

    Limiter(float val = 10.0f, int enabled = 1)
    {
        AP_Param::set_object_value(&limit, AC_SpeedLimit::var_info, "EN", enabled);
        AP_Param::set_object_value(&limit, AC_SpeedLimit::var_info, "VAL", val);
    }

    // a number of 10Hz updates at a fixed speed, returns how many asked for an RTL and on which
    // update the first one came (0 = the first, -1 = none)
    int fly(float speed, int updates, bool should_check = true, int *first = nullptr)
    {
        int triggers = 0;
        if (first != nullptr) {
            *first = -1;
        }
        for (int i = 0; i < updates; i++, t_ms += 100) {
            if (limit.update(should_check, speed, t_ms)) {
                if (triggers == 0 && first != nullptr) {
                    *first = i;
                }
                triggers++;
            }
        }
        return triggers;
    }
};

}  // namespace

TEST(SpeedLimitRTL, DisabledByDefault)
{
    AC_SpeedLimit limit;
    EXPECT_FALSE(limit.enabled());
    EXPECT_FALSE(limit.is_active());
    EXPECT_FLOAT_EQ(limit.limit(), 20.0f);
    for (int i = 0; i < 50; i++) {
        EXPECT_FALSE(limit.update(true, 40.0f, 1000 + 100 * i));
    }
}

TEST(SpeedLimitRTL, DisabledNeverTriggers)
{
    Limiter l(10.0f, 0);
    EXPECT_EQ(l.fly(30.0f, 100), 0);
}

TEST(SpeedLimitRTL, UnderTheLimitNeverTriggers)
{
    Limiter l(10.0f);
    EXPECT_EQ(l.fly(9.9f, 600), 0);
    // exactly at the limit isn't over it
    EXPECT_EQ(l.fly(10.0f, 600), 0);
}

TEST(SpeedLimitRTL, OverTheLimitTriggersStraightAway)
{
    Limiter l(10.0f);
    int first;
    EXPECT_EQ(l.fly(10.1f, 1, true, &first), 1);
    EXPECT_EQ(first, 0);
}

TEST(SpeedLimitRTL, RefusedRTLAsksAgainEverySecond)
{
    // still checked and still over for 5s means the RTL was refused: asked for at 0, 1, 2, 3, 4s
    Limiter l(10.0f);
    int first;
    EXPECT_EQ(l.fly(12.0f, 50, true, &first), 5);
    EXPECT_EQ(first, 0);
}

TEST(SpeedLimitRTL, NoRetryOnceTheRTLHasTaken)
{
    // the RTL took, so the speed isn't checked any more: no more requests however long it's over
    Limiter l(10.0f);
    EXPECT_EQ(l.fly(12.0f, 1), 1);
    EXPECT_EQ(l.fly(12.0f, 50, false), 0);
}

TEST(SpeedLimitRTL, TriggersAgainAfterComingBackUnder)
{
    Limiter l(10.0f);
    EXPECT_EQ(l.fly(12.0f, 10), 1);
    EXPECT_EQ(l.fly(8.0f, 10), 0);
    EXPECT_EQ(l.fly(12.0f, 10), 1);
}

TEST(SpeedLimitRTL, NotAppliesNeverTriggers)
{
    // on the ground, spooling up, or in a mode that isn't checked
    Limiter l(10.0f);
    EXPECT_EQ(l.fly(30.0f, 100, false), 0);
}

TEST(SpeedLimitRTL, BackInACheckedModeOverTheLimitTriggersAgain)
{
    // RTL'd, then the pilot switches back to ALT_HOLD while still over the limit
    Limiter l(10.0f);
    EXPECT_EQ(l.fly(15.0f, 5), 1);
    EXPECT_EQ(l.fly(15.0f, 20, false), 0);      // in RTL, not checked
    int first;
    EXPECT_EQ(l.fly(15.0f, 5, true, &first), 1);
    EXPECT_EQ(first, 0);
}

TEST(SpeedLimitRTL, NonPositiveLimitIsOff)
{
    Limiter zero(0.0f);
    EXPECT_EQ(zero.fly(5.0f, 50), 0);
    Limiter negative(-1.0f);
    EXPECT_EQ(negative.fly(5.0f, 50), 0);
}

TEST(SpeedLimitRTL, BadSpeedNeverTriggers)
{
    Limiter l(10.0f);
    EXPECT_EQ(l.fly(NAN, 50), 0);
    EXPECT_EQ(l.fly(INFINITY, 50), 0);
    EXPECT_EQ(l.fly(12.0f, 1), 1);
}

TEST(SpeedLimitRTL, OverrideTurnsItOff)
{
    Limiter l(10.0f);
    l.limit.set_override(true);
    EXPECT_TRUE(l.limit.get_override());
    EXPECT_TRUE(l.limit.enabled());
    EXPECT_FALSE(l.limit.is_active());
    EXPECT_EQ(l.fly(30.0f, 100), 0);
}

TEST(SpeedLimitRTL, OverrideOffWhileOverTriggersStraightAway)
{
    Limiter l(10.0f);
    l.limit.set_override(true);
    EXPECT_EQ(l.fly(30.0f, 20), 0);
    l.limit.set_override(false);
    int first;
    EXPECT_EQ(l.fly(30.0f, 5, true, &first), 1);
    EXPECT_EQ(first, 0);
}

TEST(SpeedLimitRTL, OverrideClearsATrigger)
{
    // triggered, override on and off again while still over: it asks again
    Limiter l(10.0f);
    EXPECT_EQ(l.fly(15.0f, 5), 1);
    l.limit.set_override(true);
    l.limit.set_override(false);
    EXPECT_EQ(l.fly(15.0f, 5), 1);
}

TEST(SpeedLimitRTL, LoweringTheLimitInFlight)
{
    Limiter l(20.0f);
    EXPECT_EQ(l.fly(15.0f, 50), 0);
    AP_Param::set_object_value(&l.limit, AC_SpeedLimit::var_info, "VAL", 10.0f);
    int first;
    EXPECT_EQ(l.fly(15.0f, 5, true, &first), 1);
    EXPECT_EQ(first, 0);
}

TEST(SpeedLimitRTL, EnablingInFlightWhileOver)
{
    Limiter l(10.0f, 0);
    EXPECT_EQ(l.fly(15.0f, 50), 0);
    AP_Param::set_object_value(&l.limit, AC_SpeedLimit::var_info, "EN", 1);
    EXPECT_EQ(l.fly(15.0f, 5), 1);
}

TEST(SpeedLimitRTL, GPSLossTurnsItOff)
{
    Limiter l(10.0f);
    l.limit.set_gps_ok(true);
    EXPECT_TRUE(l.limit.is_active());
    l.limit.set_gps_ok(false);
    EXPECT_FALSE(l.limit.gps_ok());
    EXPECT_FALSE(l.limit.is_active());
    EXPECT_TRUE(l.limit.enabled());
    EXPECT_EQ(l.fly(30.0f, 50), 0);
}

TEST(SpeedLimitRTL, GPSBackWhileOverTriggersStraightAway)
{
    Limiter l(10.0f);
    l.limit.set_gps_ok(false);
    EXPECT_EQ(l.fly(30.0f, 20), 0);
    l.limit.set_gps_ok(true);
    int first;
    EXPECT_EQ(l.fly(30.0f, 5, true, &first), 1);
    EXPECT_EQ(first, 0);
}

TEST(SpeedLimitRTL, GPSCriteria)
{
    const AP_GPSCheck::Criteria c = AC_SpeedLimit::gps_criteria();
    EXPECT_EQ(c.min_sats, 4);
    EXPECT_FLOAT_EQ(c.max_hacc, 10.0f);
    EXPECT_FLOAT_EQ(c.max_sacc, 2.0f);
    EXPECT_EQ(c.timeout_ms, 1000U);
    EXPECT_EQ(c.recover_ms, 3000U);
}

using Notice = AC_SpeedLimit::Notice;

// notice() over `ms` milliseconds at 10Hz, returns how many of each it sent
static void notices(AC_SpeedLimit &lim, float speed, uint32_t &t, uint32_t ms, int &warns, int &overs,
                    bool should_check = true, bool returning = false)
{
    for (uint32_t end = t + ms; t < end; t += 100) {
        const Notice n = lim.notice(should_check, returning, speed, t);
        warns += n == Notice::WARN;
        overs += n == Notice::OVER;
    }
}

TEST(SpeedLimitNotice, WarnDefaultsTo90Percent)
{
    Limiter l(20.0f);
    EXPECT_FLOAT_EQ(l.limit.warn_speed(), 18.0f);
    AP_Param::set_object_value(&l.limit, AC_SpeedLimit::var_info, "WARN", 0);
    EXPECT_FLOAT_EQ(l.limit.warn_speed(), 0.0f);
}

TEST(SpeedLimitNotice, NothingWellUnderTheLimit)
{
    Limiter l(20.0f);
    uint32_t t = 1000;
    int w = 0, o = 0;
    notices(l.limit, 17.0f, t, 10000, w, o);
    EXPECT_EQ(w, 0);
    EXPECT_EQ(o, 0);
}

TEST(SpeedLimitNotice, WarnsStraightAwayThenEveryInterval)
{
    Limiter l(20.0f);
    uint32_t t = 1000;
    int w = 0, o = 0;
    EXPECT_EQ(l.limit.notice(true, false, 18.5f, t), Notice::WARN);
    t += 100;
    notices(l.limit, 18.5f, t, 9000, w, o);         // 9.1 s in the band in all
    EXPECT_EQ(w, 3);                                // at 3, 6 and 9 s
    EXPECT_EQ(o, 0);
}

TEST(SpeedLimitNotice, WarningNeedsToDropUnderTheHysteresis)
{
    Limiter l(20.0f);                               // warn at 18, re-arm under 17
    uint32_t t = 1000;
    EXPECT_EQ(l.limit.notice(true, false, 18.2f, t), Notice::WARN);
    EXPECT_EQ(l.limit.notice(true, false, 17.5f, t + 100), Notice::NONE);
    EXPECT_EQ(l.limit.notice(true, false, 18.2f, t + 200), Notice::NONE);   // not yet re-armed
    EXPECT_EQ(l.limit.notice(true, false, 16.5f, t + 300), Notice::NONE);
    EXPECT_EQ(l.limit.notice(true, false, 18.2f, t + 400), Notice::WARN);   // re-armed
}

TEST(SpeedLimitNotice, OverAlertsEverySecondWhileOver)
{
    Limiter l(20.0f);
    uint32_t t = 1000;
    int w = 0, o = 0;
    notices(l.limit, 22.0f, t, 5000, w, o);
    EXPECT_EQ(o, 5);
    EXPECT_EQ(w, 0);
}

TEST(SpeedLimitNotice, OverCarriesOnWhileReturning)
{
    Limiter l(20.0f);
    uint32_t t = 1000;
    int w = 0, o = 0;
    notices(l.limit, 22.0f, t, 3000, w, o, false, true);   // RTL started by the limit
    EXPECT_EQ(o, 3);
    w = o = 0;
    notices(l.limit, 19.0f, t, 5000, w, o, false, true);   // slowed down: quiet, no warnings in RTL
    EXPECT_EQ(o, 0);
    EXPECT_EQ(w, 0);
}

TEST(SpeedLimitNotice, QuietWhenOffOverriddenOrNoGPS)
{
    Limiter off(20.0f, 0);
    uint32_t t = 1000;
    int w = 0, o = 0;
    notices(off.limit, 25.0f, t, 3000, w, o);
    EXPECT_EQ(w + o, 0);

    Limiter ovr(20.0f);
    ovr.limit.set_override(true);
    notices(ovr.limit, 25.0f, t, 3000, w, o);
    EXPECT_EQ(w + o, 0);

    Limiter gps(20.0f);
    gps.limit.set_gps_ok(false);
    notices(gps.limit, 25.0f, t, 3000, w, o);
    EXPECT_EQ(w + o, 0);

    Limiter other_mode(20.0f);
    notices(other_mode.limit, 25.0f, t, 3000, w, o, false, false);
    EXPECT_EQ(w + o, 0);
}

AP_GTEST_MAIN()
