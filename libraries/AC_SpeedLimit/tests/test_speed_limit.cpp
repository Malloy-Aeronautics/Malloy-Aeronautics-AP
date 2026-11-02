/*
  Tests for the ground speed limit that sends the vehicle home (AC_SpeedLimit).

  update() is fed a ground speed at 10Hz, the rate Copter calls it at, and the tests check
  when it asks for an RTL.

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

    Limiter(float val = 10.0f, int enabled = 1)
    {
        AP_Param::set_object_value(&limit, AC_SpeedLimit::var_info, "EN", enabled);
        AP_Param::set_object_value(&limit, AC_SpeedLimit::var_info, "VAL", val);
    }

    // a number of 10Hz updates at a fixed speed, returns how many asked for an RTL and on which
    // update the first one came (0 = the first, -1 = none)
    int fly(float speed, int updates, bool applies = true, int *first = nullptr)
    {
        int triggers = 0;
        if (first != nullptr) {
            *first = -1;
        }
        for (int i = 0; i < updates; i++) {
            if (limit.update(applies, speed)) {
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
    EXPECT_FALSE(limit.active());
    EXPECT_FLOAT_EQ(limit.limit(), 20.0f);
    for (int i = 0; i < 50; i++) {
        EXPECT_FALSE(limit.update(true, 40.0f));
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

TEST(SpeedLimitRTL, OneOverspeedTriggersOnce)
{
    Limiter l(10.0f);
    int first;
    // stays over for 5s (if the mode change were refused): still only one RTL request
    EXPECT_EQ(l.fly(12.0f, 50, true, &first), 1);
    EXPECT_EQ(first, 0);
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
    EXPECT_FALSE(l.limit.active());
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

AP_GTEST_MAIN()
