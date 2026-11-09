/*
  Tests for the check of whether the GPS can be trusted (AP_GPSCheck), with receivers filled in
  by hand.

  ./waf configure --board sitl
  ./waf --target tests/test_gps_check
  ./build/sitl/tests/test_gps_check
*/

#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_GPSCheck/AP_GPSCheck.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

const AP_GPSCheck::Criteria criteria { 4, 10.0f, 2.0f, 1000, 3000 };

// a receiver that passes every check
AP_GPSCheck::Receiver good_rx()
{
    AP_GPSCheck::Receiver rx {};
    rx.age_ms = 200;
    rx.healthy = true;
    rx.fix_3d = true;
    rx.sats = 12;
    rx.have_hacc = true;
    rx.hacc = 1.0f;
    rx.have_sacc = true;
    rx.sacc = 0.3f;
    return rx;
}

// a check that has had good GPS for long enough, so ok() is true at `t`
void settle(AP_GPSCheck &check, const AP_GPSCheck::Receiver &rx, uint32_t &t)
{
    check.update(&rx, 1, true, t);
    t += criteria.recover_ms;
    check.update(&rx, 1, true, t);
}

}  // namespace

TEST(GPSCheckReceiver, GoodReceiverPasses)
{
    EXPECT_TRUE(AP_GPSCheck::receiver_ok(good_rx(), criteria));
}

TEST(GPSCheckReceiver, EachCheckFailsOnItsOwn)
{
    AP_GPSCheck::Receiver rx = good_rx();
    rx.age_ms = criteria.timeout_ms;
    EXPECT_FALSE(AP_GPSCheck::receiver_ok(rx, criteria));

    rx = good_rx();
    rx.healthy = false;
    EXPECT_FALSE(AP_GPSCheck::receiver_ok(rx, criteria));

    rx = good_rx();
    rx.fix_3d = false;
    EXPECT_FALSE(AP_GPSCheck::receiver_ok(rx, criteria));

    rx = good_rx();
    rx.sats = criteria.min_sats - 1;
    EXPECT_FALSE(AP_GPSCheck::receiver_ok(rx, criteria));

    rx = good_rx();
    rx.hacc = criteria.max_hacc + 0.1f;
    EXPECT_FALSE(AP_GPSCheck::receiver_ok(rx, criteria));

    rx = good_rx();
    rx.sacc = criteria.max_sacc + 0.1f;
    EXPECT_FALSE(AP_GPSCheck::receiver_ok(rx, criteria));
}

TEST(GPSCheckReceiver, LimitsAreInclusive)
{
    AP_GPSCheck::Receiver rx = good_rx();
    rx.sats = criteria.min_sats;
    rx.hacc = criteria.max_hacc;
    rx.sacc = criteria.max_sacc;
    EXPECT_TRUE(AP_GPSCheck::receiver_ok(rx, criteria));
}

TEST(GPSCheckReceiver, UnreportedAccuracyIsNotHeldAgainstIt)
{
    AP_GPSCheck::Receiver rx = good_rx();
    rx.have_hacc = false;
    rx.hacc = 1000.0f;
    rx.have_sacc = false;
    rx.sacc = 1000.0f;
    EXPECT_TRUE(AP_GPSCheck::receiver_ok(rx, criteria));
}

TEST(GPSCheck, StartsNotOk)
{
    AP_GPSCheck check(criteria);
    EXPECT_FALSE(check.ok());
}

TEST(GPSCheck, AfterBootGPSHasToEarnIt)
{
    AP_GPSCheck check(criteria);
    const AP_GPSCheck::Receiver rx = good_rx();
    check.update(&rx, 1, true, 1000);
    EXPECT_TRUE(check.status().usable);
    EXPECT_FALSE(check.ok());
    check.update(&rx, 1, true, 1000 + criteria.recover_ms - 1);
    EXPECT_FALSE(check.ok());
    check.update(&rx, 1, true, 1000 + criteria.recover_ms);
    EXPECT_TRUE(check.ok());
}

TEST(GPSCheck, EKFNotOnGPSIsNotUsable)
{
    AP_GPSCheck check(criteria);
    const AP_GPSCheck::Receiver rx = good_rx();
    uint32_t t = 1000;
    settle(check, rx, t);
    check.update(&rx, 1, false, t + 100);
    EXPECT_FALSE(check.status().usable);
    EXPECT_FALSE(check.ok());
}

TEST(GPSCheck, NoReceiversIsNotUsable)
{
    AP_GPSCheck check(criteria);
    uint32_t t = 1000;
    settle(check, good_rx(), t);
    check.update(nullptr, 0, true, t + 100);
    EXPECT_FALSE(check.ok());
}

TEST(GPSCheck, OneGoodReceiverIsEnough)
{
    AP_GPSCheck check(criteria);
    AP_GPSCheck::Receiver rx[2] = { good_rx(), good_rx() };
    rx[0].fix_3d = false;
    check.update(rx, 2, true, 1000);
    check.update(rx, 2, true, 1000 + criteria.recover_ms);
    EXPECT_TRUE(check.ok());
    rx[1].sats = 3;
    check.update(rx, 2, true, 1100 + criteria.recover_ms);
    EXPECT_FALSE(check.ok());
}

TEST(GPSCheck, LossIsImmediate)
{
    AP_GPSCheck check(criteria);
    AP_GPSCheck::Receiver rx = good_rx();
    uint32_t t = 1000;
    settle(check, rx, t);
    EXPECT_TRUE(check.ok());
    rx.hacc = 20.0f;
    check.update(&rx, 1, true, t + 100);
    EXPECT_FALSE(check.ok());
}

TEST(GPSCheck, BackOnlyAfterRecoverTime)
{
    AP_GPSCheck check(criteria);
    AP_GPSCheck::Receiver rx = good_rx();
    check.update(&rx, 1, false, 1000);
    uint32_t t = 2000;
    check.update(&rx, 1, true, t);                  // usable again, the wait starts
    EXPECT_TRUE(check.status().usable);
    EXPECT_FALSE(check.ok());
    t += criteria.recover_ms - 1;
    check.update(&rx, 1, true, t);
    EXPECT_FALSE(check.ok());
    check.update(&rx, 1, true, t + 1);
    EXPECT_TRUE(check.ok());
}

TEST(GPSCheck, TimeLeftWhileRecovering)
{
    AP_GPSCheck check(criteria);
    const AP_GPSCheck::Receiver rx = good_rx();
    EXPECT_FALSE(check.recovering());
    EXPECT_EQ(check.ms_until_ok(1000), 0U);
    check.update(&rx, 1, true, 1000);
    EXPECT_TRUE(check.recovering());
    EXPECT_EQ(check.ms_until_ok(1000), criteria.recover_ms);
    EXPECT_EQ(check.ms_until_ok(2000), criteria.recover_ms - 1000);
    check.update(&rx, 1, true, 1000 + criteria.recover_ms);
    EXPECT_FALSE(check.recovering());
    EXPECT_EQ(check.ms_until_ok(1000 + criteria.recover_ms), 0U);
}

TEST(GPSCheck, DropoutDuringRecoveryRestartsTheWait)
{
    AP_GPSCheck check(criteria);
    const AP_GPSCheck::Receiver rx = good_rx();
    check.update(&rx, 1, false, 1000);
    check.update(&rx, 1, true, 2000);
    check.update(&rx, 1, false, 4000);              // lost again before it came back
    check.update(&rx, 1, true, 4500);
    check.update(&rx, 1, true, 4500 + criteria.recover_ms - 1);
    EXPECT_FALSE(check.ok());
    check.update(&rx, 1, true, 4500 + criteria.recover_ms);
    EXPECT_TRUE(check.ok());
}

TEST(GPSCheck, StatusFigures)
{
    AP_GPSCheck check(criteria);
    AP_GPSCheck::Receiver rx[2] = { good_rx(), good_rx() };
    rx[0].sats = 9;
    rx[0].hacc = 2.0f;
    rx[0].sacc = 0.5f;
    rx[1].sats = 14;
    rx[1].hacc = 0.8f;
    rx[1].have_sacc = false;
    check.update(rx, 2, true, 1000);
    EXPECT_EQ(check.status().sats, 14);
    EXPECT_FLOAT_EQ(check.status().hacc, 0.8f);
    EXPECT_FLOAT_EQ(check.status().sacc, 0.5f);

    // a receiver without a fix or gone silent doesn't count towards the figures
    rx[1].fix_3d = false;
    rx[0].age_ms = criteria.timeout_ms;
    check.update(rx, 2, true, 1100);
    EXPECT_EQ(check.status().sats, 0);
    EXPECT_FLOAT_EQ(check.status().hacc, -1.0f);
    EXPECT_FLOAT_EQ(check.status().sacc, -1.0f);
}

AP_GTEST_MAIN()
