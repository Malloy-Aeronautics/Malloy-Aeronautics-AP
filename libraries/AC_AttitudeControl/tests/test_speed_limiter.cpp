/*
  Tests for the attitude controller speed limiter (FMSMS-456).
  
  SpeedLimit loop runs at 400Hz, trying to see if with these unit tests 
  the changes to the core functionality of the code changes anything

*/

#include <AP_gtest.h>

#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_PID/AC_PID.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// these normally come from AC_AttitudeControl.h
#ifndef AC_ATTITUDE_CONTROL_SPEED_LIMIT_P
#define AC_ATTITUDE_CONTROL_SPEED_LIMIT_P     2.0f
#endif
#ifndef AC_ATTITUDE_CONTROL_SPEED_LIMIT_I
#define AC_ATTITUDE_CONTROL_SPEED_LIMIT_I     0.05f
#endif
#ifndef AC_ATTITUDE_CONTROL_SPEED_LIMIT_D
#define AC_ATTITUDE_CONTROL_SPEED_LIMIT_D     0.5f
#endif
#ifndef AC_ATTITUDE_CONTROL_SPEED_LIMIT_IMAX
#define AC_ATTITUDE_CONTROL_SPEED_LIMIT_IMAX  M_PI_2
#endif

namespace {

constexpr float ANGLE_TOL = 1.0e-4f;   // rad
constexpr float COMP_TOL  = 1.0e-5f;

Quaternion attitude_deg(float roll_deg, float pitch_deg, float yaw_deg)
{
    Quaternion q;
    q.from_euler(radians(roll_deg), radians(pitch_deg), radians(yaw_deg));
    return q;
}

Vector2f vel_from_course(float speed, float course_deg)
{
    return Vector2f{speed * cosf(radians(course_deg)), speed * sinf(radians(course_deg))};
}

Vector3f thrust_dir(const Quaternion &q)
{
    return q * Vector3f{0.0f, 0.0f, -1.0f};
}

// horizontal axis at right angles to the velocity
Vector3f cross_track_axis(const Vector2f &vel)
{
    const Vector2f d = vel.normalized();
    return Vector3f{d.y, -d.x, 0.0f};
}

float tilt_along(const Quaternion &q, const Vector2f &vel)
{
    return AC_AttitudeControl::speed_limit_tilt_along_track(q, vel);
}

bool quat_equal(const Quaternion &a, const Quaternion &b)
{
    return a.q1 == b.q1 && a.q2 == b.q2 && a.q3 == b.q3 && a.q4 == b.q4;
}

}  // namespace

TEST(SpeedLimitGeometry, TiltAlongTrackIsZeroWhenLevel)
{
    for (float course = 0; course < 360; course += 30) {
        EXPECT_NEAR(tilt_along(attitude_deg(0, 0, 73), vel_from_course(8, course)), 0.0f, ANGLE_TOL);
    }
}

TEST(SpeedLimitGeometry, NoseDownTowardsTravelIsPositiveTilt)
{
    // negative pitch is nose down
    const Quaternion q = attitude_deg(0, -20, 0);
    EXPECT_NEAR(tilt_along(q, vel_from_course(5, 0)), radians(20), ANGLE_TOL);
    EXPECT_NEAR(tilt_along(q, vel_from_course(5, 180)), radians(-20), ANGLE_TOL);  // flying backwards = braking
}

TEST(SpeedLimitGeometry, TiltAcrossTrackDoesNotCount)
{
    EXPECT_NEAR(tilt_along(attitude_deg(25, 0, 0), vel_from_course(5, 0)), 0.0f, ANGLE_TOL);
}

TEST(SpeedLimitGeometry, TiltAlongTrackIsZeroForZeroVelocity)
{
    EXPECT_FLOAT_EQ(tilt_along(attitude_deg(10, -30, 0), Vector2f{}), 0.0f);
}

TEST(SpeedLimitGeometry, NoChangeWhenTiltIsWithinLimit)
{
    const Quaternion target = attitude_deg(0, -10, 45);
    Quaternion out;
    EXPECT_FALSE(AC_AttitudeControl::speed_limit_derate(target, vel_from_course(8, 45), radians(20), out));
    EXPECT_TRUE(quat_equal(out, target));
}

TEST(SpeedLimitGeometry, ClampsAlongTrackTiltToLimit)
{
    const Quaternion target = attitude_deg(0, -30, 0);
    const Vector2f vel = vel_from_course(8, 0);
    Quaternion out;
    ASSERT_TRUE(AC_AttitudeControl::speed_limit_derate(target, vel, radians(10), out));
    EXPECT_NEAR(tilt_along(out, vel), radians(10), ANGLE_TOL);
}

TEST(SpeedLimitGeometry, ZeroLimitLevelsTheAlongTrackAxisOnly)
{
    const Quaternion target = attitude_deg(15, -30, 0);
    const Vector2f vel = vel_from_course(8, 0);
    Quaternion out;
    ASSERT_TRUE(AC_AttitudeControl::speed_limit_derate(target, vel, 0.0f, out));
    EXPECT_NEAR(tilt_along(out, vel), 0.0f, ANGLE_TOL);
    // still rolled right, so still pushing east
    EXPECT_GT(thrust_dir(out).y, 0.2f);
}

TEST(SpeedLimitGeometry, PreservesCrossTrackComponent)
{
    const Vector2f vel = vel_from_course(8, 0);
    const Quaternion target = attitude_deg(15, -30, 0);
    Quaternion out;
    ASSERT_TRUE(AC_AttitudeControl::speed_limit_derate(target, vel, radians(5), out));
    const Vector3f axis = cross_track_axis(vel);
    EXPECT_NEAR(thrust_dir(out) * axis, thrust_dir(target) * axis, COMP_TOL);
}

TEST(SpeedLimitGeometry, PreservesHeadingForPureAlongTrackTilt)
{
    const float yaw_deg = 37.0f;
    const Quaternion target = attitude_deg(0, -30, yaw_deg);
    Quaternion out;
    ASSERT_TRUE(AC_AttitudeControl::speed_limit_derate(target, vel_from_course(8, yaw_deg), radians(10), out));
    float roll, pitch, yaw;
    out.to_euler(roll, pitch, yaw);
    EXPECT_NEAR(wrap_PI(yaw - radians(yaw_deg)), 0.0f, ANGLE_TOL);
    EXPECT_NEAR(roll, 0.0f, ANGLE_TOL);
    EXPECT_NEAR(pitch, radians(-10), ANGLE_TOL);
}

TEST(SpeedLimitGeometry, NegativeLimitCommandsBrakingTilt)
{
    const Vector2f vel = vel_from_course(8, 0);
    Quaternion out;
    ASSERT_TRUE(AC_AttitudeControl::speed_limit_derate(attitude_deg(0, -30, 0), vel, radians(-10), out));
    EXPECT_NEAR(tilt_along(out, vel), radians(-10), ANGLE_TOL);

    // already braking harder than that, so nothing to do
    const Quaternion braking = attitude_deg(0, 20, 0);
    EXPECT_FALSE(AC_AttitudeControl::speed_limit_derate(braking, vel, radians(-10), out));
    EXPECT_TRUE(quat_equal(out, braking));
}

TEST(SpeedLimitGeometry, SameResultOnEveryCourse)
{
    for (float course = 0; course < 360; course += 15) {
        SCOPED_TRACE(course);
        const Vector2f vel = vel_from_course(8, course);
        Quaternion out;
        ASSERT_TRUE(AC_AttitudeControl::speed_limit_derate(attitude_deg(0, -30, course), vel, radians(12), out));
        EXPECT_NEAR(tilt_along(out, vel), radians(12), ANGLE_TOL);
    }
}

TEST(SpeedLimitGeometry, HandlesVelocityNotAlignedWithHeading)
{
    // pointing north but drifting north east
    const Quaternion target = attitude_deg(0, -30, 0);
    const Vector2f vel = vel_from_course(8, 45);
    Quaternion out;
    ASSERT_TRUE(AC_AttitudeControl::speed_limit_derate(target, vel, radians(5), out));
    EXPECT_NEAR(tilt_along(out, vel), radians(5), ANGLE_TOL);
    const Vector3f axis = cross_track_axis(vel);
    EXPECT_NEAR(thrust_dir(out) * axis, thrust_dir(target) * axis, COMP_TOL);
}

TEST(SpeedLimitGeometry, IsIdempotent)
{
    const Vector2f vel = vel_from_course(8, 20);
    Quaternion once, twice;
    ASSERT_TRUE(AC_AttitudeControl::speed_limit_derate(attitude_deg(10, -35, 20), vel, radians(8), once));
    AC_AttitudeControl::speed_limit_derate(once, vel, radians(8), twice);
    EXPECT_NEAR(tilt_along(twice, vel), radians(8), ANGLE_TOL);
    EXPECT_NEAR((thrust_dir(twice) - thrust_dir(once)).length(), 0.0f, 10 * COMP_TOL);
}

TEST(SpeedLimitGeometry, SmallerLimitGivesSmallerTilt)
{
    const Quaternion target = attitude_deg(5, -40, 0);
    const Vector2f vel = vel_from_course(8, 0);
    float previous = -INFINITY;
    for (float limit_deg = -20; limit_deg <= 40; limit_deg += 5) {
        Quaternion out;
        AC_AttitudeControl::speed_limit_derate(target, vel, radians(limit_deg), out);
        const float tilt = tilt_along(out, vel);
        EXPECT_GE(tilt, previous - ANGLE_TOL);
        EXPECT_LE(tilt, radians(limit_deg) + ANGLE_TOL);
        previous = tilt;
    }
}

TEST(SpeedLimitGeometry, DoesNothingBelowMinimumSpeed)
{
    const Quaternion target = attitude_deg(0, -30, 0);
    for (const float speed : {0.0f, 0.5f * AC_ATTITUDE_CONTROL_SPEED_LIMIT_MIN_SPEED}) {
        Quaternion out;
        EXPECT_FALSE(AC_AttitudeControl::speed_limit_derate(target, vel_from_course(speed, 0), 0.0f, out));
        EXPECT_TRUE(quat_equal(out, target));
        EXPECT_FALSE(out.is_nan());
    }
}

TEST(SpeedLimitGeometry, IgnoresNonFiniteInputs)
{
    const Quaternion target = attitude_deg(0, -30, 0);
    const Vector2f bad_vels[] = {
        Vector2f{NAN, 1.0f}, Vector2f{1.0f, NAN}, Vector2f{INFINITY, 0.0f}, Vector2f{-INFINITY, 2.0f},
    };
    for (const Vector2f &vel : bad_vels) {
        Quaternion out;
        EXPECT_FALSE(AC_AttitudeControl::speed_limit_derate(target, vel, 0.0f, out));
        EXPECT_TRUE(quat_equal(out, target));
    }
    Quaternion out;
    EXPECT_FALSE(AC_AttitudeControl::speed_limit_derate(target, vel_from_course(8, 0), NAN, out));
    EXPECT_TRUE(quat_equal(out, target));
}

TEST(SpeedLimitGeometry, NearInvertedTargetStaysFinite)
{
    // e.g. mid flip in ACRO
    const Vector2f vel = vel_from_course(8, 0);
    Quaternion out;
    ASSERT_TRUE(AC_AttitudeControl::speed_limit_derate(attitude_deg(0, -170, 0), vel, 0.0f, out));
    EXPECT_FALSE(out.is_nan());
    EXPECT_TRUE(out.is_unit_length());
    EXPECT_NEAR(tilt_along(out, vel), 0.0f, ANGLE_TOL);
}

TEST(SpeedLimitGeometry, RandomisedInvariants)
{
    // fixed seed checking if the failures are repeatable, silent killer
    uint32_t seed = 12345;
    auto rnd = [&seed](float lo, float hi) {
        seed = seed * 1664525u + 1013904223u;
        return lo + (hi - lo) * float(seed >> 8) / float(1u << 24);
    };
    for (int i = 0; i < 5000; i++) {
        const Quaternion target = attitude_deg(rnd(-60, 60), rnd(-60, 60), rnd(-180, 180));
        const Vector2f vel = vel_from_course(rnd(0.2f, 30), rnd(-180, 180));
        const float limit = radians(rnd(-30, 45));
        Quaternion out;
        const bool limited = AC_AttitudeControl::speed_limit_derate(target, vel, limit, out);
        SCOPED_TRACE(i);
        ASSERT_FALSE(out.is_nan());
        ASSERT_TRUE(out.is_unit_length());
        const float tilt_in = tilt_along(target, vel);
        const float tilt_out = tilt_along(out, vel);
        EXPECT_EQ(limited, tilt_in > limit);
        EXPECT_LE(tilt_out, limit > tilt_in ? tilt_in + ANGLE_TOL : limit + ANGLE_TOL);
        const Vector3f axis = cross_track_axis(vel);
        EXPECT_NEAR(thrust_dir(out) * axis, thrust_dir(target) * axis, 10 * COMP_TOL);
    }
}

namespace {

constexpr float LOOP_DT = 1.0f / 400.0f;
constexpr float LIMIT   = 10.0f;           // m/s

// Very simple copter model holding altitude.
// horizontal accel is g * tan(tilt) minus linear drag. Tops out around 22 m/s at 30 deg.
struct PointMassCopter {
    Vector2f vel;                         // NE, m/s
    Vector2f wind;                        // NE, m/s
    Vector3f thrust{0.0f, 0.0f, -1.0f};   // NED
    float attitude_tc = 0.15f;            // s
    float drag = 0.25f;                   // 1/s

    void step(const Quaternion &attitude_cmd)
    {
        const Vector3f cmd = thrust_dir(attitude_cmd);
        thrust += (cmd - thrust) * (LOOP_DT / attitude_tc);
        thrust.normalize();
        const float vertical = MAX(-thrust.z, 0.2f);
        const Vector2f accel = Vector2f{thrust.x, thrust.y} * (GRAVITY_MSS / vertical) - (vel - wind) * drag;
        vel += accel * LOOP_DT;
    }
};

// keep the PID constructor args in line with _speed_limit_pid in AC_AttitudeControl
struct SpeedLimiter {
    AC_PID pid{AC_ATTITUDE_CONTROL_SPEED_LIMIT_P, AC_ATTITUDE_CONTROL_SPEED_LIMIT_I,
               AC_ATTITUDE_CONTROL_SPEED_LIMIT_D, 0.0f, AC_ATTITUDE_CONTROL_SPEED_LIMIT_IMAX,
               0.0f, 0.0f, 0.0f, LOOP_DT};
    float limit = LIMIT;
    float min_derating_angle = 0.0f;      // rad
    bool limited = false;

    // AC_PID doesn't initialise _integrator, it relies on ArduPilot's zeroing new. Here it's on the stack.
    SpeedLimiter() { pid.reset_I(); }

    Quaternion apply(const Quaternion &target, const Vector2f &vel)
    {
        return AC_AttitudeControl::speed_limit_update(pid, limit, min_derating_angle, target, vel, limited);
    }
};

struct RunStats {
    float max_speed = 0.0f;
    float mean_speed_last_10s = 0.0f;
    float final_speed = 0.0f;
    bool any_nan = false;
};

// pass limiter = nullptr to fly with the limiter off
RunStats fly(PointMassCopter &copter, SpeedLimiter *limiter, const Quaternion &pilot_target, float seconds)
{
    RunStats s;
    const int steps = int(seconds / LOOP_DT);
    const int last_10s = int(10.0f / LOOP_DT);
    float sum = 0.0f;
    for (int i = 0; i < steps; i++) {
        const Quaternion cmd = limiter ? limiter->apply(pilot_target, copter.vel) : pilot_target;
        s.any_nan |= cmd.is_nan();
        copter.step(cmd);
        const float speed = copter.vel.length();
        s.max_speed = MAX(s.max_speed, speed);
        if (i >= steps - last_10s) {
            sum += speed;
        }
    }
    s.mean_speed_last_10s = sum / float(MIN(steps, last_10s));
    s.final_speed = copter.vel.length();
    return s;
}

}

TEST(SpeedLimitClosedLoop, BaselineWithoutLimiterExceedsLimit)
{
    // make sure the model can actually go faster than the limit
    PointMassCopter copter;
    const RunStats s = fly(copter, nullptr, attitude_deg(0, -30, 0), 40);
    EXPECT_GT(s.final_speed, 1.8f * LIMIT);
}

TEST(SpeedLimitClosedLoop, HoldsSpeedAtLimitInStillAir)
{
    PointMassCopter copter;
    SpeedLimiter limiter;
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 60);
    EXPECT_FALSE(s.any_nan);
    EXPECT_LE(s.max_speed, LIMIT * 1.05f);
    EXPECT_NEAR(s.mean_speed_last_10s, LIMIT, 0.5f);
    EXPECT_TRUE(limiter.limited);
}

TEST(SpeedLimitClosedLoop, DISABLED_DoesNotInterfereBelowLimit)
{
    // 5 deg forward only gets to about 3.4 m/s
    PointMassCopter copter;
    SpeedLimiter limiter;
    const int steps = int(30.0f / LOOP_DT);
    const Quaternion pilot = attitude_deg(0, -5, 0);
    for (int i = 0; i < steps; i++) {
        const Quaternion cmd = limiter.apply(pilot, copter.vel);
        ASSERT_FALSE(limiter.limited) << "limited at t=" << i * LOOP_DT << "s, v=" << copter.vel.length();
        ASSERT_TRUE(quat_equal(cmd, pilot));
        copter.step(cmd);
    }
    EXPECT_NEAR(limiter.pid.get_i(), 0.0f, 1.0e-3f);
}

TEST(SpeedLimitClosedLoop, DISABLED_NoOvershootAfterLongHover)
{
    // hovering means a big speed error for a long time, so this is where I windup shows up
    PointMassCopter copter;
    SpeedLimiter limiter;
    fly(copter, &limiter, attitude_deg(0, 0, 0), 120);
    EXPECT_NEAR(limiter.pid.get_i(), 0.0f, 1.0e-3f);
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 40);
    EXPECT_LE(s.max_speed, LIMIT * 1.05f);
}

TEST(SpeedLimitClosedLoop, PilotCanStillSteerWhileLimited)
{
    PointMassCopter copter;
    SpeedLimiter limiter;
    fly(copter, &limiter, attitude_deg(0, -30, 0), 30);
    const float course_before = degrees(atan2f(copter.vel.y, copter.vel.x));

    // roll right while still holding full forward
    const RunStats s = fly(copter, &limiter, attitude_deg(25, -30, 0), 30);
    const float course_after = degrees(atan2f(copter.vel.y, copter.vel.x));

    EXPECT_GT(course_after - course_before, 20.0f);
    EXPECT_LE(s.max_speed, LIMIT * 1.05f);
}

TEST(SpeedLimitClosedLoop, NoResidualLimitingAfterSlowingDown)
{
    PointMassCopter copter;
    SpeedLimiter limiter;
    fly(copter, &limiter, attitude_deg(0, -30, 0), 60);
    fly(copter, &limiter, attitude_deg(0, 0, 0), 30);       // stick centred, slows down
    ASSERT_LT(copter.vel.length(), 0.3f * LIMIT);

    // full forward again, the limiter shouldn't kick in straight away
    const Quaternion pilot = attitude_deg(0, -30, 0);
    for (int i = 0; i < int(1.0f / LOOP_DT); i++) {
        const Quaternion cmd = limiter.apply(pilot, copter.vel);
        ASSERT_FALSE(limiter.limited) << "limited at v=" << copter.vel.length();
        copter.step(cmd);
    }
}

TEST(SpeedLimitClosedLoop, TailwindAboveLimitIsNotHeldWithoutBraking)
{
    PointMassCopter copter;
    copter.wind = vel_from_course(15, 0);
    SpeedLimiter limiter;
    limiter.min_derating_angle = 0.0f;
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 60);
    EXPECT_GT(s.mean_speed_last_10s, LIMIT + 3.0f);
}

TEST(SpeedLimitClosedLoop, TailwindAboveLimitIsHeldWithBraking)
{
    PointMassCopter copter;
    copter.wind = vel_from_course(15, 0);
    SpeedLimiter limiter;
    limiter.min_derating_angle = radians(-20);
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 60);
    EXPECT_FALSE(s.any_nan);
    EXPECT_NEAR(s.mean_speed_last_10s, LIMIT, 0.5f);
}

TEST(SpeedLimitClosedLoop, RecoversFromGustAboveLimit)
{
    PointMassCopter copter;
    SpeedLimiter limiter;
    limiter.min_derating_angle = radians(-20);
    fly(copter, &limiter, attitude_deg(0, -30, 0), 30);
    copter.vel += vel_from_course(4, 0);                    // gust
    fly(copter, &limiter, attitude_deg(0, -30, 0), 5);
    EXPECT_LE(copter.vel.length(), LIMIT + 0.5f);
}

AP_GTEST_MAIN()