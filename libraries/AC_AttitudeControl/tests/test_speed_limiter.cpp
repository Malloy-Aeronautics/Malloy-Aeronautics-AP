/*
  Tests for the attitude controller speed limiter (FMSMS-456).

  SpeedLimitGeometry checks speed_limit_derate() and speed_limit_tilt_along_track() directly.
  SpeedLimitClosedLoop runs speed_limit_update() against a simple point mass model at 400Hz.
  SpeedLimitRobustness adds a noisy velocity estimate, braking, headwind and limit changes.
  SpeedLimitParams covers parameter values that make the vehicle unusable (pre-arm check fodder).

  ./waf configure --board sitl
  ./waf --target tests/test_speed_limiter
  ./build/sitl/tests/test_speed_limiter
*/

#include <AP_gtest.h>

#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_PID/AC_PID.h>

#include <random>

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

// bit for bit, so we know the target was passed through untouched
bool quat_equal(const Quaternion &a, const Quaternion &b)
{
    return memcmp(&a, &b, sizeof(Quaternion)) == 0;
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
    // fixed seed so failures are repeatable
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
// pass bands, set just outside what the correct code does (max 9.97, mean 9.97, settles in 2.8s)
constexpr float MAX_OVERSHOOT = 1.02f;     // max speed / limit
constexpr float MEAN_TOL      = 0.15f;     // m/s, steady state mean vs target

// Very simple copter model holding altitude. Thrust direction lags the command (first order),
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
    bool limited = false;

    // AC_PID doesn't initialise _integrator, it relies on ArduPilot's zeroing new. Here it's on the stack.
    SpeedLimiter() { pid.reset_I(); }

    Quaternion apply(const Quaternion &target, const Vector2f &vel)
    {
        return AC_AttitudeControl::speed_limit_update(pid, limit, target, vel, limited);
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

}  // namespace

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
    EXPECT_LE(s.max_speed, LIMIT * MAX_OVERSHOOT);
    EXPECT_NEAR(s.mean_speed_last_10s, LIMIT, MEAN_TOL);
    EXPECT_TRUE(limiter.limited);
}

TEST(SpeedLimitClosedLoop, SettlesQuicklyAtLimit)
{
    // within +-2% of the limit by 5s and stays there (default gains do it in 2.8s, half gains take 9s)
    PointMassCopter copter;
    SpeedLimiter limiter;
    const Quaternion pilot = attitude_deg(0, -30, 0);
    float last_outside = 0.0f;
    for (int i = 0; i < int(30.0f / LOOP_DT); i++) {
        copter.step(limiter.apply(pilot, copter.vel));
        if (fabsf(copter.vel.length() - LIMIT) > 0.02f * LIMIT) {
            last_outside = i * LOOP_DT;
        }
    }
    EXPECT_LT(last_outside, 5.0f);
}

TEST(SpeedLimitClosedLoop, IntegratorDecaysWhenIdle)
{
    // I builds up while holding the limit (~0.19 here) and must bleed off once the limiter stops
    // acting, otherwise the next approach starts with a stale allowance
    PointMassCopter copter;
    SpeedLimiter limiter;
    fly(copter, &limiter, attitude_deg(0, -30, 0), 60);
    ASSERT_GT(fabsf(limiter.pid.get_i()), 0.05f);
    fly(copter, &limiter, attitude_deg(0, 20, 0), 3);       // brake
    fly(copter, &limiter, attitude_deg(0, 0, 0), 10);       // then idle
    EXPECT_NEAR(limiter.pid.get_i(), 0.0f, 0.01f);
}

TEST(SpeedLimitClosedLoop, DoesNotInterfereBelowLimit)
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

TEST(SpeedLimitClosedLoop, NoOvershootAfterLongHover)
{
    // hovering means a big speed error for a long time, so this is where I windup shows up
    PointMassCopter copter;
    SpeedLimiter limiter;
    fly(copter, &limiter, attitude_deg(0, 0, 0), 120);
    EXPECT_NEAR(limiter.pid.get_i(), 0.0f, 1.0e-3f);
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 40);
    EXPECT_LE(s.max_speed, LIMIT * MAX_OVERSHOOT);
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
    EXPECT_LE(s.max_speed, LIMIT * MAX_OVERSHOOT);
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

TEST(SpeedLimitClosedLoop, TailwindAboveLimitIsNotHeld)
{
    // the limiter can only level the vehicle, never tilt it back, so a tailwind faster than the
    // limit carries it over. Known limitation, this pins the behaviour down.
    PointMassCopter copter;
    copter.wind = vel_from_course(15, 0);
    SpeedLimiter limiter;
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 60);
    EXPECT_GT(s.mean_speed_last_10s, LIMIT + 3.0f);
}

TEST(SpeedLimitClosedLoop, RecoversAfterTailwindDrops)
{
    // a tailwind faster than the limit pins the output at the 0 clamp. I mustn't keep winding
    // down while it's pinned, or the vehicle is held well under the limit after the wind drops
    PointMassCopter copter;
    copter.wind = vel_from_course(15, 0);
    SpeedLimiter limiter;
    fly(copter, &limiter, attitude_deg(0, -30, 0), 60);
    EXPECT_LT(fabsf(limiter.pid.get_i()), 0.5f);
    copter.wind.zero();
    fly(copter, &limiter, attitude_deg(0, -30, 0), 5);
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 10);
    EXPECT_NEAR(s.mean_speed_last_10s, LIMIT, MEAN_TOL);
}

TEST(SpeedLimitClosedLoop, RecoversFromGustAboveLimit)
{
    // it can only level the vehicle, so drag does the slowing down, give it time
    PointMassCopter copter;
    SpeedLimiter limiter;
    fly(copter, &limiter, attitude_deg(0, -30, 0), 30);
    copter.vel += vel_from_course(4, 0);                    // gust
    fly(copter, &limiter, attitude_deg(0, -30, 0), 15);
    EXPECT_LE(copter.vel.length(), LIMIT * MAX_OVERSHOOT);
}

namespace {

// EKF velocity with IMU vibration on it. The output predictor integrates the accel, so the
// velocity we see has the vibration (after the 20Hz INS filter) in its derivative, with GPS
// fusion pulling the error back. accel_rms is roughly the rms of that filtered accel noise.
struct NoisyVelocity {
    explicit NoisyVelocity(float accel_rms) : sigma(accel_rms) {}
    float sigma;
    std::mt19937 gen{42};
    std::normal_distribution<float> n{0.0f, 1.0f};
    Vector2f accel_noise;
    Vector2f err;

    Vector2f measure(const Vector2f &vel_true)
    {
        const float alpha = calc_lowpass_alpha_dt(LOOP_DT, 20.0f);
        const float white = sigma * 2.7f;   // gives ~sigma rms after the 20Hz filter
        accel_noise.x += alpha * (white * n(gen) - accel_noise.x);
        accel_noise.y += alpha * (white * n(gen) - accel_noise.y);
        err += (accel_noise - err) * LOOP_DT;
        return vel_true + err;
    }
};

struct ChatterStats {
    float max_speed = 0.0f;
    float max_tilt_step = 0.0f;     // largest one-loop change in commanded along-track tilt, rad
    float toggles_per_s = 0.0f;
};

// hold full forward at the limit with a noisy velocity estimate, stats over the last 30s
ChatterStats fly_noisy(SpeedLimiter &limiter, float accel_rms)
{
    PointMassCopter copter;
    NoisyVelocity est(accel_rms);
    const Quaternion pilot = attitude_deg(0, -30, 0);
    ChatterStats s;
    bool last_limited = false;
    float last_tilt = 0.0f;
    int toggles = 0;
    const int steps = int(60.0f / LOOP_DT);
    const int start = int(30.0f / LOOP_DT);
    for (int i = 0; i < steps; i++) {
        const Quaternion cmd = limiter.apply(pilot, est.measure(copter.vel));
        const float tilt = tilt_along(cmd, copter.vel);
        if (i > start) {
            s.max_tilt_step = MAX(s.max_tilt_step, fabsf(tilt - last_tilt));
            toggles += (limiter.limited != last_limited) ? 1 : 0;
        }
        last_tilt = tilt;
        last_limited = limiter.limited;
        copter.step(cmd);
        s.max_speed = MAX(s.max_speed, copter.vel.length());
    }
    s.toggles_per_s = toggles / 30.0f;
    return s;
}

}  // namespace

TEST(SpeedLimitRobustness, HoldsLimitWithNoisyVelocity)
{
    SpeedLimiter limiter;
    const ChatterStats s = fly_noisy(limiter, 1.0f);
    EXPECT_LE(s.max_speed, LIMIT * 1.05f);
}

// Known issue: the D term works on the unfiltered derivative of the velocity estimate, so near
// the limit IMU noise flips the limiter on and off and the attitude target jumps by up to the
// full pilot tilt in one loop. Re-enable once there's a D/error filter default (ATC_SPD_LIM_FLTD).
TEST(SpeedLimitRobustness, DISABLED_NoisyVelocityDoesNotChatter)
{
    SpeedLimiter limiter;
    const ChatterStats s = fly_noisy(limiter, 1.0f);
    EXPECT_LE(degrees(s.max_tilt_step), 2.0f);
    EXPECT_LE(s.toggles_per_s, 1.0f);
}

TEST(SpeedLimitRobustness, FilteringDTermReducesChatter)
{
    SpeedLimiter unfiltered;
    SpeedLimiter filtered;
    filtered.pid.filt_D_hz().set(2.0f);
    const ChatterStats a = fly_noisy(unfiltered, 1.0f);
    const ChatterStats b = fly_noisy(filtered, 1.0f);
    EXPECT_LT(b.max_tilt_step, 0.5f * a.max_tilt_step);
    EXPECT_LT(b.toggles_per_s, 0.5f * a.toggles_per_s);
    EXPECT_LE(b.max_speed, LIMIT * 1.05f);
}

// Known issue: the derating is applied after input shaping, so if the limiter comes on while
// well above the limit (ATC_SPD_LIM_EN set in flight, velocity estimate coming back after a
// dropout) the target steps straight to level in one loop, ignoring ATC_ACCEL_P/R_MAX.
TEST(SpeedLimitRobustness, DISABLED_EngagingAboveLimitIsGradual)
{
    PointMassCopter copter;
    fly(copter, nullptr, attitude_deg(0, -30, 0), 30);       // limiter off, well over the limit
    ASSERT_GT(copter.vel.length(), 1.5f * LIMIT);
    SpeedLimiter limiter;
    const Quaternion pilot = attitude_deg(0, -30, 0);
    float last_tilt = tilt_along(pilot, copter.vel);
    for (int i = 0; i < int(1.0f / LOOP_DT); i++) {
        const Quaternion cmd = limiter.apply(pilot, copter.vel);
        const float tilt = tilt_along(cmd, copter.vel);
        ASSERT_LE(degrees(fabsf(tilt - last_tilt)), 5.0f) << "step at loop " << i;
        last_tilt = tilt;
        copter.step(cmd);
    }
}

TEST(SpeedLimitRobustness, BrakingIsNeverRestricted)
{
    // above the limit and the pilot pulls back: their braking tilt goes through untouched
    PointMassCopter copter;
    fly(copter, nullptr, attitude_deg(0, -30, 0), 30);
    ASSERT_GT(copter.vel.length(), 1.5f * LIMIT);
    SpeedLimiter limiter;
    const Quaternion pilot = attitude_deg(0, 20, 0);
    for (int i = 0; i < int(5.0f / LOOP_DT) && copter.vel.length() > 1.0f; i++) {
        const Quaternion cmd = limiter.apply(pilot, copter.vel);
        ASSERT_TRUE(quat_equal(cmd, pilot)) << "braking changed at v=" << copter.vel.length();
        copter.step(cmd);
    }
}

TEST(SpeedLimitRobustness, HeadwindDoesNotRestrictPenetration)
{
    // ground speed stays under the limit into a strong headwind, so full tilt is allowed
    PointMassCopter copter;
    copter.wind = vel_from_course(15, 180);
    SpeedLimiter limiter;
    const Quaternion pilot = attitude_deg(0, -30, 0);
    for (int i = 0; i < int(40.0f / LOOP_DT); i++) {
        const Quaternion cmd = limiter.apply(pilot, copter.vel);
        ASSERT_TRUE(quat_equal(cmd, pilot)) << "limited at ground speed " << copter.vel.length();
        copter.step(cmd);
    }
    EXPECT_LT(copter.vel.length(), LIMIT);
}

TEST(SpeedLimitRobustness, LoweringLimitInFlight)
{
    PointMassCopter copter;
    SpeedLimiter limiter;
    fly(copter, &limiter, attitude_deg(0, -30, 0), 30);
    limiter.limit = 0.5f * LIMIT;
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 30);
    EXPECT_NEAR(s.mean_speed_last_10s, 0.5f * LIMIT, MEAN_TOL);
}

TEST(SpeedLimitRobustness, RaisingLimitInFlight)
{
    PointMassCopter copter;
    SpeedLimiter limiter;
    limiter.limit = 0.5f * LIMIT;
    fly(copter, &limiter, attitude_deg(0, -30, 0), 30);
    limiter.limit = LIMIT;
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 40);
    EXPECT_LE(s.max_speed, LIMIT * MAX_OVERSHOOT);
    EXPECT_NEAR(s.mean_speed_last_10s, LIMIT, MEAN_TOL);
}

// These document parameter values that make the vehicle unusable. They pass today; they're
// here so a pre-arm check can be added against them.
TEST(SpeedLimitParams, ZeroLimitStopsTranslation)
{
    PointMassCopter copter;
    SpeedLimiter limiter;
    limiter.limit = 0.0f;
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 30);
    EXPECT_LT(s.final_speed, 0.5f);
}

TEST(SpeedLimitParams, ZeroGainsStopTranslation)
{
    PointMassCopter copter;
    SpeedLimiter limiter;
    limiter.pid.kP().set(0);
    limiter.pid.kI().set(0);
    limiter.pid.kD().set(0);
    const RunStats s = fly(copter, &limiter, attitude_deg(0, -30, 0), 30);
    EXPECT_LT(s.final_speed, 0.5f);
}

AP_GTEST_MAIN()
