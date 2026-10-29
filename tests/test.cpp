#include "ovelo/ovelo.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
int checks = 0;
void check(bool value, const char* name) {
    ++checks;
    if (!value) { std::cerr << "FAIL: " << name << '\n'; std::exit(1); }
}
bool near(double a, double b, double epsilon = 1e-8) { return std::abs(a - b) <= epsilon; }
void mathTests() {
    using namespace ovelo;
    check(near(wrapAngle(radians(361)), radians(1)), "angle wrapping");
    check(near(wheelInches(36000, 2.75), pi * 2.75), "centidegrees conversion");
    check(near(wheelInches(18000, 2, 2), 2 * pi), "sensor/wheel ratio");
    const auto mixed = arcade(1, 1);
    check(near(mixed.left, 0) && near(mixed.right, 1), "arcade preserves ratio");
    check(arcade(std::numeric_limits<double>::quiet_NaN(), 0).left == 0, "invalid drive stops");
    check(near(arcade(1e308, 1e308).right, 1), "large finite mixer inputs cannot overflow");
    check(elapsed(5, 0xfffffffbu) == 10, "rollover arithmetic");
}
void odometryTests() {
    using namespace ovelo;
    Odometry odom;
    check(!odom.healthy(), "uninitialized unhealthy");
    check(odom.update({100, 100, 100, 1, 1000}), "nonzero baseline accepted");
    check(near(odom.pose().x, 0), "baseline never moves pose");
    check(odom.update({101, 101, 100, 1, 1010}), "forward sample");
    check(near(odom.pose().x, 1) && near(odom.pose().y, 0), "relative IMU reference");
    check(odom.update({101, 101, 101, 1, 1020}), "sideways sample");
    check(near(odom.pose().y, 1), "left is positive Y");
    check(odom.reset({5, 7, pi/2}), "pose reset");
    check(odom.update({102, 102, 101, 1, 1030}), "reset preserves sensor baseline");
    check(near(odom.pose().x, 5) && near(odom.pose().y, 8), "reset pose frame");
    check(!odom.reset({0, 0, std::numeric_limits<double>::quiet_NaN()}), "invalid reset rejected");

    for (const bool imu : {false, true}) {
        OdometryConfig config; config.useImu = imu;
        Odometry rotate(config);
        check(rotate.update({0, 0, 0, 0, 0}), "rotation baseline");
        const double angle = 0.1;
        check(rotate.update({-5*angle, 5*angle, -3*angle, angle, 10}), "CCW pure rotation");
        check(near(rotate.pose().x, 0) && near(rotate.pose().y, 0), "offset compensation");
        check(near(rotate.pose().theta, angle), "CCW sign with/without IMU");
        check(near(rotate.trace().forward, 0) && near(rotate.trace().dtheta, angle), "explainable trace");
        Odometry arc(config);
        arc.update({0,0,0,0,0});
        arc.update({1-5*angle,1+5*angle,-3*angle,angle,10});
        check(near(arc.pose().x, std::sin(angle)/angle) &&
              near(arc.pose().y, (1-std::cos(angle))/angle), "finite arc chord integration");
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    OdometryConfig two; two.useRight = false;
    Odometry dual(two);
    check(dual.update({0,nan,0,0,0}) && dual.update({0.5,nan,0,0,10}), "2 wheels ignore right");
    check(near(dual.pose().x, 0.5), "2 wheel translation");
    two.useImu = false;
    check(!Odometry(two).validConfig(), "2 wheels require heading source");
    OdometryConfig impossible; impossible.useImu = false; impossible.rightOffset = impossible.leftOffset;
    check(!Odometry(impossible).update({}), "zero track width rejected");

    Odometry fault;
    fault.update({0,0,0,0,0});
    check(!fault.update({nan,0,0,0,10}) && !fault.healthy(), "NaN rejected");
    check(fault.update({10,10,0,0,20}) && near(fault.pose().x,0), "recovery establishes baseline");
    check(!fault.update({10,10,0,0,20}) && fault.trace().status == OdometryStatus::Stale,
          "duplicate timestamps rejected");
    fault.update({10,10,0,0,30});
    check(!fault.update({10,10,0,0,500}) && fault.trace().status == OdometryStatus::Stale,
          "long gaps rejected");
    fault.update({10,10,0,0,510});
    check(!fault.update({100,100,0,0,520}) && fault.trace().status == OdometryStatus::Jump,
          "sensor jump rejected");
    fault.forgetBaseline();
    check(!fault.healthy() && !fault.initialized(), "explicit stopped mode rebaseline");
    Odometry rollover;
    rollover.update({0,0,0,0,0xfffffffbu});
    check(rollover.update({0.5,0.5,0,0,5}), "odometry timestamp rollover");
    Odometry wrap;
    wrap.update({0,0,0, radians(179),0});
    const double da = radians(2);
    check(wrap.update({-5*da,5*da,-3*da,radians(-179),10}), "IMU wraps at pi");
    check(near(wrap.pose().theta, da), "shortest IMU delta");
}
void motionTests() {
    using namespace ovelo;
    MotionController motion;
    check(motion.step({},10).left == 0, "idle output zero");
    motion.turnTo(pi/2, {},0,4000);
    auto out = motion.step({},10);
    check(out.left < 0 && out.right > 0, "CCW motor signs");
    motion.cancel();
    check(motion.status() == MotionStatus::Cancelled && motion.step({},20).left == 0, "cancel stops");
    motion.moveToPoint({24,0},{},0,2000);
    out = motion.step({},10);
    check(out.left > 0 && near(out.left,out.right), "drive forward");
    motion.moveToPoint({-24,0},{},0,2000);
    out = motion.step({},10);
    check(out.left * out.right < 0, "aim before driving behind robot");
    for (std::uint32_t time=20; time<=1010; time+=10) out = motion.step({},time);
    check(motion.status() == MotionStatus::Stalled && out.left == 0 && out.right == 0, "stall stops");
    motion.moveToPoint({1,0},{},0,20);
    motion.step({},10); out = motion.step({},20);
    check(motion.status() == MotionStatus::TimedOut && out.left == 0, "timeout stops");
    motion.turnTo(0,{},0,1000);
    for (std::uint32_t time=10; time<=160; time+=10) out = motion.step({},time);
    check(motion.status() == MotionStatus::Settled && out.right == 0, "settling window");
    motion.moveToPoint({0,0},{},0,1000);
    motion.step({},10); motion.step({3,0,0},20); motion.step({},30);
    check(motion.status() == MotionStatus::Running, "leaving tolerance resets settle timer");
    motion.step({},170);
    check(motion.status() == MotionStatus::Invalid, "stale controller step stops");
    motion.moveToPoint({std::numeric_limits<double>::infinity(),0},{},0,1000);
    check(motion.status() == MotionStatus::Invalid, "nonfinite goal rejected");
    motion.turnTo(0,{},0,1000); motion.step({},0);
    check(motion.status() == MotionStatus::Invalid, "duplicate control time rejected");
    motion.moveToPoint({1e308,1e308},{-1e308,-1e308,0},0,1000);
    motion.step({-1e308,-1e308,0},10);
    check(motion.status() == MotionStatus::Invalid, "overflow goal stops");
    MotionConfig bad; bad.maxDrive = 3;
    MotionController invalid(bad); invalid.turnTo(0,{},0,100);
    check(invalid.status() == MotionStatus::Invalid, "invalid controller configuration");
    motion.turnTo(0,{},0xfffffffbu,1000);
    motion.step({},5);
    check(motion.status() == MotionStatus::Running, "controller rollover");
}
void diagnosticsTests() {
    using namespace ovelo;
    check(diagnose(HandTest::Forward,{}, {2,2,0,0,100}).status == DiagnosticStatus::Pass,
          "forward direction report");
    check(diagnose(HandTest::Forward,{}, {-2,2,0,0,100}).status == DiagnosticStatus::WrongDirection,
          "reversed wheel report");
    check(diagnose(HandTest::Left,{}, {0,0,0,0,100}).status == DiagnosticStatus::NoMotion,
          "missing sideways motion report");
    check(diagnose(HandTest::Counterclockwise,{}, {-0.5,0.5,-0.3,0.1,100}).status == DiagnosticStatus::Pass,
          "rotation report");
}
} // namespace
int main() {
    mathTests(); odometryTests(); motionTests(); diagnosticsTests();
    std::cout << checks << " portable core checks passed\n";
}
