#include "ovelo/ovelo.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

// Deterministic ideal differential-drive simulation, not a real friction model.
// This closes the loop through simulated tracking wheels AND the odometry core.
int main() {
    ovelo::Odometry odom;
    ovelo::MotionController motion;
    ovelo::Pose truth{};
    ovelo::SensorSample sensors{};
    odom.update(sensors);
    motion.moveToPoint({24,24}, odom.pose(),0,6000);
    ovelo::DriveCommand command{};
    std::cout << "time_ms,x_in,y_in,theta_deg,left,right,status\n";
    for (std::uint32_t time=10; time<=6000; time+=10) {
        constexpr double dt=0.01, speed=40, driveWidth=12;
        const double forward=(command.left+command.right)/2*speed*dt;
        const double angle=(command.right-command.left)*speed/driveWidth*dt;
        const double mid=truth.theta+angle/2;
        truth.x += ovelo::sinc(angle/2)*forward*std::cos(mid);
        truth.y += ovelo::sinc(angle/2)*forward*std::sin(mid);
        truth.theta=ovelo::wrapAngle(truth.theta+angle);
        sensors.left += forward - 5*angle;
        sensors.right += forward + 5*angle;
        sensors.side += -3*angle;
        sensors.heading=truth.theta;
        sensors.timeMs=time;
        if (!odom.update(sensors)) { std::cerr << "Odometry rejected simulation\n"; return 1; }
        command=motion.step(odom.pose(),time);
        if (time%100==0 || !motion.running()) {
            const auto p=odom.pose();
            std::cout << std::fixed << std::setprecision(3) << time << ',' << p.x << ',' << p.y << ','
                      << ovelo::degrees(p.theta) << ',' << command.left << ',' << command.right << ','
                      << ovelo::statusName(motion.status()) << '\n';
        }
        if (!motion.running()) break;
    }
    if (motion.status()!=ovelo::MotionStatus::Settled ||
        ovelo::distance({truth.x,truth.y},{24,24})>motion.config().positionTolerance ||
        ovelo::distance({truth.x,truth.y},{odom.pose().x,odom.pose().y})>1e-6) {
        std::cerr << "Closed-loop simulation did not settle correctly\n";
        return 1;
    }
    return 0;
}
