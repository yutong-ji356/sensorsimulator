#include "sensorsim/core/trajectory.hpp"
#include "sensorsim/utils/quaternion.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace sensorsim;

// Helper function for approximate equality
bool approxEqual(Real a, Real b, Real tol = 1e-4) {
    return std::abs(a - b) < tol;
}

bool approxEqual(const Vec3& a, const Vec3& b, Real tol = 1e-4) {
    return (a - b).norm() < tol;
}

void testCircularTrajectory() {
    std::cout << "Testing circular trajectory..." << std::endl;
    
    Real duration = 10.0;
    Real dt = 0.01;
    TrajectoryGenerator gen(duration, dt);
    
    Real radius = 5.0;
    Real angular_vel = 0.5;  // rad/s
    
    Trajectory traj = gen.circular(radius, angular_vel, 0.0);
    
    // Check basic properties
    assert(traj.size() == gen.numSteps());
    
    // Check first state
    const State& s0 = traj[0];
    assert(approxEqual(s0.position(0), radius));  // Starts at (radius, 0, 0)
    assert(approxEqual(s0.position(1), 0.0));
    
    // Check that object stays at constant radius
    for (size_t i = 0; i < traj.size(); ++i) {
        Real r = std::sqrt(traj[i].position(0) * traj[i].position(0) + 
                          traj[i].position(1) * traj[i].position(1));
        assert(approxEqual(r, radius, 1e-3));
    }
    
    // Check angular velocity is constant
    for (size_t i = 0; i < traj.size(); ++i) {
        assert(approxEqual(traj[i].angular_velocity(2), angular_vel, 1e-3));
    }
    
    std::cout << "  ✓ Circular trajectory passed" << std::endl;
}

void testStraightLineTrajectory() {
    std::cout << "Testing straight line trajectory..." << std::endl;
    
    Real duration = 5.0;
    Real dt = 0.01;
    TrajectoryGenerator gen(duration, dt);
    
    Vec3 start(0, 0, 0);
    Vec3 velocity(1, 2, 0);  // m/s
    
    Trajectory traj = gen.straightLine(start, velocity);
    
    // Check velocity is constant
    for (size_t i = 0; i < traj.size(); ++i) {
        assert(approxEqual(traj[i].velocity, velocity));
    }
    
    // Check acceleration is zero (constant velocity)
    for (size_t i = 1; i < traj.size() - 1; ++i) {
        assert(approxEqual(traj[i].acceleration.norm(), 0.0, 1e-2));
    }
    
    // Check position increases linearly
    const State& s_final = traj.back();
    Vec3 expected_pos = start + velocity * duration;
    assert(approxEqual(s_final.position, expected_pos, 1e-2));
    
    std::cout << "  ✓ Straight line trajectory passed" << std::endl;
}

void testHoverWithRotation() {
    std::cout << "Testing hover with rotation..." << std::endl;
    
    Real duration = 5.0;
    Real dt = 0.01;
    TrajectoryGenerator gen(duration, dt);
    
    Vec3 hover_pos(0, 0, 1);
    Real yaw_rate = 0.3;  // rad/s
    
    Trajectory traj = gen.hoverWithRotation(hover_pos, yaw_rate);
    
    // Check position stays constant
    for (size_t i = 0; i < traj.size(); ++i) {
        assert(approxEqual(traj[i].position, hover_pos));
    }
    
    // Check velocity is zero
    for (size_t i = 0; i < traj.size(); ++i) {
        assert(approxEqual(traj[i].velocity.norm(), 0.0, 1e-3));
    }
    
    // Check angular velocity is constant around z-axis
    for (size_t i = 0; i < traj.size(); ++i) {
        assert(approxEqual(traj[i].angular_velocity(2), yaw_rate, 1e-3));
        assert(approxEqual(traj[i].angular_velocity(0), 0.0, 1e-3));
        assert(approxEqual(traj[i].angular_velocity(1), 0.0, 1e-3));
    }
    
    std::cout << "  ✓ Hover with rotation passed" << std::endl;
}

void testFigureEightTrajectory() {
    std::cout << "Testing figure-8 trajectory..." << std::endl;
    
    Real duration = 10.0;
    Real dt = 0.01;
    TrajectoryGenerator gen(duration, dt);
    
    Trajectory traj = gen.figureEight(10.0, 10.0, 0.0);
    
    // Just verify it generates correct number of points
    assert(traj.size() == gen.numSteps());
    
    // Check that trajectory is roughly centered at origin
    Vec3 mean_pos = Vec3::Zero();
    for (const auto& state : traj) {
        mean_pos += state.position;
    }
    mean_pos /= traj.size();
    
    assert(approxEqual(mean_pos.norm(), 0.0, 1.0));  // Should be near origin
    
    std::cout << "  ✓ Figure-8 trajectory passed" << std::endl;
}

void testKinematicConsistency() {
    std::cout << "Testing kinematic consistency..." << std::endl;
    
    Real duration = 2.0;
    Real dt = 0.01;
    TrajectoryGenerator gen(duration, dt);
    
    Trajectory traj = gen.circular(5.0, 0.5, 0.0);
    
    // Check that velocity matches position derivative (numerically)
    for (size_t i = 1; i < traj.size() - 1; ++i) {
        Vec3 vel_numeric = (traj[i+1].position - traj[i-1].position) / (2.0 * dt);
        assert(approxEqual(traj[i].velocity, vel_numeric, 1e-2));
    }
    
    // Check that acceleration matches velocity derivative
    for (size_t i = 1; i < traj.size() - 1; ++i) {
        Vec3 accel_numeric = (traj[i+1].velocity - traj[i-1].velocity) / (2.0 * dt);
        assert(approxEqual(traj[i].acceleration, accel_numeric, 1e-2));
    }
    
    std::cout << "  ✓ Kinematic consistency passed" << std::endl;
}

int main() {
    std::cout << "=== Trajectory Generation Tests ===" << std::endl << std::endl;
    
    try {
        testCircularTrajectory();
        testStraightLineTrajectory();
        testHoverWithRotation();
        testFigureEightTrajectory();
        testKinematicConsistency();
        
        std::cout << std::endl << "✓ All trajectory tests passed!" << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "✗ Test failed: " << e.what() << std::endl;
        return 1;
    }
}
