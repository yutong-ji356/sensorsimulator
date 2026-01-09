#include "sensorsim/core/trajectory.hpp"
#include <random>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sensorsim {
    //constructor
    TrajectoryGenerator::TrajectoryGenerator(Real duration, Real dt) : duration_(duration), dt_(dt), num_steps_(static_cast<size_t>(duration / dt)){ //based on steps/frames and timestamps
        time_vector_.reserve(num_steps_);
        for (size_t i = 0; i < num_steps_; ++i) {
            time_vector_.push_back(i * dt);
        }
    }

    //circular trajectory
    Trajectory TrajectoryGenerator::circular(Real radius, Real angular_velocity, Real height) const{
        // store position at each time step
        std::vector<Vec3> positions; 
        positions.reserve(num_steps_);
    
        // store orientations (quat) at each time step
        std::vector<Quat> orientations;
        orientations.reserve(num_steps_);
    
        for (size_t i = 0; i < num_steps_; ++i) {
            //current time
            Real t = time_vector_[i];
            Real theta = angular_velocity * t; //angle around circle
        
            // Position on circle
            Vec3 pos(
                radius * std::cos(theta),//x
                radius * std::sin(theta),//y
                height //z
            );
            positions.push_back(pos);
        
            // Orientation: yaw = tangent to circle
            Real yaw = theta + M_PI / 2.0; //theta  + 90 degrees is tangent direction
            orientations.push_back(quat::fromYaw(yaw));
        }
    
        //build full trajectory with the defined orientation
        return buildWithOrientation(positions, orientations);
    }

    //figure eight trajectory
    Trajectory TrajectoryGenerator::figureEight(Real scale, Real period,Real height) const{
        std::vector<Vec3> positions;//store positions
        positions.reserve(num_steps_);

        // Angular frequency ω = 2π / T
        Real omega = 2.0 * M_PI / period;

        for (size_t i = 0; i < num_steps_; ++i) {
            Real t = time_vector_[i];

            // Parametric lemniscate (figure-eight)
            Vec3 pos(
                scale * std::sin(omega * t),
                scale * std::sin(2.0 * omega * t) / 2.0,
                height
            );
            positions.push_back(pos);
        }

        // Orientation will be inferred from velocity
        return buildFromPosition(positions);
    }

    Trajectory TrajectoryGenerator::straightLine(const Vec3& start,const Vec3& velocity) const{
        std::vector<Vec3> positions;
        positions.reserve(num_steps_);

        for (size_t i = 0; i < num_steps_; ++i) {
            Real t = time_vector_[i];

            // x(t) = x0 + v * t
            positions.push_back(start + velocity * t);
        }

        // Orientation faces direction of velocity
        Real yaw = std::atan2(velocity.y(), velocity.x());

        // Same orientation at every time step
        std::vector<Quat> orientations(num_steps_, quat::fromYaw(yaw));

        return buildWithOrientation(positions, orientations);
    }

    Trajectory TrajectoryGenerator::randomWalk(Real step_size,unsigned int seed) const{
        // random number generator (use random seed if seed == 0)
        std::mt19937 rng(seed == 0 ? std::random_device{}() : seed); //if seed = 0 it is truly random

        // Gaussian noise with std deviation = step_size
        std::normal_distribution<Real> dist(0.0, step_size);

        std::vector<Vec3> positions;
        positions.reserve(num_steps_);

        // Start at origin
        Vec3 pos = Vec3::Zero();
        positions.push_back(pos);

        for (size_t i = 1; i < num_steps_; ++i) {
            // Random step in x, y, z
            Vec3 step(dist(rng), dist(rng), dist(rng));
            pos += step;
            positions.push_back(pos);
        }

        return buildFromPosition(positions);
    }

    Trajectory TrajectoryGenerator::hoverWithRotation(const Vec3& position,Real yaw_rate) const{
        // Position is constant at every time step
        std::vector<Vec3> positions(num_steps_, position);

        std::vector<Quat> orientations;
        orientations.reserve(num_steps_);

        for (size_t i = 0; i < num_steps_; ++i) {
            // Yaw increases linearly: ψ(t) = ωt
            Real yaw = yaw_rate * time_vector_[i];
            orientations.push_back(quat::fromYaw(yaw));
        }

        return buildWithOrientation(positions, orientations);
    } 

    //velocity
    std::vector<Vec3> TrajectoryGenerator::computeVelocity(const std::vector<Vec3>& position) const{
        std::vector<Vec3> velocity;
        velocity.reserve(num_steps_);

        // Forward difference at first sample
        // v(0) ≈ [p(1) - p(0)] / dt
        velocity.push_back((position[1] - position[0]) / dt_);

        // Central difference for interior points
        // v(i) ≈ [p(i+1) - p(i-1)] / (2·dt)
        for (size_t i = 1; i < num_steps_ - 1; ++i) {
            velocity.push_back((position[i+1] - position[i-1]) / (2.0 * dt_));
        }

        // Backward difference at last sample
         // v(n-1) ≈ [p(n-1) - p(n-2)] / dt
        velocity.push_back((position[num_steps_-1] - position[num_steps_-2]) / dt_);

        return velocity;
    }

    //acceleration
    std::vector<Vec3> TrajectoryGenerator::computeAcceleration(
    const std::vector<Vec3>& velocity) const
    {
    std::vector<Vec3> acceleration;
    acceleration.reserve(num_steps_);
    
    // FORWARD DIFFERENCE (first point)
    // a(0) ≈ [v(1) - v(0)] / dt
    acceleration.push_back((velocity[1] - velocity[0]) / dt_);
    
    // CENTRAL DIFFERENCE (middle points)
    // a(i) ≈ [v(i+1) - v(i-1)] / (2·dt)
    for (size_t i = 1; i < num_steps_ - 1; ++i) {
        acceleration.push_back((velocity[i+1] - velocity[i-1]) / (2.0 * dt_));
    }
    
    // BACKWARD DIFFERENCE (last point)
    // a(n-1) ≈ [v(n-1) - v(n-2)] / dt
    acceleration.push_back((velocity[num_steps_-1] - velocity[num_steps_-2]) / dt_);
    
    return acceleration;
    }
 
    std::vector<Quat> TrajectoryGenerator::computeOrientation(
    const std::vector<Vec3>& velocity) const
    {
    std::vector<Quat> orientation;
    orientation.reserve(num_steps_);
    
    for (size_t i = 0; i < num_steps_; ++i) {
        Vec3 v = velocity[i];
        
        // MAGNITUDE OF VELOCITY
        // .norm() is Eigen's function for vector magnitude: √(x² + y² + z²)
        Real speed = v.norm();
        
        // EDGE CASE
        if (speed < 1e-6) {
            // Object not moving - maintain previous orientation
            if (i > 0) {
                // Copy previous orientation (smooth)
                orientation.push_back(orientation[i-1]);
            } else {
                // First point with no velocity: use identity (no rotation)
                orientation.push_back(Quat::Identity());
            }
        } else {
            // MOVING: face direction of motion
            // Extract yaw (rotation around Z-axis) from velocity vector
            // atan2(y, x) gives angle in X-Y plane
            Real yaw = std::atan2(v.y(), v.x());
            
            // Create quaternion representing rotation around Z-axis
            // Roll and pitch remain 0 (object stays level)
            orientation.push_back(quat::fromYaw(yaw));
        }
    }
    
    return orientation;
    }
    

    //angular velocity from orientation
    std::vector<Vec3> TrajectoryGenerator::computeAngularVelocity(
    const std::vector<Quat>& orientation) const
    {
    std::vector<Vec3> angular_velocity;
    angular_velocity.reserve(num_steps_);
    
    // FIRST POINT (forward difference)
    // Compare orientation[0] to orientation[1]
    // quat::angularVelocity() computes:
    // 1. Rotation difference: Δq = q₂ × q₁⁻¹
    // 2. Convert to axis-angle: (axis, angle)
    // 3. Angular velocity: ω = axis × (angle / dt)
    angular_velocity.push_back(quat::angularVelocity(
        orientation[0], orientation[1], dt_));
    
    // MIDDLE POINTS
    // Uses forward difference (could use central, but forward is simpler here)
    // ω(i) ≈ angle_from(q[i], q[i+1]) / dt
    for (size_t i = 1; i < num_steps_ - 1; ++i) {
        angular_velocity.push_back(quat::angularVelocity(
            orientation[i], orientation[i+1], dt_));
    }
    
    // LAST POINT (backward difference)
    // Compare orientation[n-2] to orientation[n-1]
    angular_velocity.push_back(quat::angularVelocity(
        orientation[num_steps_-2], orientation[num_steps_-1], dt_));
    
    return angular_velocity;
}


// TRAJECTORY ASSEMBLY METHODS
//These methods combine position, velocity, acceleration, orientation, and
// angular velocity into complete State objects
    
//traj from position only
Trajectory TrajectoryGenerator::buildFromPosition(
    const std::vector<Vec3>& position) const
{
    // COMPUTE ALL DERIVATIVES
    // 'auto' = automatic type deduction (std::vector<Vec3>)
    auto velocity = computeVelocity(position);           // First derivative
    auto acceleration = computeAcceleration(velocity);   // Second derivative
    auto orientation = computeOrientation(velocity);     // From velocity direction
    auto angular_velocity = computeAngularVelocity(orientation);  // From orientation change
    
    // ASSEMBLE TRAJECTORY
    Trajectory traj;  // Trajectory is typedef for std::vector<State>
    traj.reserve(num_steps_);  // Pre-allocate to avoid reallocations
    
    // BUILD EACH STATE
    for (size_t i = 0; i < num_steps_; ++i) {
        // Create State struct (defined in types.hpp)
        State state;
        
        // FILL STATE FIELDS
        // Each State contains complete information at one instant in time
        state.time = time_vector_[i];       // Timestamp
        state.position = position[i];       // 3D position [m]
        state.velocity = velocity[i];       // 3D velocity [m/s]
        state.acceleration = acceleration[i]; // 3D acceleration [m/s²]
        state.orientation = orientation[i];  // Quaternion (rotation)
        state.angular_velocity = angular_velocity[i];  // Angular rate [rad/s]
        
        // Add to trajectory sequence
        traj.push_back(state);
    }
    
    return traj;
}

//build trajectory with know orientation, not compute from velocity
    Trajectory TrajectoryGenerator::buildWithOrientation(
    const std::vector<Vec3>& position,
    const std::vector<Quat>& orientation) const
{
    // COMPUTE DERIVATIVES
    // no compute orientation since given
    auto velocity = computeVelocity(position);
    auto acceleration = computeAcceleration(velocity);
    auto angular_velocity = computeAngularVelocity(orientation);  // Use provided orientation
    
    // ASSEMBLE TRAJECTORY
    Trajectory traj;
    traj.reserve(num_steps_);
    
    // BUILD EACH STATE
    for (size_t i = 0; i < num_steps_; ++i) {
        State state;
        state.time = time_vector_[i];
        state.position = position[i];
        state.velocity = velocity[i];
        state.acceleration = acceleration[i];
        state.orientation = orientation[i];  // Use provided orientation
        state.angular_velocity = angular_velocity[i];
        traj.push_back(state);
    }
    
    return traj;
}



    }
    
