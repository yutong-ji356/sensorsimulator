#include "core/trajectory.hpp"
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
    std::vector<Vec3> TrajectoryGenerator::computeVelocity(const std::vector<Vec3>& position) const{
        std::vector<Vec3> velocity;
        velocity.reserve(num_steps_);

        // Forward difference at first sample
        velocity.push_back((position[1] - position[0]) / dt_);

        // Central difference for interior points
        for (size_t i = 1; i < num_steps_ - 1; ++i) {
            velocity.push_back((position[i+1] - position[i-1]) / (2.0 * dt_));
        }

        // Backward difference at last sample
        velocity.push_back((position[num_steps_-1] - position[num_steps_-2]) / dt_);

        return velocity;
    }




}

