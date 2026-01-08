#pragma once

#include "types.hpp"
#include "quaternion.hpp"
#include <vector>
#include <functional>
#include <cmath>

namespace sensorsim {

//Generate ground truth trajectories for testing sensors: pos, velo, acc, ori, w
 
class TrajectoryGenerator {
    public: //can be accessed outside
    TrajectoryGenerator(Real duration, Real dt);

    //getters
    size_t numSteps() const {return num_steps_;}
    Real dt() const { return dt_; }
    Real duration() const { return duration_; }

    //trajectories

    //circular motion
    Trajectory circular(Real radius = 10.0, Real angular_velocity = 0.5, Real height = 0.0) const;

    //figure 8
    Trajectory figureEight(Real radius = 10.0, Real angular_velocity = 0.5, Real height = 0.0) const;

    //straight line
    Trajectory straightLine(const Vec3& start, const Vec3& velocity) const;

    //randomewalk
    Trajectory randomWalk(Real step_size = 0.1,unsigned int seed = 0) const;
    
    //hover and rotating
    Trajectory hoverWithRotation(const Vec3& position = Vec3(0,0,1),Real yaw_rate = 0.3) const;

    private: //internal access
    //initial parameters:
    Real duration_;              // Total trajectory duration
    Real dt_;                    // Time step
    size_t num_steps_;           // Number of steps
    std::vector<Real> time_vector_;  // Precomputed time vector

    //compute velocity from position using position difference
    std::vector<Vec3> computeVelocity(const std::vector<Vec3>& position) const;

    // acceleration from velocity
    std::vector<Vec3> computeAcceleration(const std::vector<Vec3>& velocity) const;

    //orientation(quat) from velocity vectors. objects face direction of motion
    std::vector<Quat> computeOrientation(const std::vector<Vec3>& velocity) const;

    // angular velo from orientation quat
    std::vector<Vec3> computeAngularVelocity(const std::vector<Quat>& orientation) const;

    // build trajectory from positions
    Trajectory buildFromPosition(const std::vector<Vec3>& position) const;

    // build trajectory from positions and precomputed orientations
    Trajectory buildWithOrientation(const std::vector<Vec3>& position,const std::vector<Quat>& orientation) const;

};
   

} // namespace sensorsim