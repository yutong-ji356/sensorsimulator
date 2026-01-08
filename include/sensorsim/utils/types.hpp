
#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <memory>

namespace sensorsim { //strict

// Scalar types
using Real = double;  // Can switch to float for embedded simulation, type real number
// implementation: Real x = 3.14, (Real = double)

// Vector types
using Vec3 = Eigen::Vector3d; //3x1
using Vec4 = Eigen::Vector4d; //4x1
using VecX = Eigen::VectorXd; //nx1

// Matrix types
using Mat3 = Eigen::Matrix3d; //3x3
using Mat4 = Eigen::Matrix4d; //4x4
using MatX = Eigen::MatrixXd; //mxn
//Vec3 rotated = rotation(Mat3) * point(Vec3) //3x3 x 3x1 = 3x1
// Quaternion
using Quat = Eigen::Quaterniond; //4 nums for 3d rotation

// Trajectory state
struct State {
    Real time;                  // Timestamp in seconds
    Vec3 position;              // Position [x, y, z] in meters
    Vec3 velocity;              // Velocity [vx, vy, vz] in m/s
    Vec3 acceleration;          // Acceleration [ax, ay, az] in m/s²
    Quat orientation;           // Orientation as quaternion
    Vec3 angular_velocity;      // Angular velocity [wx, wy, wz] in rad/s
    
    State() //initial parameters
        : time(0.0)
        , position(Vec3::Zero()) //Vec3::Zero() is for creating [0,0,0] vector
        , velocity(Vec3::Zero())
        , acceleration(Vec3::Zero())
        , orientation(Quat::Identity())
        , angular_velocity(Vec3::Zero()) 
    {}
};

// Sensor measurement types
struct ImuMeasurement { //motion over time
    Real timestamp;
    Vec3 accelerometer;  // m/s² gravity on object = [0,0,9.81] reaction force
    Vec3 gyroscope;      // rad/s
};

struct GpsMeasurement {
    Real timestamp;
    Vec3 position;       // meters
    Real accuracy;       // horizontal accuracy in meters
    bool valid;          // false if dropout
};

// Trajectory - sequence of states
using Trajectory = std::vector<State>;

} 