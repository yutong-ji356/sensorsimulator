#pragma once

#include "sensorsim/utils/types.hpp"
#include <cmath>

//euler <-> quat conversion
namespace sensorsim{ //nested so its sensorsim::quat
    namespace quat{
        //converts euler angles(roll,pitch,yaw) to quat
        //zyx rotation
        inline Quat fromEuler(Real roll, Real pitch, Real yaw){ //inline - appears in a lot of files but it is one function across, static just copies per file
            const Eigen::AngleAxisd roll_x (roll,  Vec3::UnitX());
            const Eigen::AngleAxisd pitch_y(pitch, Vec3::UnitY());
            const Eigen::AngleAxisd yaw_z  (yaw,   Vec3::UnitZ());

            return Quat(yaw_z * pitch_y * roll_x);
        }

        inline Vec3 toEuler(const Quat& q) { //does not modify q
            const Mat3 R = q.toRotationMatrix(); //convert quat to rotation matrix/euler angles, returns rollpitchyaw in rad

            Vec3 euler;
            euler.x() = std::atan2(R(2,1), R(2,2));   // roll
            euler.y() = std::asin(-R(2,0));           // pitch
            euler.z() = std::atan2(R(1,0), R(0,0));   // yaw

            return euler;
        }
        // Create quaternion from yaw only 
        inline Quat fromYaw(Real yaw) {
            return Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ()));
        }

        //Extract yaw angle from quaternion 
        inline Real getYaw(const Quat& q) {
            return toEuler(q).z();
        }

        //quat operations
        inline Quat multiply(const Quat& q1, const Quat& q2){
            return q1*q2;
        }

        inline Quat inverse(const Quat& q){
            return q.inverse(); //function already integrated
        }

        inline Quat slerp(const Quat& q1, const Quat& q2, Real t){
            return q1.slerp(t,q2);
        }

        //quat motion (IMU math)

        //quat derivative
        // q dot = 0.5 * q * [0,wx,wy,wz]<-angular velocity vector in body frame, q dot is time derivative. orientation quat change
        inline Quat derivative(const Quat& q, const Vec3& omega) {
            const Quat omega_q(0.0, omega.x(), omega.y(), omega.z());

            return Quat( //matrix multi
               0.5 * ( q.w()*omega_q.w() - q.x()*omega_q.x()
                     - q.y()*omega_q.y() - q.z()*omega_q.z()),

               0.5 * ( q.w()*omega_q.x() + q.x()*omega_q.w()
                     + q.y()*omega_q.z() - q.z()*omega_q.y()),

               0.5 * ( q.w()*omega_q.y() - q.x()*omega_q.z()
                     + q.y()*omega_q.w() + q.z()*omega_q.x()),

               0.5 * ( q.w()*omega_q.z() + q.x()*omega_q.y()
                     - q.y()*omega_q.x() + q.z()*omega_q.w())
            );
        }

        //quat integration with euler integration
        inline Quat integrate(const Quat& q, const Vec3& omega, Real dt) {
            const Quat q_dot = derivative(q, omega);

            Quat q_next(
                q.w() + q_dot.w() * dt,
                q.x() + q_dot.x() * dt,
                q.y() + q_dot.y() * dt,
                q.z() + q_dot.z() * dt
            );

            q_next.normalize();  // keep unit quaternion
            return q_next;
        }

        //estimate angular velocity from two poistions
        inline Vec3 angularVelocity(const Quat& q1, const Quat& q2, Real dt) {
            const Quat delta = q2 * q1.inverse(); //rotation that occured over dt. 
            const Eigen::AngleAxisd aa(delta); //quat to axis angle = rotation angle around axis

            return aa.axis() * (aa.angle() / dt); //w = theta/dt times axis
        }

        //derivative - given angular velo, how does orientation change
        //angular velocity - given orientation change, what is angular velocity that caused it

        //matrix converstions
        inline Mat3 toRotationMatrix(const Quat& q) {
            return q.toRotationMatrix();
        }

        inline Quat fromRotationMatrix(const Mat3& R) {
            return Quat(R);
        }


        //check quat is unit length
        inline bool isValid(const Quat& q, Real tolerance = 1e-6) {//qnorm = sqrt w^2 + x...y...z..
            return std::abs(q.norm() - 1.0) < tolerance;//<1 true valid quat, >1 false and invalid
        }
    }
}
