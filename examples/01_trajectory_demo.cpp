#include "sensorsim/core/trajectory.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace sensorsim;

void saveTrajectoryCsv(const Trajectory& traj, const std::string& filename) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }
    
    // CSV header
    file << "time,px,py,pz,vx,vy,vz,ax,ay,az,qw,qx,qy,qz,wx,wy,wz\n";
    
    file << std::fixed << std::setprecision(6);
    
    for (const auto& state : traj) {
        file << state.time << ","
             << state.position(0) << "," << state.position(1) << "," << state.position(2) << ","
             << state.velocity(0) << "," << state.velocity(1) << "," << state.velocity(2) << ","
             << state.acceleration(0) << "," << state.acceleration(1) << "," << state.acceleration(2) << ","
             << state.orientation.w() << "," << state.orientation.x() << "," 
             << state.orientation.y() << "," << state.orientation.z() << ","
             << state.angular_velocity(0) << "," << state.angular_velocity(1) << "," 
             << state.angular_velocity(2) << "\n";
    }
    
    file.close();
    std::cout << "Saved trajectory to " << filename << std::endl;
}

void printTrajectoryStats(const Trajectory& traj, const std::string& name) {
    std::cout << "\n" << name << " Statistics:" << std::endl;
    std::cout << "  Number of points: " << traj.size() << std::endl;
    std::cout << "  Duration: " << traj.back().time << " seconds" << std::endl;
    
    // Calculate average speed
    Real avg_speed = 0.0;
    for (const auto& state : traj) {
        avg_speed += state.velocity.norm();
    }
    avg_speed /= traj.size();
    std::cout << "  Average speed: " << avg_speed << " m/s" << std::endl;
    
    // Calculate position bounds
    Vec3 min_pos = traj[0].position;
    Vec3 max_pos = traj[0].position;
    for (const auto& state : traj) {
        for (int i = 0; i < 3; ++i) {
            min_pos(i) = std::min(min_pos(i), state.position(i));
            max_pos(i) = std::max(max_pos(i), state.position(i));
        }
    }
    std::cout << "  Position bounds:" << std::endl;
    std::cout << "    X: [" << min_pos(0) << ", " << max_pos(0) << "]" << std::endl;
    std::cout << "    Y: [" << min_pos(1) << ", " << max_pos(1) << "]" << std::endl;
    std::cout << "    Z: [" << min_pos(2) << ", " << max_pos(2) << "]" << std::endl;
}

int main() {
    std::cout << "=== Trajectory Demo ===" << std::endl << std::endl;
    
    Real duration = 10.0;  // seconds
    Real dt = 0.01;        // 100 Hz
    
    TrajectoryGenerator gen(duration, dt);
    
    std::cout << "Trajectory Generator Configuration:" << std::endl;
    std::cout << "  Duration: " << duration << " seconds" << std::endl;
    std::cout << "  Time step: " << dt << " seconds" << std::endl;
    std::cout << "  Frequency: " << (1.0 / dt) << " Hz" << std::endl;
    std::cout << "  Total points: " << gen.numSteps() << std::endl;
    
    // Generate different trajectories
    std::cout << "\n--- Generating Trajectories ---" << std::endl;
    
    std::cout << "\n1. Circular trajectory..." << std::endl;
    Trajectory circular = gen.circular(5.0, 0.5, 0.0);
    saveTrajectoryCsv(circular, "../results/circular.csv");
    printTrajectoryStats(circular, "Circular");
    
    std::cout << "\n2. Figure-8 trajectory..." << std::endl;
    Trajectory figure8 = gen.figureEight(10.0, 10.0, 0.0);
    saveTrajectoryCsv(figure8, "../results/figure8.csv");
    printTrajectoryStats(figure8, "Figure-8");
    
    std::cout << "\n3. Straight line trajectory..." << std::endl;
    Vec3 start(0, 0, 0);
    Vec3 velocity(1, 1, 0);
    Trajectory straight = gen.straightLine(start, velocity);
    saveTrajectoryCsv(straight, "../results/straight.csv");
    printTrajectoryStats(straight, "Straight Line");
    
    std::cout << "\n4. Hover with rotation..." << std::endl;
    Trajectory hover = gen.hoverWithRotation(Vec3(0, 0, 1), 0.3);
    saveTrajectoryCsv(hover, "../results/hover.csv");
    printTrajectoryStats(hover, "Hover with Rotation");
    
    std::cout << "\n5. Random walk..." << std::endl;
    Trajectory random = gen.randomWalk(0.1, 42);
    saveTrajectoryCsv(random, "../results/random_walk.csv");
    printTrajectoryStats(random, "Random Walk");
    
    std::cout << "\n=== Demo Complete ===" << std::endl;
    std::cout << "✓ All trajectories generated successfully!" << std::endl;
    std::cout << "Check the results/ directory for CSV files." << std::endl;
    std::cout << "\nTo visualize, run:" << std::endl;
    std::cout << "  cd results" << std::endl;
    std::cout << "  python plot_trajectories.py" << std::endl;
    
    return 0;
}
