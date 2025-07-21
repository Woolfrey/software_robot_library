/**
 * @file    SplineTrajectory.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   This class gives a spline as a function of time over a finite series of points.
 * 
 * @details Given a set of points and times, this class will generate piece-wise polynomials to interpolate
 *          between them. It ensures continuity at the velocity and acceleration level.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef SPLINE_TRAJECTORY_H_
#define SPLINE_TRAJECTORY_H_

#include <Math/MathFunctions.h>
#include <Math/Spline.h>
#include <Trajectory/TrajectoryBase.h>

namespace RobotLibrary { namespace Trajectory {
/**
 * @brief This class gives a spline as a function of time over a finite series of points.
 */
class SplineTrajectory : public RobotLibrary::Trajectory::TrajectoryBase
{
    public:
        
        /**
         * @brief An empty constructor.
         */
        SplineTrajectory() {}
        
        /**
         * @brief Constructor.
         * @param waypoints The states (position, velocity, acceleration) for given points in time.
         * @param times The time at which to pass through each waypoint.
         * @param polynomialOrder The type of polynomial interpolation to use.
         */
        SplineTrajectory(const std::vector<RobotLibrary::Trajectory::State> &waypoints,
                         const std::vector<double> &times,
                         const unsigned int &polynomialOrder);
        
        /**
         * @brief A basic constructor for a cubic spline where the waypoints and start velocity are given.
         *        The final velocity is assumed to be zero.
         * @param positions An array of positions to pass through.
         * @param times The time at which to pass through each position.
         * @param startVelocity The initial velocity of the trajectory.
         */
        SplineTrajectory(const std::vector<Eigen::VectorXd> &positions,
                         const std::vector<double> &times,
                         const Eigen::VectorXd &startVelocity);
                                                
        /**
         * @brief Query the state for the given input time.
         *        This overrides the virtual function in the TrajectoryBase class.
         * @param time The point at which to evaluate the trajectory.
         * @return The position, velocity, acceleration as a State data structure.
         */
        RobotLibrary::Trajectory::State
        query_state(const double &time);
    
    private:
        
        std::vector<RobotLibrary::Math::Spline> _spline;                                            ///< Underlying spline object over real numbers
        
};                                                                                                  // Semicolon needed after a class declarationS

} } // namespace

#endif
