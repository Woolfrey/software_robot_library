/**
 * @file    TrapezoidalVelocity.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 1.0
 * @brief   A class with a trapezoidal velocity profile.
 * 
 * @details This class generates a trajectory using a trapezoidal velocity profile. It will ramp up
 *          at a given acceleration, and coast at a constant speed, before decelerating.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <Trajectory/TrapezoidalVelocity.h>
#include <vector>

namespace RobotLibrary { namespace Trajectory {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
TrapezoidalBase::TrapezoidalBase(const Eigen::VectorXd &startPosition,
                                 const Eigen::VectorXd &endPosition,
                                 const double          &maxVel,
                                 const double          &maxAccel,
                                 const double          &startTime)
{
    if(startPosition.size() != endPosition.size())
    {
        throw std::invalid_argument("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
                                    "Dimensions of input arguments do not match. "
                                    "The start point had " + std::to_string(startPosition.size()) + " elements, "
                                    "and the end point had " + std::to_string(endPosition.size()) + " elements.");
    }
    else if(maxVel <= 0 or maxAccel <= 0)
    {
        throw std::invalid_argument("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
                                    "Velocity and acceleration arguments were "
                                    + std::to_string(maxVel) + " and " + std::to_string(maxAccel) +
                                    " but must be positive.");
    }
     
    // Assign the initial values in the base class
    _dimensions = startPosition.size();

    _startPoint.position     = startPosition;
    _startPoint.velocity     = Eigen::VectorXd::Zero(_dimensions);
    _startPoint.acceleration = Eigen::VectorXd::Zero(_dimensions);

    _endPoint.position     = endPosition;
    _endPoint.velocity     = Eigen::VectorXd::Zero(_dimensions);
    _endPoint.acceleration = Eigen::VectorXd::Zero(_dimensions);

    _startTime = startTime;

    double normaliser = -1;                                                                         // Needed to scale velocity to be dimensionless

    double vel = maxVel;                                                                            // Temporary placeholder

    for(int i = 0; i < _dimensions; i++)
    {
        double distance = abs(startPosition(i) - endPosition(i));                                   // Magnitude of distance between points
      
        normaliser = (distance > normaliser) ? distance : normaliser;                               // Normaliser must be the largest distance
      
        double temp = sqrt(maxAccel*distance);                                                      // Peak velocity as a function of max acceleration
      
        vel = (vel > temp) ? temp : vel;                                                            // Ensure max velocity does not exceed acceleration, distance constraints
    }

    _normalisedVel = vel/normaliser;                                                                // Need to normalise so we can scale between 0 and 1

    _normalisedAcc = maxAccel/normaliser;                                                           // Scale the acceleration to match the velocity

    _rampTime = vel/maxAccel;                                                                       // Length of time to accelerate to max speed

    _rampDistance = 0.5*_normalisedAcc*_rampTime*_rampTime;                                         // s = 0.5*a*t^2

    _coastTime = (1.0 - 2*_rampDistance)/_normalisedVel;                          // Time spent moving at max speed

    _coastDistance = _normalisedVel*_coastTime;                                   // Distance travelled moving at max speed

    _endTime = _startTime + 2*_rampTime + _coastTime;                       // Total time passed
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Trajectory::State 
TrapezoidalBase::query_state(const double &time)
{
         if(time <= _startTime) return _startPoint;
    else if(time >= _endTime)   return _endPoint;
    else
    {
        RobotLibrary::Trajectory::State state;                                                      // Value to be returned
        
        double elapsedTime = time - _startTime;                                                     // As it says

        double s, sd, sdd;                                                                          // Interpolating scalars

        if(elapsedTime < _rampTime)
        {
              s = _normalisedAcc*elapsedTime*elapsedTime/2.0;
             sd = _normalisedAcc*elapsedTime;
            sdd = _normalisedAcc;
        }
        else if(elapsedTime < _rampTime + _coastTime)
        {
              s = _rampDistance + _normalisedVel*(elapsedTime - _rampTime);
             sd = _normalisedVel;
            sdd = 0.0;
        }
        else
        {
            double t = elapsedTime - _coastTime - _rampTime;
           
              s =  _rampDistance  + _coastDistance + _normalisedVel*t - _normalisedAcc*t*t/2.0;
             sd =  _normalisedVel - _normalisedAcc*t;
            sdd = -_normalisedAcc;
        }

        // Interpolate the state

        state.position     = (1.0 - s)*_startPoint.position + s*_endPoint.position;
        state.velocity     =  sd*(_endPoint.position - _startPoint.position);
        state.acceleration = sdd*(_endPoint.position - _startPoint.position);
        
        return state;
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
TrapezoidalVelocity::TrapezoidalVelocity(const std::vector<Eigen::VectorXd> &waypoints,
                                         const double &maxVelocity,
                                         const double &maxAcceleration,
                                         const double &startTime)
{ 
    if(waypoints.size() < 2)
    {
          throw std::invalid_argument("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
                                      "A minimum of 2 waypoints is required to generate a trajectory.");
    }
    
    double start = startTime;
    
    for(int i = 0; i < waypoints.size()-1; i++)                                                     // There are n-1 trajectories for n waypoints
    {
        _trajectories.emplace_back(waypoints[i], waypoints[i+1],
                                         maxVelocity, maxAcceleration, start);
        
        start = _trajectories.back().end_time();                                              // Start of next trajectory is the end of this one

    }
    
    _startTime = startTime;
    _endTime = start;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Trajectory::State
TrapezoidalVelocity::query_state(const double &time)
{
    for(int i = 0; i < _trajectories.size()-1; i++)
    {
        if(time < _trajectories[i].end_time())
        {
            return _trajectories[i].query_state(time);                                        // Must be on the ith trajectory
        }
    }
    
    // else
    return _trajectories.back().query_state(time);                                            // Final trajectory
}

} }
