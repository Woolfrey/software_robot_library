/**
 * @file   TrapezoidalVelocity.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A trajectory with constant velocity.
 */

#ifndef TRAPEZOIDAL_VELOCITY_H_
#define TRAPEZOIDAL_VELOCITY_H_

#include "TrajectoryBase.h"

#include <Eigen/Geometry>                                                                           // Eigen::Vector, Eigen::Matrix, Eigen::Quaternion

/**
 * This class defines a trapezoidal velocity profile between 2 points.
 */
template <class DataType>
class TrapezoidalBase : public TrajectoryBase<DataType>
{
     public:

        /**
         * Emtpy constructor.
         */
        TrapezoidalBase() {}

        /**
         * Full constructor.
         * @param startPoint A vector of positions for the beginning of the trajectory.
         * @param endPoint A vector of positions for the end of the trajectory.
         * @param maxVel A scalar for the maximum speed.
         * @param maxAcc A scalar for the maximum acceleration and deceleration.
         * @param startTime The time that the trajectory begins.
         */
        TrapezoidalBase(const Eigen::Vector<DataType, Eigen::Dynamic> &startPosition,
                        const Eigen::Vector<DataType, Eigen::Dynamic> &endPosition,
                        const DataType                                &maxVel,
                        const DataType                                &maxAccel,
                        const DataType                                &startTime)
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
            this->_dimensions = startPosition.size();

            this->_startPoint.position     = startPosition;
            this->_startPoint.velocity     = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);
            this->_startPoint.acceleration = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);

            this->_endPoint.position     = endPosition;
            this->_endPoint.velocity     = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);
            this->_endPoint.acceleration = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);

            this->_startTime = startTime;

            DataType normaliser = -1;                                                               // Needed to scale velocity to be dimensionless

            DataType vel = maxVel;                                                                  // Temporary placeholder

            for(int i = 0; i < this->_dimensions; i++)
            {
                DataType distance = abs(startPosition(i) - endPosition(i));                         // Magnitude of distance between points
              
                normaliser = (distance > normaliser) ? distance : normaliser;                       // Normaliser must be the largest distance
              
                DataType temp = sqrt(maxAccel*distance);                                            // Peak velocity as a function of max acceleration
              
                vel = (vel > temp) ? temp : vel;                                                    // Ensure max velocity does not exceed acceleration, distance constraints
            }

            this->_normalisedVel = vel/normaliser;                                                  // Need to normalise so we can scale between 0 and 1

            this->_normalisedAcc = maxAccel/normaliser;                                             // Scale the acceleration to match the velocity

            this->_rampTime = vel/maxAccel;                                                         // Length of time to accelerate to max speed

            this->_rampDistance = 0.5*this->_normalisedAcc*this->_rampTime*this->_rampTime;         // s = 0.5*a*t^2

            this->_coastTime = (1.0 - 2*this->_rampDistance)/this->_normalisedVel;                  // Time spent moving at max speed

            this->_coastDistance = this->_normalisedVel*this->_coastTime;                           // Distance travelled moving at max speed

            this->_endTime = this->_startTime + 2*this->_rampTime + this->_coastTime;               // Total time passed
        }
          
        /**
        * Query the state for the given time. Override from TrajectoryBase class.
        * @param pos A storage location for the position.
        * @param vel A storage location for the velocity.
        * @param acc A storage location for the acceleration.
        * @param time The time at which to compute the state.
        * @return Returns false if there are any issues.
        */
        State<DataType> query_state(const DataType &time)
        {
                 if(time <= this->_startTime) return this->_startPoint;
            else if(time >= this->_endTime)   return this->_endPoint;
            else
            {
                State<DataType> state;                                                                      // Value to be returned
                
                DataType elapsedTime = time - this->_startTime;                                             // As it says

                DataType s, sd, sdd;                                                                        // Interpolating scalars

                if(elapsedTime < this->_rampTime)
                {
                      s = this->_normalisedAcc*elapsedTime*elapsedTime/2.0;
                     sd = this->_normalisedAcc*elapsedTime;
                    sdd = this->_normalisedAcc;
                }
                else if(elapsedTime < this->_rampTime + this->_coastTime)
                {
                      s = this->_rampDistance + this->_normalisedVel*(elapsedTime - this->_rampTime);
                     sd = this->_normalisedVel;
                    sdd = 0.0;
                }
                else
                {
                    DataType t = elapsedTime - this->_coastTime - this->_rampTime;
                   
                      s =  this->_rampDistance  + this->_coastDistance + this->_normalisedVel*t - this->_normalisedAcc*t*t/2.0;
                     sd =  this->_normalisedVel - this->_normalisedAcc*t;
                    sdd = -this->_normalisedAcc;
                }

                // Interpolate the state

                state.position     = (1.0 - s)*this->_startPoint.position + s*this->_endPoint.position;
                state.velocity     =  sd*(this->_endPoint.position - this->_startPoint.position);
                state.acceleration = sdd*(this->_endPoint.position - this->_startPoint.position);
                
                return state;
            }
        }
          
          /**
           * Query the total execution time for the trajectory.
           */
          DataType duration() const { return this->_coastTime + 2*this->_rampTime; }
          
     private:
     
          DataType _coastDistance;                                                                  ///< Distance covered at maximum speed
          DataType _coastTime;                                                                      ///< Length of time to move at max speed
          DataType _normalisedVel;                                                                  ///< Maximum velocity, normalised so total distance  = 1
          DataType _normalisedAcc;                                                                  ///< Maximum acceleration, normalised so total distance = 1
          DataType _rampDistance;                                                                   ///< Distance traveled whilst accelerating
          DataType _rampTime;                                                                       ///< Length of time to accelerate
                    
};                                                                                                  // Semicolon needed after class declaration


/**
 * This class builds upon the TrapezoidalBase to include any number of waypoints.
 */
template <class DataType>
class TrapezoidalVelocity : public TrajectoryBase<DataType>
{
    public:
    
        /**
         * Constructor.
         * @param waypoints An array of positions to pass through.
         * @param maxVelocity Defines the height for the trapezoidal velocity profile.
         * @param maxAcceleration Defines the slope of the trapezoidal velocity.
         * @param startTime When the trajectory commences.
         */
        TrapezoidalVelocity(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &positions,
                            const DataType &maxVelocity,
                            const DataType &maxAcceleration,
                            const DataType &startTime);
                           
        /**
         * Constructor with only 2 points.
         * @param startPosition As it says.
         * @param endPostion As it says.
         * @param maxVelocity Defines the height of the trapezoidal velocity profile.
         * @param maxAcceleration Defines the slope of the trapezoidal velocity profile.
         * @param startTime When the trajectory commences.
         */
        TrapezoidalVelocity(const Eigen::Vector<DataType,Eigen::Dynamic> &startPosition,
                            const Eigen::Vector<DataType,Eigen::Dynamic> &endPosition,
                            const DataType &maxVelocity,
                            const DataType &maxAcceleration,
                            const DataType &startTime)
        :
        TrapezoidalVelocity(std::vector<Eigen::Vector<DataType,Eigen::Dynamic>>{startPosition, endPosition},
                            maxVelocity, maxAcceleration, startTime) {}
    
        /**
         * Query the trajectory state for the given time.
         * @param time The point at which to evaluate the state.
         * @return A State data structure containing the position, velocity, and acceleration.
         */
        inline
        State<DataType>
        query_state(const DataType &time);
        
    private:

        std::vector<TrapezoidalBase<DataType>> _trajectories;                                       ///< An array of individual trapezoidal trajectories connecting and 2 points.
        
};                                                                                                  // Semicolon required after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
TrapezoidalVelocity<DataType>::TrapezoidalVelocity(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
                                                   const DataType &maxVelocity,
                                                   const DataType &maxAcceleration,
                                                   const DataType &startTime)
{ 
    if(waypoints.size() < 2)
    {
          throw std::invalid_argument("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
                                      "A minimum of 2 waypoints is required to generate a trajectory.");
    }
    
    DataType start = startTime;
    
    for(int i = 0; i < waypoints.size()-1; i++)                                                     // There are n-1 trajectories for n waypoints
    {
        this->_trajectories.emplace_back(waypoints[i], waypoints[i+1],
                                         maxVelocity, maxAcceleration, start);
        
        start = this->_trajectories.back().end_time();                                              // Start of next trajectory is the end of this one

    }
    
    this->_startTime = startTime;
    this->_endTime = start;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
inline
State<DataType>
TrapezoidalVelocity<DataType>::query_state(const DataType &time)
{
    for(int i = 0; i < this->_trajectories.size()-1; i++)
    {
        if(time < this->_trajectories[i].end_time())
        {
            return this->_trajectories[i].query_state(time);                                        // Must be on the ith trajectory
        }
    }
    
    // else
    return this->_trajectories.back().query_state(time);                                            // Final trajectory

}

#endif
