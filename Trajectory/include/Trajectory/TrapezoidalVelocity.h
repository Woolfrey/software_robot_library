/**
 * @file   TrapezoidalVelocity.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A trajectory with constant velocity.
 */

#ifndef TRAPEZOIDAL_VELOCITY_H_
#define TRAPEZOIDAL_VELOCITY_H_

#include <Eigen/Geometry>                                                                           // Eigen::Vector, Eigen::Matrix, Eigen::Quaternion
#include <TrajectoryBase.h>

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
                        const DataType &maxVel,
                        const DataType &maxAccel,
                        const DataType &startTime);
          
        /**
        * Query the state for the given time. Override from TrajectoryBase class.
        * @param pos A storage location for the position.
        * @param vel A storage location for the velocity.
        * @param acc A storage location for the acceleration.
        * @param time The time at which to compute the state.
        * @return Returns false if there are any issues.
        */
        State<DataType> query_state(const DataType &time);
          
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

#endif
