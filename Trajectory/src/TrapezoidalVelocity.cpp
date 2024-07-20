/**
 * @file   TrapezoidalVelocity.cpp
 * @author Jon Woolfrey
 * @date   July 2024
 * @brief  Source files for the TrapezoidalVelocity class(es).
 */
 
#include <TrapezoidalVelocity.h>
#include <vector>

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
    this->_dimensions = startPosition.size();

    this->_startPoint.position     = startPosition;
    this->_startPoint.velocity     = Eigen::VectorXd::Zero(this->_dimensions);
    this->_startPoint.acceleration = Eigen::VectorXd::Zero(this->_dimensions);

    this->_endPoint.position     = endPosition;
    this->_endPoint.velocity     = Eigen::VectorXd::Zero(this->_dimensions);
    this->_endPoint.acceleration = Eigen::VectorXd::Zero(this->_dimensions);

    this->_startTime = startTime;

    double normaliser = -1;                                                               // Needed to scale velocity to be dimensionless

    double vel = maxVel;                                                                  // Temporary placeholder

    for(int i = 0; i < this->_dimensions; i++)
    {
        double distance = abs(startPosition(i) - endPosition(i));                                   // Magnitude of distance between points
      
        normaliser = (distance > normaliser) ? distance : normaliser;                               // Normaliser must be the largest distance
      
        double temp = sqrt(maxAccel*distance);                                                      // Peak velocity as a function of max acceleration
      
        vel = (vel > temp) ? temp : vel;                                                            // Ensure max velocity does not exceed acceleration, distance constraints
    }

    this->_normalisedVel = vel/normaliser;                                                          // Need to normalise so we can scale between 0 and 1

    this->_normalisedAcc = maxAccel/normaliser;                                                     // Scale the acceleration to match the velocity

    this->_rampTime = vel/maxAccel;                                                                 // Length of time to accelerate to max speed

    this->_rampDistance = 0.5*this->_normalisedAcc*this->_rampTime*this->_rampTime;                 // s = 0.5*a*t^2

    this->_coastTime = (1.0 - 2*this->_rampDistance)/this->_normalisedVel;                          // Time spent moving at max speed

    this->_coastDistance = this->_normalisedVel*this->_coastTime;                                   // Distance travelled moving at max speed

    this->_endTime = this->_startTime + 2*this->_rampTime + this->_coastTime;                       // Total time passed
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
inline
State 
TrapezoidalBase::query_state(const double &time)
{
         if(time <= this->_startTime) return this->_startPoint;
    else if(time >= this->_endTime)   return this->_endPoint;
    else
    {
        State state;                                                                                // Value to be returned
        
        double elapsedTime = time - this->_startTime;                                               // As it says

        double s, sd, sdd;                                                                          // Interpolating scalars

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
            double t = elapsedTime - this->_coastTime - this->_rampTime;
           
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
inline
State
TrapezoidalVelocity::query_state(const double &time)
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
