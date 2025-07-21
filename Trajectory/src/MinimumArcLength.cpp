/**
 * @file    MinimumArcLength.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.0
 * @brief   Source code for the minimum arc length class.
 *
 * @details This is used for creating curves over a 2D plane with a minimum path length. Internally
 *          it is parameterised in polar coordinates. The angle parameter follows a cubic spline,
 *          whereas the radius parameter is a hyperbolic cosine. It is suitable for short paths for
 *          differential drive robots subject to non-holonomic constraints.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <cmath>                                                                 
#include <Trajectory/MinimumArcLength.h>

namespace RobotLibrary { namespace Trajectory {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
MinimumArcLength::MinimumArcLength(const RobotLibrary::Model::Pose2D &startPose,
 		                           const Eigen::Vector2d &endPoint,
 		                           const double &startTime,
 		                           const double &endTime)
: _initialPose(startPose)
{
	if (startTime == endTime)
	{
		throw std::invalid_argument("[ERROR] [TRAJECTORY GENERATOR] Constructor: "
					                "Start time was equal to end time (" + std::to_string(startTime) + " = "
					                + std::to_string(endTime) + "). You cannot move in zero time!");
	}
	else if (startTime > endTime)
	{
		throw std::invalid_argument("[ERROR] [TRAJECTORY GENERATOR] Constructor: "
		                            "Start time was greater than end time (" + std::to_string(startTime) + " > "
		                            + std::to_string(endTime) + "). You cannot go back in time!");
	}
	
	Eigen::Vector2d localEndPoint = _initialPose.inverse() * endPoint;                              // Transform to local coordinate frame

	double r = localEndPoint.norm();                                                                // i.e. the final radius of the curve
	
	double theta = atan2(localEndPoint[1], localEndPoint[0]);                                       // Get the angle for the polar coordinates

    RobotLibrary::Math::FunctionPoint startValues, endValues;
    startValues.value            = 0.0;                                                             // Always start from zero, in either case
    startValues.firstDerivative  = 0.0;                                                             // Velocity at start is zero
    startValues.secondDerivative = 0.0;                                                             // Acceleration at start is zero
    endValues.firstDerivative    = 0.0;                                                             // Velocity at end is zero
    endValues.secondDerivative   = 0.0;                                                             // Acceleration at end is zero
    
    double finalLocalHeading;                                                                       // We need to compute this conditionally
    
    // Generate the trajectory based on whether we follow curve or straight line
    if (abs(theta) < 1e-03)
    {
        _straightLine     = true;
        endValues.value   = r;                                                                      // Use the radius parameter
        finalLocalHeading = 0.0;
    }
    else                                                                                            // Form curved path
    {
        endValues.value = theta;                                                                    // Use the angle parameter
        
        _directionChange = (theta > 0) ? 1 : -1;
        
        // NOTE: r(\theta) = c_1 * e^{\theta) + c_2 * e^{-\theta},
        //       but since r_1 = 0.0 and \theta_1 = 0.0 we have c_2 = -c_1.
        _c1 = r / (2 * sinh(theta));                                                                // Need the absolute angle to prevent sign flips?
        _c2 = -_c1;
        
        double dr = _c1 * exp(theta) - _c2 * exp(-theta);
        double dx = dr * cos(theta) - r * sin(theta);
        double dy = dr * sin(theta) + r * cos(theta);

        finalLocalHeading = atan2(dy * _directionChange, dx * _directionChange);                    // Here theta is a substitute for omega so we get the correct sign
    }
    
     _polynomial = RobotLibrary::Math::Polynomial(startValues, endValues, startTime, endTime, 5);   // Create quintic polynomial
     
     // Resize values now for the state
     _state.position.resize(3);                                                                     // Save as x, y, \psi
     _state.velocity.resize(2);                                                                     // Save as v, \omega
     _state.acceleration.resize(2);                                                                 // Save as a, \alpha
     
     // We need to save values in the underlying base class
     _startTime = startTime;
     _endTime   = endTime;
     
     _startPoint.position     = Eigen::Vector3d(startPose.translation()[0], startPose.translation()[1], startPose.angle());
     _startPoint.velocity     = Eigen::Vector2d::Zero();
     _startPoint.acceleration = Eigen::Vector2d::Zero();
    
     _endPoint.position     = Eigen::Vector3d(endPoint[0], endPoint[1], startPose.angle() + finalLocalHeading);
     _endPoint.velocity     = Eigen::Vector2d::Zero();
     _endPoint.acceleration = Eigen::Vector2d::Zero();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Get the state for the given time                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Trajectory::State
MinimumArcLength::query_state(const double &time)
{
         if (time <= _startTime) return _startPoint;
    else if (time >  _endTime)   return _endPoint;
    else
    {
        // NOTE: The trajectory is specified in the initial coordinate frame of the robot;
        //       we need to transform it.
        double localHeading;
        Eigen::Vector2d localPosition;

        RobotLibrary::Math::FunctionPoint point = _polynomial.evaluate_point(time);
        
        if (_straightLine)
        {  
            localPosition[0] = point.value;                                                         // x-coordinate
            localPosition[1] = 0;                                                                   // y-coordinate
            
            localHeading = 0.0;
            
            _state.velocity[0] = point.firstDerivative;                                             // Linear velocity = dr/dt
            _state.velocity[1] = 0.0;                                                               // No turning
            
            _state.acceleration[0] = point.secondDerivative;                                        // Linear acceleration
            _state.acceleration[1] = 0.0;                                                           // No turning
        }
        else
        {
            double theta = point.value;
            double omega = point.firstDerivative;
            double alpha = point.secondDerivative;
            
            double r   = _c1 * exp(theta) + _c2 * exp(-theta);
            double dr  = _c1 * exp(theta) - _c2 * exp(-theta);
            double ddr = r;
            
            localPosition[0] = r * cos(theta);                                                      // x-coordinate
            localPosition[1] = r * sin(theta);                                                      // y-coordinate
            
            double dx = dr * cos(theta) - r * sin(theta);
            double dy = dr * sin(theta) + r * cos(theta);
                
            localHeading = atan2(dy * _directionChange, dx * _directionChange);                     // Angular velocity could be zero, so here we use the direction change as a substitute
         
            double ds = sqrt(r * r + dr * dr);                                                      // Partial derivative of arc length w.r.t. angle parameter
            
            _state.velocity[0] = omega * ds;                                                        // Linear velocity = ds/dt = d\theta/dt * ds/d\theta
            _state.velocity[1] = omega;                                                             // Angular velocity
            
            _state.acceleration[0] = alpha * ds + (2.0 * r * dr * omega * omega) / ds;              // Linear acceleration
            _state.acceleration[1] = alpha;                                                         // Angular acceleration
        }
        
        _state.position.head(2) = _initialPose * localPosition;                                     // Point transformation
        _state.position[2]      = _initialPose.angle() + localHeading;                              // Convert to global coordinate frame
 
        return _state;                         
    }
}

} } // namespace
