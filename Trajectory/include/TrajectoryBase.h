    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                               A base class for trajectories                                   //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef TRAJECTORYBASE_H_
#define TRAJECTORYBASE_H_

#include <Eigen/Core>                                                                               // Eigen::VectorXf
#include <iostream>                                                                                 // std::cout

class TrajectoryBase
{
	public:
		TrajectoryBase() {}                                                                 // Empty constructor
		
		TrajectoryBase(const float &startTime,
		               const float &endTime,
		               const unsigned int &dim);
			
		// Get the desired state for the given time
		virtual bool get_state(Eigen::VectorXf &pos,
		                       Eigen::VectorXf &vel,
		                       Eigen::VectorXf &acc,
		                       const float     &time) = 0;
                                       
                // Other functions
                float start_time() const { return this->_startTime; }
                float end_time()   const { return this->_endTime;   }
		              
	protected:
		
		float _startTime, _endTime;
		
		unsigned int dimensions;
};                                                                                                  // Semicolon needed after class declaration

#endif
