    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                            A base class for polynomial trajectories                            //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POLYNOMIAL_H_
#define POLYNOMIAL_H_

#include <TrajectoryBase.h>
#include <string>
#include <vector>                                                                                  // std::vector

class Polynomial : public TrajectoryBase
{
	public:
		Polynomial() {}                                                                     // Empty constructor

		// Constructor for translations; delegate zero velocity
		Polynomial(const Eigen::VectorXf &startPoint,
			   const Eigen::VectorXf &endPoint,
			   const float           &startTime,
			   const float           &endTime,
			   const unsigned int    &order)
		:
		Polynomial(startPoint,
                           endPoint,
                           Eigen::VectorXf::Zero(startPoint.size()),
                           Eigen::VectorXf::Zero(endPoint.size()),
                           startTime,
                           endTime,
                           order) {}
		
		// Constructor with positions and endpoint velocities
		Polynomial(const Eigen::VectorXf &startPoint,
			   const Eigen::VectorXf &endPoint,
			   const Eigen::VectorXf &startVelocity,
			   const Eigen::VectorXf &endVelocity,
			   const float           &startTime,
			   const float           &endTime,
			   const unsigned int    &order);
		
		// Get the desired state
		bool get_state(Eigen::VectorXf &pos,
			       Eigen::VectorXf &vel,
			       Eigen::VectorXf &acc,
			       const float     &time);
		
	private:
		unsigned int _order;
		
		std::vector<std::vector<float>> coeff;                                              // Polynomial coefficients
		
		bool compute_coefficients(const Eigen::VectorXf &startPoint,
					  const Eigen::VectorXf &endPoint,
					  const Eigen::VectorXf &startVelocity,
					  const Eigen::VectorXf &endVelocity);
		
};                                                                                                 // Semicolon needed after class declaration

#endif
