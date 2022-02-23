    ////////////////////////////////////////////////////////////////////////////////////////////////////  
   //                                                                                                //
  //                           A minimum jerk trajectory between 2 points                           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef QUINTIC_H_
#define QUINTIC_H_

#include <Eigen/Core>                                                                              // Eigen::VectorXf

class Quintic
{
	public:
		Quintic() {}                                                                       // Empty constructor
	
		Quintic(const Eigen::VectorXf &startPoint,                                         // Constructor for trajectory across real numbers
			const Eigen::VectorXf &endPoint,
			const float &startTime,
			const float &endTime);
			
		bool get_state(Eigen::VectorXf &pos,                                               // Get the desired position for the given time
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);
	
	private:
		bool isNotValid = true;                                                            // Object won't do anything if this is true
		float a, b, c;                                                                     // Polynomial coefficients
		float t1, t2;                                                                      // Start time and end time
		Eigen::VectorXf p1, p2;                                                            // Start point and end point for real numbers
		
};                                                                                                 // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
Quintic::Quintic(const Eigen::VectorXf &startPoint,                                                // Constructor for trajectory across real numbers
                 const Eigen::VectorXf &endPoint,
                 const float &startTime,
                 const float &endTime):
                 p1(startPoint),
                 p2(startPoint),
                 t1(startTime),
                 t2(endTime)
{
	// Check the inputs are the same length
	if(startPoint.size() != endPoint.size())
	{
		std::cerr << "[ERROR][QUINTIC] Constructor : Vectors are not of equal length!" << std::endl;
		std::cerr << " startPoint: " << startPoint.size() << " endPoint: " << endPoint.size() << std::endl;
	}
	else
	{
		this->isNotValid = false;
		
		// Check start time < end time
		if(this->t1 > this->t2)
		{
			std::cerr << "[WARNING][QUINTIC] Constructor : Start time " << this->t1
				<< " is greater than end time " << this->t2 << ". Swapping values..." << std::endl;
			float temp = this->t1;
			this->t1 = this->t2;
			this->t2 = temp;
		}
		
		// Compute coefficients x(t) = a*t^5 + b*t^4 + c*t^3		
		float dt = this->t1 - this->t2;			// Time difference
		this->a =   6*pow(dt,-5);
		this->b = -15*pow(dt,-4);
		this->c =  10*pow(dt,-3);
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Get the desired state for the given time                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Quintic::get_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel, Eigen::VectorXf &acc, const float &time)
{
	// Check the input arguments are of equal length
	if(pos.size() != vel.size() || vel.size() != acc.size())
	{
		std::cerr << "[ERROR][QUINTIC] get_state() : Input arguments are not of equal length!" << std::endl;
		std::cerr << "pos: " << pos.size() << " vel: " << vel.size() << " acc: " << acc.size() << std::endl;
				
		pos = this->p1;                                                                    // Remain at the start
		vel.setZero(); acc.setZero();                                                      // Don't move
		
		return false;
	}
	else if(this->isNotValid)
	{
		std::cerr << "[ERROR][QUINTIC] get_state() : A problem occured during construction of this object." << std::endl;
		
		pos = this->p1;                                                                    // Remain at the start
		vel.setZero(); acc.setZero();                                                      // Don't move
		
		return false;
	}
	else
	{
		// Get the time
		double dt;	
		if(time < this->t1) 		dt = 0.0;                                          // Not yet started, remain at beginning
		else if(time < this->t2)	dt = time - this->t1;                              // Somewhere in the middle
		else				dt = this->t2 - this->t1;                          // Finished; remain at end
		
		// Compute coefficients for interpolation
		float s =      this->a*pow(dt,5) +    this->b*pow(dt,4) +   this->c*pow(dt,3);
		float sd =   5*this->a*pow(dt,4) +  4*this->b*pow(dt,3) + 3*this->c*pow(dt,2);
		float sdd = 20*this->a*pow(dt,3) + 12*this->b*pow(dt,2) + 6*this->c;	
		
		// Compute the state
		pos = (1-s)*this->p1 + s*this->p2;
		vel =  sd*(this->p2 - this->p1);
		acc = sdd*(this->p2 - this->p1);
		
		return true;
	}
}

#endif

