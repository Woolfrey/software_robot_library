#include <TrajectoryBase.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
TrajectoryBase::TrajectoryBase(const float        &startTime,
                               const float        &endTime,
                               const unsigned int &dim)
                               :
                               _startTime(startTime),
                               _endTime(endTime),
                               dimensions(dim)
{
	if(startTime == endTime)
	{
		auto start = std::to_string(startTime);
		auto end   = std::to_string(endTime);
		
		std::string message = "[ERROR] [TRAJECTORY BASE] Constructor: "
		                      "Start time of " + start + " is equal to "
		                      "end time of " + end + ". Cannot ove in zero time!";
		                     
		throw std::runtime_error(message);
	}
	else if(startTime > endTime)
	{
		std::cout << "[WARNING] [TRAJECTORY BASE] Constructor: "
		          << "Start time of " << startTime << " is greater than "
		          << "end time of " << endTime << ". "
		          << "Swapping values to avoid problems..." << std::endl;
      		
      		float temp       = this->_endTime;
      		this->_endTime   = this->_startTime;
      		this->_startTime = temp;
      	}
}
