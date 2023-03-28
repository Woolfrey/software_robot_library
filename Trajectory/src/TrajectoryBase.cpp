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
	std::string errorMessage = "[ERROR] [TRAJECTORY BASE] Constructor: ";
	
	if(startTime == endTime)
	{
		errorMessage += "Start time of " + std::to_string(startTime) + " is equal to "
		                "end time of " + std::to_string(endTime) + ". "
		                "You cannot move faster than light.";
		                
		throw std::invalid_argument(errorMessage);
	}
	else if(startTime > endTime)
	{
		errorMessage += "Start time of " + std::to_string(startTime) + " is greater than "
		                "end time of " + std::to_string(endTime) + ". "
		                "You cannot go back to the past.";
	
		throw std::logic_error(errorMessage);
      	}
}
