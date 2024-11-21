/**
 * @file   TrajectoryBase.cpp
 * @author Jon Woolfrey
 * @date   July 2024
 * @brief  Source files for the TrajectoryBase class.
 */
 
#include "TrajectoryBase.h"

namespace RobotLibrary {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
TrajectoryBase::TrajectoryBase(const State  &startPoint,
                               const State  &endPoint,
                               const double &startTime,
                               const double &endTime)
                               : _startPoint(startPoint),
                                 _endPoint(endPoint),
                                 _startTime(startTime),
                                 _endTime(endTime),
                                 _dimensions(startPoint.position.size())
{
    if(startTime == endTime)
    {
        throw std::logic_error("[ERROR] [TRAJECTORY BASE] Constructor: "
                               "Start time is equal to the end time for the trajectory "
                               "(" + std::to_string(startTime) + " = " + std::to_string(endTime) + "). "
                               "You cannot move faster than light.");
    }
    else if(startTime > endTime)
    {
        throw std::logic_error("[ERROR] [TRAJECTORY BASE] Constructor: "
                               "Start time is greater than end time for the trajectory "
                               "(" + std::to_string(startTime) + " > " + std::to_string(endTime) + "). "
                               "You cannot go back in time.");
    }
    else if(startPoint.position.size()     != startPoint.velocity.size()
         or startPoint.acceleration.size() != endPoint.position.size()
         or endPoint.position.size()       != endPoint.velocity.size()
         or endPoint.velocity.size()       != endPoint.acceleration.size())
    {
        throw std::invalid_argument("[ERROR] [TRAJECTORY BASE] Constructor: "
                                    "Dimensions of arguments do not match. "
                                    "The start position had " + std::to_string(startPoint.position.size()) + " elements, "
                                    "the start velocity had " + std::to_string(startPoint.velocity.size()) + " elements, "
                                    "the start acceleration had " + std::to_string(startPoint.acceleration.size()) + " elements, "
                                    "the end position had " + std::to_string(endPoint.position.size()) + " elements, "
                                    "the end velocity had " + std::to_string(endPoint.velocity.size()) + " elements, and "
                                    "the end acceleration had " + std::to_string(endPoint.acceleration.size()) + " elements.");
    }
}

}
