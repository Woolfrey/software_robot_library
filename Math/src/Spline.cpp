/**
 * @file   Spline.cpp
 * @author Jon Woolfrey
 * @date   July 2024
 * @brief  Source files for the Spline class.
 */
 
#include <Spline.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Spline::Spline(const std::vector<FunctionPoint> &values,
               const std::vector<double>        &points,
               const unsigned int               &order)
               : _numberOfSplines(values.size()-1),
                 _points(points)
{
    // NOTE: There are n-1 splines for n supporting points
    
    if(_numberOfSplines < 1)
    {
        throw std::invalid_argument("[ERROR] [SPLINE] Constructor: "
                                    "A minimum of 2 points is required to generate a spline.");
    }
    else if(values.size() != points.size())
    {
        throw std::invalid_argument("[ERROR] [SPLINE] Constructor: "
                                    "Dimensions of arguments do not match. "
                                    "The were " + std::to_string(points.size()) + " points, "
                                    "but " + std::to_string(values.size()) + " values defined.");
    }
    
    // Create the polynomials joining the points
    for(int i = 0; i < this->_numberOfSplines; i++)
    {
        if(points[i] == points[i+1])
        {
            throw std::invalid_argument("[ERROR] [SPLINE] Constructor: "
                                        "Point " + std::to_string(i) + " is equal to point "
                                        + std::to_string(i+1) + " (" + std::to_string(points[i]) +
                                        " == " + std::to_string(points[i+1]) + ").");
        }
        
        this->_polynomial.emplace_back(values[i], values[i+1], points[i], points[i+1], order);
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Evaluate the spline for the given point.                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
FunctionPoint
Spline::evaluate_point(const double &input)
{
    // NOTE: A spline is not defined outside of its support points
    
    if(input <= this->_points.front())
    {
        return this->_polynomial.front().evaluate_point(this->_points.front());
    }
    else
    {
        for(int i = 0; i < this->_points.size()-1; i++)
        {
            if(input < this->_points[i+1])
            {
                return this->_polynomial[i].evaluate_point(input);                                  // Must be on the ith spline
            }
        }
    }
    // else
    return this->_polynomial.back().evaluate_point(this->_points.back());
}
