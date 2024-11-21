/**
 * @file   Spline.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class defining splines.
 */

#ifndef SPLINE_H_
#define SPLINE_H_

#include "Polynomial.h" 
#include <vector>                                                                            // Using "" tells the compiler to look locally

namespace RobotLibrary {

class Spline
{
    public:
        
        /**
         * Constructor.
         * @param values The polynomial value, and its first and second derivatives at the given points.
         * @param points The corresponding independent variable for the spline values.
         */
        Spline(const std::vector<FunctionPoint> &values,
               const std::vector<double>        &points,
               const unsigned int               &order = 3);
               
        /**
         * Query the spline values & derivatives for the given point.
         * @param input The independent variable for evaluating the spline.
         * @return A FunctionPoint data structure containing the value and derivatives.
         */
        FunctionPoint
        evaluate_point(const double &input);
    
    private:
    
    unsigned int _numberOfSplines;                                                                  ///< As it says.
    
    std::vector<Polynomial> _polynomial;                                                            ///< The individual sections of the spline.
    
    std::vector<double> _points;                                                                    ///< The support points on the spline
     
};                                                                                                  // Semicolon needed after a class declaration

}

#endif
