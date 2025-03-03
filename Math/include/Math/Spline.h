/**
 * @file    Spline.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Represents a series of polynomials connecting several points.
 * 
 * @details This class will interpolate across multiple points using piecewise polynomials.
 *          It currently supports cubic splines, and ensures continuity down to the second derivative.
 *          Higher order polynomials are possible, but continuity of derivatives is not yet supported (please help!)
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef SPLINE_H_
#define SPLINE_H_

#include <Math/Polynomial.h>

#include <vector>                                                                                   // Using "" tells the compiler to look locally

namespace RobotLibrary { namespace Math {

class Spline
{
    public:
        
        /**
         * @brief Constructor.
         * @param values The polynomial value, and its first and second derivatives at the given points.
         * @param points The corresponding independent variable for the spline values.
         */
        Spline(const std::vector<FunctionPoint> &values,
               const std::vector<double>        &points,
               const unsigned int               &order = 3);
               
        /**
         * @brief Query the spline values & derivatives for the given point.
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

} }

#endif
