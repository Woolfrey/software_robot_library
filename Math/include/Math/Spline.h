/**
 * @file    Spline.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   Represents a series of RobotLibrary::Math::Polynomials connecting several points.
 * 
 * @details This class will interpolate across multiple points using piecewise RobotLibrary::Math::Polynomials.
 *          It currently supports cubic splines, and ensures continuity down to the second derivative.
 *          Higher order RobotLibrary::Math::Polynomials are possible, but continuity of derivatives is not yet supported (please help!)
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef SPLINE_H
#define SPLINE_H

#include <Math/Polynomial.h>

#include <vector>                                                                                   // std::vector

namespace RobotLibrary { namespace Math {

/**
 * @brief A class that interpolates over multiple points using piece-wise polynomials.
 */
class Spline
{
    public:
        
        /**
         * @brief Constructor.
         * @param values The RobotLibrary::Math::Polynomial value, and its first and second derivatives at the given points.
         * @param points The corresponding independent variable for the spline values.
         */
        Spline(const std::vector<RobotLibrary::Math::FunctionPoint> &values,
               const std::vector<double>        &points,
               const unsigned int               &order = 3);
               
        /**
         * @brief Query the spline values & derivatives for the given point.
         * @param input The independent variable for evaluating the spline.
         * @return A RobotLibrary::Math::FunctionPoint data structure containing the value and derivatives.
         */
        RobotLibrary::Math::FunctionPoint
        evaluate_point(const double &input);
    
    private:
    
    unsigned int _numberOfSplines;                                                                  ///< As it says.
    
    std::vector<RobotLibrary::Math::Polynomial> _polynomial;                                        ///< The individual sections of the spline.
    
    std::vector<double> _points;                                                                    ///< The support points on the spline
     
};                                                                                                  // Semicolon needed after a class declaration

} }

#endif
