/**
 * @file    Polynomial.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Represents a polynomial function.
 * 
 * @details This class represents a polynomial function of the form  f(x) = a_0 + a_1 * x + a_2 * x^2 + ...
 *          By querying a point it can return the interpolated value and its derivatives.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef POLYNOMIAL_H_
#define POLYNOMIAL_H_

#include <Eigen/Dense>                                                                              // Eigen::Matrix, Eigen::Vector, and decompositions

namespace RobotLibrary { namespace Math {

/**
 * @brief A data structure for representing the output of a function f(x), and its derivatives df/dx, d^2f/dx^2.
 */
struct FunctionPoint
{
    double value            = 0.0;                                                                  ///< The value y = f(x)
    double firstDerivative  = 0.0;                                                                  ///< dy/dx
    double secondDerivative = 0.0;                                                                  ///< d^2y/dx^2
    
};                                                                                                  // Semicolon required after a class declaration

/**
 * @brief A class representing polynomial functions f(x) = c_0 + c_1*x + c_2*x^2 + ... + c_n*x^n.
 */
class Polynomial
{
    public:
        
        /**
         * @brief Empty constructor.
         */
        Polynomial() {}
        
        /**
         * @brief Constructor.
         * @param startPoint The value, first derivative, and second derivative a given point.
         * @param endPoint The value, first derivative, and second derivative for a given point.
         * @param order The number of terms in the polynomial.
         */
        Polynomial(const FunctionPoint &startValues,
                   const FunctionPoint &endValues,
                   const double        &startPoint,
                   const double        &endPoint,
                   const unsigned int  &order);
        
        /**
         * @brief Compute the value of the polynomial f(x) = c_0 + c_1*x1 + ... + c_n*x^n for a given x.
         * @param input The "x" (independent variable) for the polynomial.
         * @param An array containing the function value f(x) and its derivatives f'(x), f''(x).
         */
        FunctionPoint
        evaluate_point(const double &input);

    private:
    
        unsigned int _order;                                                                        ///< Degrees of freedom in the polynomial.
        
        Eigen::VectorXd _coefficients;                                                              ///< As it says.
        
};                                                                                                  // Semicolon needed after class declaration

} }

#endif
