/**
 * @file   Polynomial.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class defining a polynomial function.
 */

#ifndef POLYNOMIAL_H_
#define POLYNOMIAL_H_

#include <Eigen/Dense>                                                                              // Eigen::Matrix, Eigen::Vector, and decompositions

/**
 * A data structure for representing the output of a function f(x), and its derivatives df/dx, d^2f/dx^2.
 */
template <class DataType>
struct FunctionPoint
{
    DataType value            = 0.0;                                                                ///< The value y = f(x)
    DataType firstDerivative  = 0.0;                                                                ///< dy/dx
    DataType secondDerivative = 0.0;                                                                ///< d^2y/dx^2
    
};                                                                                                  // Semicolon required after a class declaration


/**
 * A class representing polynomial functions f(x) = c_0 + c_1*x + c_2*x^2 + ... + c_n*x^n.
 */
template <class DataType>
class Polynomial
{
    public:
        
        /**
         * Empty constructor.
         */
        Polynomial() {}
        
        /**
         * Constructor.
         * @param startPoint The value, first derivative, and second derivative a given point.
         * @param endPoint The value, first derivative, and second derivative for a given point.
         * @param order The number of terms in the polynomial.
         */
        Polynomial(const FunctionPoint<DataType> &startValue,
                   const FunctionPoint<DataType> &endValue,
                   const DataType &startPoint,
                   const DataType &endPoint,
                   const unsigned int &order);
        
        /**
         * Compute the value of the polynomial f(x) = c_0 + c_1*x1 + ... + c_n*x^n for a given x.
         * @param input The "x" (independent variable) for the polynomial.
         * @param An array containing the function value f(x) and its derivatives f'(x), f''(x).
         */
        inline
        FunctionPoint<DataType>
        evaluate_point(const DataType &input);

    private:
    
        unsigned int _order;                                                                        ///< Degrees of freedom in the polynomial.
        
        Eigen::Vector<DataType,Eigen::Dynamic> _coefficients;                                       ///< As it says.
        
};                                                                                                  // Semicolon needed after class declaration

#endif
