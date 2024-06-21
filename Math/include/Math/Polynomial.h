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

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      Constructor                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
Polynomial<DataType>::Polynomial(const FunctionPoint<DataType> &startValues,
                                 const FunctionPoint<DataType> &endValues,
                                 const DataType &startPoint,
                                 const DataType &endPoint,
                                 const unsigned int &order)
                                 : _order(order)
{
    if(order%2 == 0)
    {
        throw std::invalid_argument("[ERROR] [POLYNOMIAL] Constructor: "
                                    "Order of the polynomial was " + std::to_string(order) +
                                    " but it must be an odd number.");
    }
    
    this->_coefficients.resize(order+1);                                                            // A polynomial of degree n has n+1 coefficients
    
    unsigned int n = (order+1)/2;                                                                   // Makes indexing a little easier
    
    Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> X(order+1,order+1); X.setZero();
    
    for(int i = 0; i < n; i++)                                                                      // Only need to enumerate across half the number of rows
    {
        for(int j = i; j < X.cols(); j++)
        {
            DataType derivativeCoeff;

                 if(i == 0) derivativeCoeff = 1.0;
            else if(i == 1) derivativeCoeff = j;
            else
            {
                derivativeCoeff = 1.0;
                for(int k = 0; k < i; k++) derivativeCoeff *= (j-k);
            }
            
            X(i,j)   = derivativeCoeff*pow(startPoint,j-i);                                         // Row i pertains to first point
            X(i+n,j) = derivativeCoeff*pow(endPoint,j-i);                                           // Row i+n pertains to second point
        }
    }
    
    Eigen::Vector<DataType,Eigen::Dynamic> supportPoints(order+1); supportPoints.setZero();
    
    supportPoints(0) = startValues.value;
    supportPoints(n) = endValues.value;
    
    // Add endpoint derivative values
    if(order >= 3)
    {
       supportPoints(1)   = startValues.firstDerivative;
       supportPoints(n+1) = endValues.firstDerivative;
    }
    
    // Add second derivative values
    if(order >= 5)
    {
        supportPoints(2)   = startValues.secondDerivative;
        supportPoints(2+n) = startValues.secondDerivative;
    }
    
    // NOTE: Higher derivatives are assumed to be 0
    
    this->_coefficients = X.partialPivLu().solve(supportPoints);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Evaluate the polynomial at a given point.                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
inline
FunctionPoint<DataType>
Polynomial<DataType>::evaluate_point(const DataType &input)
{
    FunctionPoint<DataType> output = {0.0, 0.0, 0.0};                                               // Value to be returned

    for(int i = 0; i <= this->_order; i++)
    {
                  output.value            +=         this->_coefficients(i)*pow(input,i-0);
        if(i > 0) output.firstDerivative  +=       i*this->_coefficients(i)*pow(input,i-1);
        if(i > 1) output.secondDerivative += (i-1)*i*this->_coefficients(i)*pow(input,i-2);
    }
    
    return output;
}

#endif
