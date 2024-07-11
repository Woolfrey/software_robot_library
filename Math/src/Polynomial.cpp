/**
 * @file   Polynomial.cpp
 * @author Jon Woolfrey
 * @date   July 2024
 * @brief  Source files for the Polynomial class.
 */

#include <Polynomial.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      Constructor                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Polynomial::Polynomial(const FunctionPoint &startValues,
                       const FunctionPoint &endValues,
                       const double        &startPoint,
                       const double        &endPoint,
                       const unsigned int  &order)
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
    
    Eigen::MatrixXd X(order+1,order+1); X.setZero();
    
    for(int i = 0; i < n; i++)                                                                      // Only need to enumerate across half the number of rows
    {
        for(int j = i; j < X.cols(); j++)
        {
            double derivativeCoeff;

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
    
    Eigen::VectorXd supportPoints(order+1); supportPoints.setZero();
    
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
FunctionPoint
Polynomial::evaluate_point(const double &input)
{
    FunctionPoint output = {0.0, 0.0, 0.0};                                                         // Value to be returned

    for(int i = 0; i <= this->_order; i++)
    {
                  output.value            +=         this->_coefficients(i)*pow(input,i-0);
        if(i > 0) output.firstDerivative  +=       i*this->_coefficients(i)*pow(input,i-1);
        if(i > 1) output.secondDerivative += (i-1)*i*this->_coefficients(i)*pow(input,i-2);
    }
    
    return output;
}
