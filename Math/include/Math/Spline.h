/**
 * @file   Spline.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class defining splines.
 */

#ifndef SPLINE_H_
#define SPLINE_H_

#include <Math/Polynomial.h>

template <class DataType>
class Spline
{
    public:
        
        /**
         * Constructor.
         * @param values The polynomial value, and its first and second derivatives at the given points.
         * @param points The corresponding independent variable for the spline values.
         */
        Spline(const std::vector<FunctionPoint<DataType>> &values,
               const std::vector<DataType> &points,
               const unsigned int &order);
               
        /**
         * Query the spline values & derivatives for the given point.
         * @param input The independent variable for evaluating the spline.
         * @return A FunctionPoint data structure containing the value and derivatives.
         */
        inline
        FunctionPoint<DataType>
        evaluate_point(const DataType &input);
    
    private:
    
    unsigned int _numberOfSplines;
    
    std::vector<Polynomial<DataType>> _polynomial;
    
    std::vector<DataType> _points;
     
};                                                                                                  // Semicolon needed after a class declaration
#endif
