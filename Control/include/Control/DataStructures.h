/**
 * @file    DataStructures.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    August 2025
 * @version 2.1.0
 *
 * @brief   Contains custom structs used in Control classes.
 *
 * @copyright (c) 2025 Jon Woolfrey
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 *            Contact: jonathan.woolfrey@gmail.com
 *
 * @see https://github.com/Woolfrey/software_robot_library
 * @see https://github.com/Woolfrey/software_simple_qp
 */

#ifndef CONTROL_DATA_STRUCTS_H
#define CONTROL_DATA_STRUCTS_H

#include <Math/QPSolver.h>

#include <Eigen/Core>                                                                               // Eigen::Matrix

namespace RobotLibrary { namespace Control {
/**
 * @brief A container for a control barrier function.
 * @note Standard form for optimsation is -\dot{b}^T * u \le \alpha(b)
 */
struct BarrierConstraints
{
    double scalar;                                                                                  ///< Right-hand-side of the CBF
    
    Eigen::Matrix<double,1,Eigen::Dynamic> rowVector;                                               ///< Left-hand-side of the CBF
};

} } // namespace

#endif
