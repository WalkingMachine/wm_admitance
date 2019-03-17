/**
 * \file    ControlTypes.h
 * \author  Olivier Lavoie
 */

#ifndef CONTROL_TYPE_H
#define CONTROL_TYPE_H

#include <eigen3/Eigen/Eigen>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <vector>

namespace wm_admitance
{
    namespace utilities
    {
        struct TransferFunctionCoefficient
        {
            Eigen::ArrayXXd aNumeratorFactor;
            Eigen::ArrayXXd aDenominatorFactor;
        };

        struct RobotData
        {
            std::vector<double> aLinkMass;
            std::vector<tf::Matrix3x3> aInertiaTensor;
            std::vector<tf::Vector3> aCenterOfMass;
        };

    } // namespace utilities
} // namespace wm_admitance

#endif // CONTROL_TYPE_H