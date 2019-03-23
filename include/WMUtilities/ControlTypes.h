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

namespace wm_admittance
{
    namespace utilities
    {
        /**
         * \brief Une structure contenant le numérateur et
         *        le dénominateur d'une fonction de transfert
         */
        struct TransferFunctionCoefficient
        {
            Eigen::ArrayXXd aNumeratorFactor;
            Eigen::ArrayXXd aDenominatorFactor;
        };

        /**
         * \brief Une structure contenant les informations
         *        utiles pour un robot
         */
        struct RobotData
        {
            std::vector<double> aLinkMass;
            std::vector<tf::Matrix3x3> aInertiaTensor;
            std::vector<tf::Vector3> aCenterOfMass;
        };

    } // namespace utilities
} // namespace wm_admittance

#endif // CONTROL_TYPE_H