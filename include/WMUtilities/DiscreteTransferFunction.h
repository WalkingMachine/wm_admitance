/**
 * \file    DiscreteTransferFunction.h
 * \author  Olivier Lavoie
 */

#ifndef DISCRETE_TRANSFER_FUNCTION_H
#define DISCRETE_TRANSFER_FUNCTION_H

#include "ControlTypes.h"

#include <eigen3/Eigen/Eigen>

namespace wm_admitance
{
    namespace utilities
    {

        struct TransferFunctionCoefficient;

        class DiscreteTransferFunction final
        {

        public:
            DiscreteTransferFunction(int pNbTransferFunction, const TransferFunctionCoefficient& pTransferFunctionCoefficient,
                                     int pFilterOrder);
            ~DiscreteTransferFunction() noexcept = default;

            Eigen::VectorXd update(const Eigen::VectorXd& pError);
            
            void setZero();

        private:
            Eigen::VectorXd filterXOrder() const;
            void updateHistory();

            int aFilterOrder;
            int aNbTransferFunction;
            Eigen::VectorXd aFilterResult;
            Eigen::ArrayXXd aOutputHistory;
            Eigen::ArrayXXd aErrorHistory;

            const TransferFunctionCoefficient aTransferFunctionCoefficient;
        };
    } // namespace utilities
} // namespace wm_admitance

#endif // DISCRETE_TRANSFER_FUNCTION_H