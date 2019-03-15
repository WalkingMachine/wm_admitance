/**
 * \file    DiscreteTransferFunction.cpp
 * \author  Olivier Lavoie 
 */

#include "DiscreteTransferFunction.h"

using namespace wm_admitance::utilities;

DiscreteTransferFunction::DiscreteTransferFunction(int pNbTransferFunction,
                                                   const TransferFunctionCoefficient& pTransferFunctionCoefficient,
                                                   int pFilterOrder) :
    aFilterOrder(pFilterOrder),
    aNbTransferFunction(pNbTransferFunction),
    aTransferFunctionCoefficient(pTransferFunctionCoefficient)
{
    setZero();
}

Eigen::VectorXd DiscreteTransferFunction::update(const Eigen::VectorXd& pError)
{
    aErrorHistory.col(0) = pError;
    aFilterResult = filterXOrder();
    updateHistory();
    aOutputHistory.col(0) = aFilterResult;
    return aFilterResult;
}

void DiscreteTransferFunction::updateHistory()
{
    Eigen::ArrayXXd lOutputHistory = Eigen::ArrayXXd::Zero(aNbTransferFunction, aFilterOrder);
    Eigen::ArrayXXd lErrorHistory  = Eigen::ArrayXXd::Zero(aNbTransferFunction, aFilterOrder + 1);
    lOutputHistory.matrix().block(0, 1, aNbTransferFunction, aFilterOrder - 1) =
        aOutputHistory.matrix().block(0, 0, aNbTransferFunction, aFilterOrder - 1);
    lErrorHistory.matrix().block(0, 1, aNbTransferFunction, aFilterOrder) =
        aErrorHistory.matrix().block(0, 0, aNbTransferFunction, aFilterOrder);
    aOutputHistory = lOutputHistory;
    aErrorHistory = lErrorHistory;
}

Eigen::VectorXd DiscreteTransferFunction::filterXOrder() const
{
    Eigen::VectorXd lNumeratorResult = (aTransferFunctionCoefficient.aNumeratorFactor * aErrorHistory).rowwise().sum();
    Eigen::VectorXd lDenominatorResult = (aTransferFunctionCoefficient.aDenominatorFactor * aOutputHistory).rowwise().sum();
    return lNumeratorResult - lDenominatorResult;
}

void DiscreteTransferFunction::setZero() 
{
    aFilterResult = Eigen::VectorXd::Zero(aNbTransferFunction);
    aOutputHistory = Eigen::ArrayXXd::Zero(aNbTransferFunction, aFilterOrder);
    aErrorHistory = Eigen::ArrayXXd::Zero(aNbTransferFunction, aFilterOrder + 1);
}