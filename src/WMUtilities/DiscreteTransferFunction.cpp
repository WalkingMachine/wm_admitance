/**
 * \file    DiscreteTransferFunction.cpp
 * \author  Olivier Lavoie 
 */

#include "DiscreteTransferFunction.h"

using namespace wm_admitance::utilities;


/**
 * \brief Constructeur par défaut qui initialise les informations nécessaires
 *        au calcul d'une fonction de transfert
 * \param pNbTransferFunction Le nombre de functions de transfert
 * \param[in] pTransferFunctionCoefficient LEs coefficients de la fonction de transfert
 * \param pFilterOrder L'ordre du filtre
 */
DiscreteTransferFunction::DiscreteTransferFunction(int pNbTransferFunction,
                                                   const TransferFunctionCoefficient& pTransferFunctionCoefficient,
                                                   int pFilterOrder) :
    aFilterOrder(pFilterOrder),
    aNbTransferFunction(pNbTransferFunction),
    aTransferFunctionCoefficient(pTransferFunctionCoefficient)
{
    setZero();
}

/**
 * \brief Mets à jour les nouvelles données après rétro-action de l'erreur
 * \param pError L'erreur à considérer
 * \return Les nouvelles données sous un format «Eigen»
 */
Eigen::VectorXd DiscreteTransferFunction::update(const Eigen::VectorXd& pError)
{
    aErrorHistory.col(0) = pError;
    aFilterResult = filterXOrder();
    updateHistory();
    aOutputHistory.col(0) = aFilterResult;
    return aFilterResult;
}

/**
 * \brief Mets à jour les nouvelles données après rétro-action de l'erreur
 * \param pError L'erreur à considérer
 * \return Les nouvelles données sous un format vectorielle
 */
std::vector<double> DiscreteTransferFunction::updateVector(const Eigen::VectorXd& pError)
{
    Eigen::VectorXd lEigen = update(pError);
    return std::vector<double>(lEigen.data(), lEigen.data() + lEigen.rows() * lEigen.cols());
}

/**
 * \brief Conserve les anciennes données après filtrage de la fonction de transfert
 */
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

/**
 * \brief Effectue un filtrage de la fonction de transfert
 * \return Retourne les données après filtrage
 */
Eigen::VectorXd DiscreteTransferFunction::filterXOrder() const
{
    Eigen::VectorXd lNumeratorResult = (aTransferFunctionCoefficient.aNumeratorFactor * aErrorHistory).rowwise().sum();
    Eigen::VectorXd lDenominatorResult = (aTransferFunctionCoefficient.aDenominatorFactor * aOutputHistory).rowwise().sum();
    return lNumeratorResult - lDenominatorResult;
}

/**
 * \brief Initialise à zéro les structures internes
 */
void DiscreteTransferFunction::setZero() 
{
    aFilterResult = Eigen::VectorXd::Zero(aNbTransferFunction);
    aOutputHistory = Eigen::ArrayXXd::Zero(aNbTransferFunction, aFilterOrder);
    aErrorHistory = Eigen::ArrayXXd::Zero(aNbTransferFunction, aFilterOrder + 1);
}