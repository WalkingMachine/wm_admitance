/**
 * \file    DiscreteTransferFunction.cpp
 * \author  Olivier Lavoie 
 */

#include "DiscreteTransferFunction.h"

#include <fstream>

using namespace wm_admitance::utilities;

/**
 * \brief Constructeur par défaut qui utilise la reconfiguration dynamique
 * \param[in] pConfigFilePath Le chemin vers le fichier de configuration YAML
 *
 * \note Ce constructeur utilise la reconfiguration dynamique pour appliquer la fonction
 *       de transfert
 */
DiscreteTransferFunction::DiscreteTransferFunction(const std::string& pConfigFilePath) :
    ConfigManager("FTParametersManager"),
    aConfigFilePath(pConfigFilePath)
{
    init(); // Les attributs de classe seront mis à jour après cette fonction
    setZero();
}

/**
 * \brief Constructeur avec paramètres qui initialise les informations nécessaires
 *        au calcul d'une fonction de transfert
 * \param pNbTransferFunction Le nombre de functions de transfert
 * \param[in] pTransferFunctionCoefficient LEs coefficients de la fonction de transfert
 * \param pFilterOrder L'ordre du filtre
 */
DiscreteTransferFunction::DiscreteTransferFunction(int pNbTransferFunction,
                                                   const TransferFunctionCoefficient& pTransferFunctionCoefficient,
                                                   int pFilterOrder) :
    ConfigManager("FTParametersManager"), // Pas utilisé, mais obligatoirement construit
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

/**
 * \brief Fonction de «callback» déclenché par le changement d'un coefficient
 * \param[in] pConfig L'object de configuration
 */
void DiscreteTransferFunction::onDynamicReconfigureChange(const FTParametersConfig& pConfig)
{
    aTransferFunctionCoefficient.aDenominatorFactor.row(0)
        << pConfig.FunctionTransfer1_denominator2, pConfig.FunctionTransfer1_denominator3;
    aTransferFunctionCoefficient.aNumeratorFactor.row(0)
        << pConfig.FunctionTransfer1_numerator1, pConfig.FunctionTransfer1_numerator2, pConfig.FunctionTransfer1_numerator3;

    aTransferFunctionCoefficient.aDenominatorFactor.row(1)
        << pConfig.FunctionTransfer2_denominator2, pConfig.FunctionTransfer2_denominator3;
    aTransferFunctionCoefficient.aNumeratorFactor.row(1)
        << pConfig.FunctionTransfer2_numerator1, pConfig.FunctionTransfer2_numerator2, pConfig.FunctionTransfer2_numerator3;

    aTransferFunctionCoefficient.aDenominatorFactor.row(2)
        << pConfig.FunctionTransfer3_denominator2, pConfig.FunctionTransfer3_denominator3;
    aTransferFunctionCoefficient.aNumeratorFactor.row(2)
        << pConfig.FunctionTransfer3_numerator1, pConfig.FunctionTransfer3_numerator2, pConfig.FunctionTransfer3_numerator3;

    aTransferFunctionCoefficient.aDenominatorFactor.row(3)
        << pConfig.FunctionTransfer4_denominator2, pConfig.FunctionTransfer4_denominator3;
    aTransferFunctionCoefficient.aNumeratorFactor.row(3)
        << pConfig.FunctionTransfer4_numerator1, pConfig.FunctionTransfer4_numerator2, pConfig.FunctionTransfer4_numerator3;

    aTransferFunctionCoefficient.aDenominatorFactor.row(4)
        << pConfig.FunctionTransfer5_denominator2, pConfig.FunctionTransfer5_denominator3;
    aTransferFunctionCoefficient.aNumeratorFactor.row(4)
        << pConfig.FunctionTransfer5_numerator1, pConfig.FunctionTransfer5_numerator2, pConfig.FunctionTransfer5_numerator3;
}

/**
 * \brief Fonction qui sauvegarde le fichier de configuration
 * \param[in] pConfig L'object de configuration
 */
void DiscreteTransferFunction::writeConfigFile(const FTParametersConfig& pConfig)
{
    YAML::Emitter lOut;
    lOut.SetIndent(4);

    lOut << YAML::BeginMap;
    lOut << YAML::Comment("Ces paramètres ne peuvent pas être modifiés dynamiquement");
    lOut << YAML::Key << "filter_order";
    lOut << YAML::Value << aFilterOrder;
    lOut << YAML::Key << "nb_functionTransfer";
    lOut << YAML::Value << aNbTransferFunction;
    lOut << YAML::EndMap << YAML::Newline;

    lOut << YAML::BeginMap;
    lOut << YAML::Comment("Ces paramètres peuvent être modifiés dynamiquement");
    lOut << YAML::Key << "FunctionTransfer1";  
    lOut << YAML::Value << YAML::BeginMap;
    lOut << YAML::Key << "numerator_1" << YAML::Value << pConfig.FunctionTransfer1_numerator1;;
    lOut << YAML::Key << "numerator_2" << YAML::Value << pConfig.FunctionTransfer1_numerator2;
    lOut << YAML::Key << "numerator_3" << YAML::Value << pConfig.FunctionTransfer1_numerator3;
    lOut << YAML::Key << "denominateur_2" << YAML::Value << pConfig.FunctionTransfer1_denominator2;
    lOut << YAML::Key << "denominateur_3" << YAML::Value << pConfig.FunctionTransfer1_denominator3;
    lOut << YAML::EndMap << YAML::Newline;

    lOut << YAML::Key << "FunctionTransfer2";  
    lOut << YAML::Value << YAML::BeginMap;
    lOut << YAML::Key << "numerator_1" << YAML::Value << pConfig.FunctionTransfer2_numerator1;;
    lOut << YAML::Key << "numerator_2" << YAML::Value << pConfig.FunctionTransfer2_numerator2;
    lOut << YAML::Key << "numerator_3" << YAML::Value << pConfig.FunctionTransfer2_numerator3;
    lOut << YAML::Key << "denominateur_2" << YAML::Value << pConfig.FunctionTransfer2_denominator2;
    lOut << YAML::Key << "denominateur_3" << YAML::Value << pConfig.FunctionTransfer2_denominator3;
    lOut << YAML::EndMap << YAML::Newline;

    lOut << YAML::Key << "FunctionTransfer3";  
    lOut << YAML::Value << YAML::BeginMap;
    lOut << YAML::Key << "numerator_1" << YAML::Value << pConfig.FunctionTransfer3_numerator1;;
    lOut << YAML::Key << "numerator_2" << YAML::Value << pConfig.FunctionTransfer3_numerator2;
    lOut << YAML::Key << "numerator_3" << YAML::Value << pConfig.FunctionTransfer3_numerator3;
    lOut << YAML::Key << "denominateur_2" << YAML::Value << pConfig.FunctionTransfer3_denominator2;
    lOut << YAML::Key << "denominateur_3" << YAML::Value << pConfig.FunctionTransfer3_denominator3;
    lOut << YAML::EndMap << YAML::Newline;

    lOut << YAML::Key << "FunctionTransfer4";  
    lOut << YAML::Value << YAML::BeginMap;
    lOut << YAML::Key << "numerator_1" << YAML::Value << pConfig.FunctionTransfer4_numerator1;;
    lOut << YAML::Key << "numerator_2" << YAML::Value << pConfig.FunctionTransfer4_numerator2;
    lOut << YAML::Key << "numerator_3" << YAML::Value << pConfig.FunctionTransfer4_numerator3;
    lOut << YAML::Key << "denominateur_2" << YAML::Value << pConfig.FunctionTransfer4_denominator2;
    lOut << YAML::Key << "denominateur_3" << YAML::Value << pConfig.FunctionTransfer4_denominator3;
    lOut << YAML::EndMap << YAML::Newline;

    lOut << YAML::Key << "FunctionTransfer5";  
    lOut << YAML::Value << YAML::BeginMap;
    lOut << YAML::Key << "numerator_1" << YAML::Value << pConfig.FunctionTransfer5_numerator1;;
    lOut << YAML::Key << "numerator_2" << YAML::Value << pConfig.FunctionTransfer5_numerator2;
    lOut << YAML::Key << "numerator_3" << YAML::Value << pConfig.FunctionTransfer5_numerator3;
    lOut << YAML::Key << "denominateur_2" << YAML::Value << pConfig.FunctionTransfer5_denominator2;
    lOut << YAML::Key << "denominateur_3" << YAML::Value << pConfig.FunctionTransfer5_denominator3;
    lOut << YAML::EndMap;
    lOut << YAML::EndMap;

    std::ofstream lOfs(aConfigFilePath);
    lOfs << lOut.c_str();
}

/**
 * \brief Fonction qui fait la lecture du fichier de configuration
 * \param[in] pConfig L'object de configuration
 *
 * \note Exception will be thrown if node doesn't exists.
 */
void DiscreteTransferFunction::readConfigFile(FTParametersConfig& pConfig)
{
    YAML::Node node = YAML::LoadFile(aConfigFilePath);

    aFilterOrder = node["filter_order"].as<size_t>();
    aNbTransferFunction = node["nb_functionTransfer"].as<size_t>();

    aTransferFunctionCoefficient.aNumeratorFactor = Eigen::ArrayXXd::Zero(aNbTransferFunction, aFilterOrder + 1);
    aTransferFunctionCoefficient.aDenominatorFactor = Eigen::ArrayXXd::Zero(aNbTransferFunction, aFilterOrder);

    pConfig.FunctionTransfer1_numerator1 = node["FunctionTransfer1"]["numerator_1"].as<double>();
    pConfig.FunctionTransfer1_numerator2 = node["FunctionTransfer1"]["numerator_2"].as<double>();
    pConfig.FunctionTransfer1_numerator3 = node["FunctionTransfer1"]["numerator_3"].as<double>();
    pConfig.FunctionTransfer1_denominator2 = node["FunctionTransfer1"]["denominateur_2"].as<double>();
    pConfig.FunctionTransfer1_denominator3 = node["FunctionTransfer1"]["denominateur_3"].as<double>();

    pConfig.FunctionTransfer2_numerator1 = node["FunctionTransfer2"]["numerator_1"].as<double>();
    pConfig.FunctionTransfer2_numerator2 = node["FunctionTransfer2"]["numerator_2"].as<double>();
    pConfig.FunctionTransfer2_numerator3 = node["FunctionTransfer2"]["numerator_3"].as<double>();
    pConfig.FunctionTransfer2_denominator2 = node["FunctionTransfer2"]["denominateur_2"].as<double>();
    pConfig.FunctionTransfer2_denominator3 = node["FunctionTransfer2"]["denominateur_3"].as<double>();

    pConfig.FunctionTransfer3_numerator1 = node["FunctionTransfer3"]["numerator_1"].as<double>();
    pConfig.FunctionTransfer3_numerator2 = node["FunctionTransfer3"]["numerator_2"].as<double>();
    pConfig.FunctionTransfer3_numerator3 = node["FunctionTransfer3"]["numerator_3"].as<double>();
    pConfig.FunctionTransfer3_denominator2 = node["FunctionTransfer3"]["denominateur_2"].as<double>();
    pConfig.FunctionTransfer3_denominator3 = node["FunctionTransfer3"]["denominateur_3"].as<double>();

    pConfig.FunctionTransfer4_numerator1 = node["FunctionTransfer4"]["numerator_1"].as<double>();
    pConfig.FunctionTransfer4_numerator2 = node["FunctionTransfer4"]["numerator_2"].as<double>();
    pConfig.FunctionTransfer4_numerator3 = node["FunctionTransfer4"]["numerator_3"].as<double>();
    pConfig.FunctionTransfer4_denominator2 = node["FunctionTransfer4"]["denominateur_2"].as<double>();
    pConfig.FunctionTransfer4_denominator3 = node["FunctionTransfer4"]["denominateur_3"].as<double>();

    pConfig.FunctionTransfer5_numerator1 = node["FunctionTransfer5"]["numerator_1"].as<double>();
    pConfig.FunctionTransfer5_numerator2 = node["FunctionTransfer5"]["numerator_2"].as<double>();
    pConfig.FunctionTransfer5_numerator3 = node["FunctionTransfer5"]["numerator_3"].as<double>();
    pConfig.FunctionTransfer5_denominator2 = node["FunctionTransfer5"]["denominateur_2"].as<double>();
    pConfig.FunctionTransfer5_denominator3 = node["FunctionTransfer5"]["denominateur_3"].as<double>();
    
}