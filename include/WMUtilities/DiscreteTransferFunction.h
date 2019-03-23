/**
 * \file    DiscreteTransferFunction.h
 * \author  Olivier Lavoie
 */

#ifndef DISCRETE_TRANSFER_FUNCTION_H
#define DISCRETE_TRANSFER_FUNCTION_H

#include "ControlTypes.h"
#include "ConfigManager.h"

#include <vector>
#include <eigen3/Eigen/Eigen>

#include <wm_admitance/FTParametersConfig.h>

namespace wm_admitance
{
    namespace utilities
    {
        /**
         * \brief Une classe qui effectue un calcul d'asservissement
         *        avec une fonction de transert.
         */
        class DiscreteTransferFunction : public ConfigManager<FTParametersConfig>
        {

        public:
            DiscreteTransferFunction(const std::string& pConfigFilePath);
            DiscreteTransferFunction(int pNbTransferFunction, const TransferFunctionCoefficient& pTransferFunctionCoefficient,
                                     int pFilterOrder);
            ~DiscreteTransferFunction() noexcept = default;

            Eigen::VectorXd update(const Eigen::VectorXd& pError);
            std::vector<double> updateVector(const Eigen::VectorXd& pError);
            
            void setZero();

        private:
            void onDynamicReconfigureChange(const FTParametersConfig& pConfig) override;
            void writeConfigFile(const FTParametersConfig& pConfig) override;
            void readConfigFile(FTParametersConfig& pConfig) override;

            Eigen::VectorXd filterXOrder() const;
            void updateHistory();

            std::string aConfigFilePath;

            int aFilterOrder;
            int aNbTransferFunction;
            Eigen::VectorXd aFilterResult;
            Eigen::ArrayXXd aOutputHistory;
            Eigen::ArrayXXd aErrorHistory;

            TransferFunctionCoefficient aTransferFunctionCoefficient;
        };
    } // namespace utilities
} // namespace wm_admitance

#endif // DISCRETE_TRANSFER_FUNCTION_H