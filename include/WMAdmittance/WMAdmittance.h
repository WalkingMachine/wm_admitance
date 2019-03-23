// \file WMAdmittance.h
// \brief Déclaration de la classe WMAdmittance
// \author Kevin Blackburn
// \author Olivier Lavoie

#ifndef WM_ADMITANCE_H
#define WM_ADMITANCE_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <atomic>
#include <future>
#include <mutex>
#include <thread>

#include <dynamic_reconfigure/server.h>
#include <wm_admittance/AdmittanceConfig.h>

#include "WMGravityModel.h"

#include "../WMUtilities/DiscreteTransferFunction.h"
#include "../WMUtilities/ConfigManager.h"

namespace wm_admittance
{
    /**
     * \brief Type d'une conteneur de vitesse d'admittance avec le nom du joint
     */
    using AdmittanceVelocityMapType = std::map<std::string, double>;

    /**
     * \brief Une classe «singleton» qui effectue l'agorithme d'admittance
     *
     * \details Cette classe calcules les vitesses requises sur chaque joint donné
     *          via le fichier de configuration (sara_admittance.yaml). Les autres joints
     *          sont tout de même pris en considération dans le calcul.
     *
     * \note En ce moment, cette classe supporte seulement l'admittance sur un seul bras.
     *       Cette fonctionnalité devra être ajoutée si nécessaire.
     */
    class WMAdmittance : public utilities::ConfigManager<AdmittanceConfig>
    {
    public:
        static WMAdmittance* getInstance();

        double getAdmittanceVelocityFromJoint(const std::string& pJointName) const;

        /**
         * \brief Vérifie si le mode admittance est activé
         * \return Retourne «true» si le mode admittance est activé, sinon «false»
         */
        bool isAdmittanceEnabled() const noexcept { return aEnableAdmittance; }

        void process();

    private:
        WMAdmittance();
        ~WMAdmittance() noexcept = default;
        WMAdmittance(const WMAdmittance&)= delete;
        WMAdmittance& operator=(const WMAdmittance&)= delete;

        void onDynamicReconfigureChange(const AdmittanceConfig& pConfig) override;
        void writeConfigFile(const AdmittanceConfig& pConfig) override;
        void readConfigFile(AdmittanceConfig& pConfig) override;

        void jointStateCallback(const sensor_msgs::JointState& pMsg);

        void updateAdmittanceVelocity(const std::vector<double>& pAdmittanceVelocity);
        std::vector<double> calculateAdmittanceTorque(const std::vector<double>& pCompensatedTorque);

        double getEffortFromJoint(const std::string& pJointName) const;

        static std::atomic<WMAdmittance*> aInstance;
        static std::mutex aMutex;

        bool aEnableAdmittance{false};
        bool aVerboseMode{false};
        bool aFirstCheck{false};

        std::unique_ptr<utilities::DiscreteTransferFunction> aDiscreteTF;

        sensor_msgs::JointState aJointState;

        ros::NodeHandle aAdmittanceNode;
        std::unique_ptr<WMGravityModel> aGravityModel;

        ros::Subscriber aJointStateSub;

        std::vector<std::string> aJointNames;

        AdmittanceVelocityMapType aAdmittanceVelocityMap;
    };
} // namespace wm_admittance
#endif // WM_ADMITANCE_H

