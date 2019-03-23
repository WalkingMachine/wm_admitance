// \file WMAdmitance.h
// \brief Déclaration de la classe WMAdmitance
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
#include <wm_admitance/AdmitanceConfig.h>

#include "WMGravityModel.h"

#include "../WMUtilities/DiscreteTransferFunction.h"
#include "../WMUtilities/ConfigManager.h"

namespace wm_admitance
{
    /**
     * \brief Type d'une conteneur de vitesse d'admittance avec le nom du joint
     */
    using AdmitanceVelocityMapType = std::map<std::string, double>;

    /**
     * \brief Une classe «singleton» qui effectue l'agorithme d'admittance
     *
     * \details Cette classe calcules les vitesses requises sur chaque joint donné
     *          via le fichier de configuration (sara_admitance.yaml). Les autres joints
     *          sont tout de même pris en considération dans le calcul.
     *
     * \note En ce moment, cette classe supporte seulement l'admitance sur un seul bras.
     *       Cette fonctionnalité devra être ajoutée si nécessaire.
     */
    class WMAdmitance : public utilities::ConfigManager<AdmitanceConfig>
    {
    public:
        static WMAdmitance* getInstance();

        double getAdmitanceVelocityFromJoint(const std::string& pJointName) const;

        /**
         * \brief Vérifie si le mode admittance est activé
         * \return Retourne «true» si le mode admittance est activé, sinon «false»
         */
        bool isAdmitanceEnabled() const noexcept { return aEnableAdmitance; }

        void process();

    private:
        WMAdmitance();
        ~WMAdmitance() noexcept = default;
        WMAdmitance(const WMAdmitance&)= delete;
        WMAdmitance& operator=(const WMAdmitance&)= delete;

        void onDynamicReconfigureChange(const AdmitanceConfig& pConfig) override;
        void writeConfigFile(const AdmitanceConfig& pConfig) override;
        void readConfigFile(AdmitanceConfig& pConfig) override;

        void jointStateCallback(const sensor_msgs::JointState& pMsg);

        void updateAdmitanceVelocity(const std::vector<double>& pAdmitanceVelocity);
        std::vector<double> calculateAdmitanceTorque(const std::vector<double>& pCompensatedTorque);

        double getEffortFromJoint(const std::string& pJointName) const;

        static std::atomic<WMAdmitance*> aInstance;
        static std::mutex aMutex;

        bool aEnableAdmitance{false};
        bool aVerboseMode{false};
        bool aFirstCheck{false};

        std::unique_ptr<utilities::DiscreteTransferFunction> aDiscreteTF;

        sensor_msgs::JointState aJointState;

        ros::NodeHandle aAdmitanceNode;
        std::unique_ptr<WMGravityModel> aGravityModel;

        ros::Subscriber aJointStateSub;

        std::vector<std::string> aJointNames;

        AdmitanceVelocityMapType aAdmitanceVelocityMap;
    };
} // namespace wm_admitance
#endif // WM_ADMITANCE_H

