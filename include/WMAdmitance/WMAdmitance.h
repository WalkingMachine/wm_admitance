// \file WMAdmitance.h
// \brief Declaration of gravity module.
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
#include <wm_admitance/sara_admitanceConfig.h>

#include "WMGravityModel.h"

#include "../WMUtilities/DiscreteTransferFunction.h"

namespace wm_admitance
{
    /**
     * \brief Type of map of joint name by admitance velocity type
     */
    using AdmitanceVelocityMapType = std::map<std::string, double>;

    class WMAdmitance final
    {
    public:
        static WMAdmitance* getInstance();

        double getAdmitanceVelocityFromJoint(const std::string& pJointName) const;

        bool isAdmitanceEnabled() const noexcept { return aEnableAdmitance; }

        void process();

    private:
        WMAdmitance();
        ~WMAdmitance() noexcept = default;
        WMAdmitance(const WMAdmitance&)= delete;
        WMAdmitance& operator=(const WMAdmitance&)= delete;

        void jointStateCallback(const sensor_msgs::JointState& pMsg);
        void dynamicReconfigureCallback(sara_admitance::sara_admitanceConfig &pConfig, uint32_t pLevel);

        void updateAdmitanceVelocity(const std::vector<double>& pAdmitanceVelocity);
        std::vector<double> calculateAdmitanceTorque(const std::vector<double>& pCompensatedTorque);

        double getEffortFromJoint(const std::string& pJointName) const;

        static std::atomic<WMAdmitance*> aInstance;
        static std::mutex aMutex;

        bool aEnableAdmitance{false};
        bool aVerboseMode{false};
        bool aFirstCheck{false};

        dynamic_reconfigure::Server<sara_admitance::sara_admitanceConfig> aDynamicConfigServer;

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

