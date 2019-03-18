// \file WMAdmitance.h
// \brief Declaration of gravity module.
// Created by kevin on 07/03/2019.

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

        double getAdmitanceVelocityFromJoint(const std::string& pJointName);

        void process();

    private:
        WMAdmitance();
        ~WMAdmitance() noexcept = default;
        WMAdmitance(const WMAdmitance&)= delete;
        WMAdmitance& operator=(const WMAdmitance&)= delete;

        void jointStateCallback(const sensor_msgs::JointState& pMsg);

        void updateAdmitanceVelocity(const std::vector<double>& pAdmitanceVelocity);
        std::vector<double> calculateAdmitanceTorque(const std::vector<double>& pCompensatedTorque);

        double getEffortFromJoint(const std::string& pJointName);

        static std::atomic<WMAdmitance*> aInstance;
        static std::mutex aMutex;

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

