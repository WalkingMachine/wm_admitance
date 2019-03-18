// \file WMAdmitance.cpp
// \brief Definition of gravity module.
// Created by kevin on 07/03/2019.

#include "WMAdmitance.h"

#include <ros/ros.h>
#include <stdexcept>

using namespace wm_admitance;
using namespace wm_admitance::utilities;

namespace std
{
    template<typename T, typename... Args>
    unique_ptr<T> make_unique(Args&&... args)
    {
        return unique_ptr<T>(new T(forward<Args>(args)...));
    }
}

WMAdmitance::WMAdmitance(ros::NodeHandle& pRootNode)
{
    // Construire la fonction de transfer DiscreteTransferFunction

    pRootNode.getParam("sara_admitance/joint_names", aJointNames);

    std::string lURDFPath;
    pRootNode.getParam("sara_admitance/urdf_path", lURDFPath);

    aGravityModel = std::make_unique<WMGravityModel>(aJointNames, std::move(lURDFPath), 7);

    TransferFunctionCoefficient lFunctionCoeff;
    lFunctionCoeff.aNumeratorFactor = Eigen::ArrayXXd::Zero(aJointNames.size(), 2);
    lFunctionCoeff.aDenominatorFactor = Eigen::ArrayXXd::Zero(aJointNames.size(), 2 + 1);

    for (int i = 0; i < aJointNames.size(); i++)
    {
        lFunctionCoeff.aNumeratorFactor.row(i) << -1.970052211604768, 0.970446261452074;
        lFunctionCoeff.aDenominatorFactor.row(i)   << 0.004925623091321, 0.009851246182642, 0.004925623091321;
    }

    aDiscreteTF = std::make_unique<DiscreteTransferFunction>(
            aJointNames.size(),
            std::move(lFunctionCoeff),
            2);

    for (const auto& lJointName : aJointNames)
    {
        aAdmitanceVelocityMap.emplace(lJointName, 0.0f);
    }

    aJointStateSub = aAdmitanceNode.subscribe("joint_states", 1, &WMAdmitance::jointStateCallback, this);
}

double WMAdmitance::getAdmitanceVelocityFromJoint(const std::string& pJointName)
{
    double lVelocity = 0.0f;
    const auto& lIter = aAdmitanceVelocityMap.find(pJointName);
    if (lIter != aAdmitanceVelocityMap.end())
    {
        lVelocity = lIter->second;
    }
    return lVelocity;
}

void WMAdmitance::process()
{
    //ROS_INFO("%lf", aGravityModel->process()[0]);
    std::vector<double> lAdmitanceTorque =
        calculateAdmitanceTorque(aGravityModel->process());

    size_t lIndex = 0;
    for (const auto& lJointName : aJointNames)
    {
        ROS_INFO("%s: %lf", lJointName.c_str(), lAdmitanceTorque[lIndex]);
        ++lIndex;
    }

    //Eigen::VectorXd lEigenVect(5);
    //size_t lIndexEigen = 0;
    //for (const auto& lTorque : lAdmitanceTorque)
    //{
    //    lEigenVect(lIndexEigen) = lTorque;
    //    ++lIndexEigen;
    //}


    //aDiscreteTF->update(lEigenVect);

    //updateAdmitanceVelocity(
    //    aDiscreteTF->updateVector(Eigen::Map<Eigen::VectorXd>(lAdmitanceTorque.data(), lAdmitanceTorque.size())));

    //updateAdmitanceVelocity(lAdmitanceTorque);

    //for (const auto& lJointName : aJointNames)
    //{
    //    ROS_INFO("Effort of %s: %lf", lJointName.c_str(), getAdmitanceVelocityFromJoint(lJointName));
    //}
}

void WMAdmitance::jointStateCallback(const sensor_msgs::JointState& pMsg)
{
    if (!aFirstCheck)
    {
        aFirstCheck = true;
        for (const auto& lMapItem : aAdmitanceVelocityMap)
        {
            bool lJointNamePresent = false;
            for (const auto& lJointName : pMsg.name)
            {
                if (lMapItem.first == lJointName)
                {
                    lJointNamePresent = true;
                    break;
                }
            }

            if (!lJointNamePresent)
            {
                ROS_ERROR("WMAdmitance: Joint '%s' is not present in topic /joint_states.", lMapItem.first.c_str());
                throw std::runtime_error("WMAdmitance: Joint '" + lMapItem.first + "' is not present in topic /joint_states.");
            }
        }
    }
    else
    {
        aJointState = pMsg;
    }
}

void WMAdmitance::updateAdmitanceVelocity(const std::vector<double>& pAdmitanceVelocity)
{
    size_t lIndex = 0;
    for (const auto& lJointName : aJointNames)
    {
        aAdmitanceVelocityMap[lJointName] = pAdmitanceVelocity[lIndex];
        ++lIndex;
    }
}

std::vector<double> WMAdmitance::calculateAdmitanceTorque(const std::vector<double>& pCompensatedTorque)
{
    size_t lIndex = 0;
    std::vector<double> lAdmitanceTorque;
    for (const auto& lJointName : aJointNames)
    {
        ROS_INFO("Real effort: %lf", getEffortFromJoint(lJointName));
        ROS_INFO("Compensated effort: %lf", pCompensatedTorque[lIndex]);
        lAdmitanceTorque.emplace_back(getEffortFromJoint(lJointName) - pCompensatedTorque[lIndex]);
        ++lIndex;
    }
    return lAdmitanceTorque;
}

double WMAdmitance::getEffortFromJoint(const std::string& pJointName)
{
    size_t lIndex = 0;
    double lEffort = 0.0f;
    for (const auto& lJointName : aJointState.name)
    {
        if (lJointName == pJointName)
        {
            lEffort = aJointState.effort[lIndex];
            break;
        }
        ++lIndex;
    }
}