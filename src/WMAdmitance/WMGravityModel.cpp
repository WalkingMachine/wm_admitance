// \file WMGravityModel.cpp
// \brief Definition of gravity module.
// Created by kevin on 07/03/2019.

#include "WMGravityModel.h"

#include <ros/ros.h>
#include <stdexcept>
#include <tf/transform_listener.h>

using namespace wm_admitance;

namespace
{
    const std::string gBaseTFName = ""; // ajouter le nom du TF de la base
}

WMGravityModel::WMGravityModel(const std::vector<std::string>& pTFNames) :
    aTFNames(pTFNames)
{
    // Initialisation du URDF parser
    if (!aURDFModel.initParam("/robot_description"))
    {
         ROS_ERROR("Failed to parse URDF parameter server '/robot_description'");
         throw std::runtime_error("Failed to parse URDF parameter server '/robot_description'");
    }

    urdf::LinkConstSharedPtr lLink;
    lLink = aURDFModel.links_.at("right_clavicular_link");
    ROS_INFO("MASSE: %lf", lLink->inertial->mass);
}

CompensatedTorqueVector WMGravityModel::process()
{
    CompensatedTorqueVector lCompensatedTorque = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

    return lCompensatedTorque;
}

std::vector<tf::Quaternion>  WMGravityModel::retrievePositionFromTF()
{
    std::vector<tf::Quaternion> lQuaternionList;
    for (const std::string& lTFName : aTFNames)
    {
        try
        {
            tf::StampedTransform lTransform;
            aListener.lookupTransform("/base_link", lTFName,
                                 ros::Time(0), lTransform);

            lQuaternionList.emplace_back(std::move(lTransform.getRotation()));
        }
        catch (const tf::TransformException& ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    return lQuaternionList;
}