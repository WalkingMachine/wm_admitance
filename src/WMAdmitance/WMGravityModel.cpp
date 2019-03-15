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

WMGravityModule::WMGravityModule(const std::vector<std::string>& pTFNames, const std::string& pURDFFilePath) :
    aTFNames(pTFNames),
    aURDFFilePath(pURDFFilePath)
{
    // Initialisation du URDF parser
    if (!aURDFModel.initFile(aURDFFilePath))
    {
         ROS_ERROR("Failed to parse urdf file '%s'", pURDFFilePath.c_str());
         throw std::runtime_error("Failed to parse urdf file " + pURDFFilePath);
    }

    // Peut-etre une liste de subscribe ici
    //aIMUSubHandle = aGravityNode.subscribe("imu", 1000, &WMGravityModule::imuCallback, this);
}

CompensatedTorqueVector WMGravityModule::process()
{

}

std::vector<tf::Quaternion>  WMGravityModule::retrievePositionFromTF()
{
   const auto& lList = retrievePositionFromTF()

    // On utilise aTFNames pour trouver les TFs des joints
    std::vector<std::string> lTFNameList;


    std::vector<tf::Quaternion> lQuaternionList;
    for (const std::string& lTFName : lTFNameList)
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

//void WMGravityModule::imuCallback(const sensor_msgs::Imu::ConstPtr& pIMUMessage)
//{
//    // http://wiki.ros.org/Robots/evarobot/Tutorials/indigo/IMU
//    // On set atomiquement des attributs de classes (float_ ou l'objet orientation.
//}