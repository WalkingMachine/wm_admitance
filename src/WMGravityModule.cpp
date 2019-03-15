// \file WMGravityModule.cpp
// \brief Definition of gravity module.
// Created by kevin on 07/03/2019.

#include "WMGravityModule.h"
#include <ros/ros.h>
#include <stdexcept>
#include <tf/transform_listener.h>
#include <WMGravityModule.h>


using namespace wm_admitance;

namespace
{
    const std::string gBaseTFName = ""; // ajouter le nom du TF de la base
}

WMGravityModule::WMGravityModule(const std::vector<std::string>& pTFNames, const std::string& pURDFFilePath, size_t pActuatorCount) :
    aTFNames(pTFNames),
    aActuatorCount{pActuatorCount},
    aURDFFilePath(pURDFFilePath)
{
    // Initialisation du URDF parser
    if (!aURDFModel.initFile(aURDFFilePath))
    {
         ROS_ERROR("Failed to parse urdf file '%s'", pURDFFilePath.c_str());
         throw std::runtime_error("Failed to parse urdf file " + pURDFFilePath);
    }

    aLinkMass.resize(pActuatorCount);
    // Peut-etre une liste de subscribe ici
    //aIMUSubHandle = aGravityNode.subscribe("imu", 1000, &WMGravityModule::imuCallback, this);
}

CompensatedTorqueVector WMGravityModule::process()
{
    tf::StampedTransform s;
    s.getRotation().


}

// Pas certain de ce qu'on devrait retourner, geometry_msgs::PointStamped peut être?
std::vector<tf::StampedTransform>  WMGravityModule::retrievePositionFromTF()
{
    // On utilise aTFNames pour trouver les TFs des joints
    // Récupère la position du TF du joint passé en paramètre
}

void WMGravityModule::retriveRobotConfiguration()
{
    urdf::LinkConstSharedPtr lLink;

    lLink = aURDFModel.links_.at("right_clavicular_link");
    aLinkMass[0] = lLink->inertial->mass;

    lLink = aURDFModel.links_.at("right_upper_arm_upper_link");
    aLinkMass[1] = lLink->inertial->mass;

    lLink = aURDFModel.links_.at("right_upper_arm_lower_link");
    aLinkMass[2] = lLink->inertial->mass;

    lLink = aURDFModel.links_.at("right_elbow_pitch_joint");
    aLinkMass[3] = lLink->inertial->mass;

    lLink = aURDFModel.links_.at("right_forearm_lower_link");
    aLinkMass[4] = lLink->inertial->mass;

    lLink = aURDFModel.links_.at("right_wrist_upper_link");
    aLinkMass[5] = lLink->inertial->mass;

    lLink = aURDFModel.links_.at("right_wrist_lower_link");
    aLinkMass[6] = lLink->inertial->mass;

    lLink = aURDFModel.links_.at("right_socket_link");
    aLinkMass[7] = lLink->inertial->mass;
}

//void WMGravityModule::imuCallback(const sensor_msgs::Imu::ConstPtr& pIMUMessage)
//{
//    // http://wiki.ros.org/Robots/evarobot/Tutorials/indigo/IMU
//    // On set atomiquement des attributs de classes (float_ ou l'objet orientation.
//}