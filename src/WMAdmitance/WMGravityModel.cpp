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

WMGravityModel::WMGravityModel(const std::vector<std::string>& pTFNames, const std::string& pURDFFilePath) :
    aTFNames(pTFNames),
    aURDFFilePath(pURDFFilePath)
{
    // Initialisation du URDF parser
    if (!aURDFModel.initFile(aURDFFilePath))
    {
         ROS_ERROR("Failed to parse urdf file '%s'", pURDFFilePath.c_str());
         //throw std::runtime_error("Failed to parse urdf file " + pURDFFilePath);
    }
}

CompensatedTorqueVector WMGravityModel::process()
{
    CompensatedTorqueVector lCompensatedTorque = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};


    //ROS_INFO("%lf", lCompensatedTorque[0]);

    return lCompensatedTorque;
}

// Pas certain de ce qu'on devrait retourner, geometry_msgs::PointStamped peut être?
std::vector<tf::StampedTransform>  WMGravityModel::retrievePositionFromTF()
{
    // On utilise aTFNames pour trouver les TFs des joints
    // Récupère la position du TF du joint passé en paramètre

    return std::vector<tf::StampedTransform>();
}