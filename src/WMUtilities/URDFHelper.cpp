//
// Created by olavoie on 15/03/19.
//
#include <ros/ros.h>
#include <URDFHelper.h>

#include "URDFHelper.h"

using namespace wm_admittance::utilities;

/**
 * \brief Récupère les données nécessaires avec un fichier URDF
 * \param[in] pURDFFilePath Le chemin absolu vers le fichier URDF
 * \param pActuatorCount Le nombre de joint à considérer
 * \return Les données nécessaires récupérées du URDF
 */
RobotData URDFHelper::getRobotData(const std::string& pURDFFilePath, const size_t pActuatorCount)
{
    if (!aURDFModel.initFile(pURDFFilePath))
    {
        ROS_ERROR("Failed to parse urdf file '%s'", pURDFFilePath.c_str());
        throw std::runtime_error("Failed to parse urdf file " + pURDFFilePath);
    }

    return retrieveDesiredData(pActuatorCount);
}

/**
 * \brief Récupère les données nécessaires avec le paramètre du serveur
 * \param pActuatorCount Le nombre de joint à considérer
 * \return Les données nécessaires récupérées du URDF
 *
 * \note Il faut que ROS contienne le URDF (via sara_description)
 */
RobotData URDFHelper::getRobotData(const size_t pActuatorCount)
{
    if (!aURDFModel.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf from param");
        throw std::runtime_error("Failed to parse urdf from param ");
    }

    return retrieveDesiredData(pActuatorCount);
}

/**
 * \brief Récupère les données désidées pour le calcule de l'admittance
 * \param pActuatorCount Le nombre de joint à considérer
 * \return Les données nécessaires récupérées du URDF
 */
RobotData URDFHelper::retrieveDesiredData(const size_t pActuatorCount) {

    RobotData lRobotData;
    lRobotData.aLinkMass.resize(pActuatorCount);
    lRobotData.aInertiaTensor.resize(pActuatorCount);
    lRobotData.aCenterOfMass.resize(pActuatorCount);

    std::vector<std::string> lLinkName {"right_clavicular_link", "right_upper_arm_upper_link", "right_upper_arm_lower_link",
                                        "right_forearm_upper_link", "right_wrist_upper_link", "right_wrist_lower_link", "right_socket_link"};

    size_t lActuatorCount{0};
    for (std::string & linkName : lLinkName)
    {
        urdf::LinkConstSharedPtr lLink;
        lLink = aURDFModel.links_.at(linkName);

        lRobotData.aLinkMass[lActuatorCount] = lLink->inertial->mass;

        lRobotData.aCenterOfMass[lActuatorCount].setValue(lLink->inertial->origin.position.x,
                                                          lLink->inertial->origin.position.y,
                                                          lLink->inertial->origin.position.z);

        lRobotData.aInertiaTensor[lActuatorCount].setValue(lLink->inertial->ixx, lLink->inertial->ixy, lLink->inertial->ixz,
                                                           lLink->inertial->ixy, lLink->inertial->iyy, lLink->inertial->iyz,
                                                           lLink->inertial->ixz, lLink->inertial->iyz, lLink->inertial->izz);
        ++lActuatorCount;
    }

    return lRobotData;
}