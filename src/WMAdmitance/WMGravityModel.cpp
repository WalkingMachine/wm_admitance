// \file WMGravityModel.cpp
// \brief Definition of gravity module.
// Created by kevin on 07/03/2019.

#include "WMGravityModel.h"

#include <ros/ros.h>
#include <stdexcept>
#include <tf/transform_listener.h>
#include <WMGravityModel.h>
#include "URDFHelper.h"


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
    aJointTransform.resize(aActuatorCount);
    aRotationMatrix.resize(aActuatorCount + 1);
    aTranslationVector.resize(aActuatorCount + 1);
    aAccelerationVector.resize(aActuatorCount + 1);
    aForce.resize(aActuatorCount);
    aBackwardForce.resize(aActuatorCount + 1);
    aBackwardTorque.resize(aActuatorCount + 1);
    aCompensatedTorque.resize(aActuatorCount);

    // Get only needed information from URDF file
    utilities::URDFHelper lURDFParser;
    aRobotData = lURDFParser.getRobotData(aURDFFilePath, aActuatorCount);

    // Peut-etre une liste de subscribe ici
    //aIMUSubHandle = aGravityNode.subscribe("imu", 1000, &WMGravityModule::imuCallback, this);
}

CompensatedTorqueVector WMGravityModule::process()
{
    aJointTransform = retrievePositionFromTF();
    retrieveTransformInformation();

    aAccelerationVector[0].setValue(0.0, 0.0, -9.81); // adding gravity

    aBackwardForce[aActuatorCount + 1].setZero();
    aBackwardTorque[aActuatorCount + 1].setZero();

    for (size_t i{0}; i < aActuatorCount; ++i)
    {
        aAccelerationVector[i + 1] = aRotationMatrix[i].transpose() * aAccelerationVector[i];
        aForce[i] = aRobotData.aLinkMass[i] * aAccelerationVector[i + 1];
    }

    for (size_t i{aActuatorCount}; i > 0; --i)
    {
        aBackwardForce[i - 1]  = aRotationMatrix[i] * aBackwardForce[i] + aForce[i - 1];
        aBackwardTorque[i - 1] = aRotationMatrix[i] * aBackwardTorque[i] + aRobotData.aCenterOfMass[i - 1].cross(aForce[i - 1]) + aTranslationVector[i].cross(aRotationMatrix[i] * aBackwardForce[i]);
        aCompensatedTorque[i] = aBackwardTorque[i -1].z();
    }

    return aCompensatedTorque;
}

// Pas certain de ce qu'on devrait retourner, geometry_msgs::PointStamped peut être?
std::vector<tf::StampedTransform>  WMGravityModule::retrievePositionFromTF()
{
    // On utilise aTFNames pour trouver les TFs des joints
    // Récupère la position du TF du joint passé en paramètre
}

void WMGravityModule::retrieveTransformInformation()
{
    size_t lActuatorCount{0};
    for (tf::StampedTransform & lTransform : aJointTransform)
    {
        aRotationMatrix[lActuatorCount] = lTransform.getBasis();
        aTranslationVector[lActuatorCount] = lTransform.getOrigin();
        ++lActuatorCount;
    }
    aRotationMatrix[lActuatorCount + 1].setIdentity();
    aTranslationVector[lActuatorCount + 1].setZero();
}

//void WMGravityModule::imuCallback(const sensor_msgs::Imu::ConstPtr& pIMUMessage)
//{
//    // http://wiki.ros.org/Robots/evarobot/Tutorials/indigo/IMU
//    // On set atomiquement des attributs de classes (float_ ou l'objet orientation.
//}