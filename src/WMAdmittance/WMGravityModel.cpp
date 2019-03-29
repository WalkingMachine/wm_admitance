// \file WMGravityModel.cpp
// \brief Definition of gravity module.
// \author Kevin Blackburn
// \author Olivier Lavoie
// \author Véronica Romero

#include "WMGravityModel.h"

#include <ros/ros.h>
#include <stdexcept>
#include <tf/transform_listener.h>
#include <WMGravityModel.h>
#include "../WMUtilities/URDFHelper.h"


using namespace wm_admittance;

namespace
{
    const std::string gBaseTFName = "base_link";
}

/**
 * \brief Constructeur par défaut qui initialize les structures nécessaire
 *        au calcul des torques compensés des joints.
 * \param[in] pTFNames
 * \param pActuatorCount Le nombre de joints à considérer
 * \param pVerboseMode Le mode d'affichage
 *
 * \note Le nombre de joint peut différer du nombre de référentiels, car il est possible que
 *       certains joints ne peuvent pas capter le torque appliqué.
 */
WMGravityModel::WMGravityModel(const std::vector<std::string>& pTFNames, size_t pActuatorCount, bool pVerboseMode) :
    aTFNames(pTFNames),
    aActuatorCount(pActuatorCount),
    aVerboseMode(pVerboseMode)
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
    aRobotData = lURDFParser.getRobotData(aActuatorCount);
}

/**
 * \brief Fonction principale qui doit être appelée pour calculer les torques compensés
 * \return La lsite des torques compensés
 *
 * \note Cette fonction est normalement appelée par la classe WMAdmittance
 */
CompensatedTorqueVector WMGravityModel::process()
{
    aJointTransform = retrievePositionFromTF();
    retrieveTransformInformation();

    aAccelerationVector[0].setValue(0.0, 0.0, 9.81); // adding gravity

    aBackwardForce[aActuatorCount].setZero();
    aBackwardTorque[aActuatorCount].setZero();

    for (size_t i{0}; i < aActuatorCount; ++i)
    {
        aAccelerationVector[i + 1] = aRotationMatrix[i].transpose() * aAccelerationVector[i];
        aForce[i] = aRobotData.aLinkMass[i] * aAccelerationVector[i + 1];
    }

    for (size_t i{aActuatorCount}; i > 0; --i)
    {
        aBackwardForce[i - 1]  = aRotationMatrix[i] * aBackwardForce[i] + aForce[i - 1];
        aBackwardTorque[i - 1] = aRotationMatrix[i] * aBackwardTorque[i] + aRobotData.aCenterOfMass[i - 1].cross(aForce[i - 1]) + aTranslationVector[i].cross(aRotationMatrix[i] * aBackwardForce[i]);
        aCompensatedTorque[i - 1] = aBackwardTorque[i -1].z();
    }

    if (aVerboseMode)
    {
        size_t lIndex = 0;
        for (const auto& lTFName : aTFNames)
        {
            ROS_INFO("Compensated Torque of %s: %lf", lTFName.c_str(), aCompensatedTorque[lIndex]);
            ++lIndex;
        }
    }

    return aCompensatedTorque;
}

/**
 * \brief Récupère la liste des référentiels selon les noms fournis à la construction.
 * \return Retourne la liste des référentiels
 *
 * \throw runtime_error Si un référentiel inexistant est accédé 
 */
std::vector<tf::StampedTransform>  WMGravityModel::retrievePositionFromTF()
{
    std::vector<tf::StampedTransform> lTransformList;
    std::string lLastFrame{gBaseTFName};
    for (const std::string& lTFName : aTFNames)
    {
        try
        {
            tf::StampedTransform lTransform;
            ros::Time lNow = ros::Time::now();

            if (aListener.waitForTransform(lLastFrame, lTFName, lNow, ros::Duration(0.01)))
            {
                aListener.lookupTransform(lLastFrame, lTFName, lNow, lTransform);

                lTransformList.emplace_back(std::move(lTransform));
                lLastFrame = lTFName;
            }
        }
        catch (const tf::TransformException& ex)
        {
            ROS_ERROR("Failed to retrieve transform of '%s': %s", lTFName.c_str(), ex.what());
            throw std::runtime_error("Failed to retrieve transform of '" + lTFName + 
                "': " + ex.what());
        }
    }
    return lTransformList;
}

/**
 * \brief Récupère les informations nécessaires dans les objets des référentiels
 */
void WMGravityModel::retrieveTransformInformation()
{
    size_t lActuatorCount{0};
    for (tf::StampedTransform & lTransform : aJointTransform)
    {
        aRotationMatrix[lActuatorCount] = lTransform.getBasis();
        aTranslationVector[lActuatorCount] = lTransform.getOrigin();
        ++lActuatorCount;
    }
    aRotationMatrix[lActuatorCount].setIdentity();
    aTranslationVector[lActuatorCount].setZero();
}
