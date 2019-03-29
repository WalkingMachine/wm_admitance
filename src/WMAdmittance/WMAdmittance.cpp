// \file WMAdmittance.cpp
// \brief Définition de la classe WMAdmittance
// \author Kevin Blackburn
// \author Olivier Lavoie

#include "WMAdmittance.h"
#include <geometry_msgs/Pose.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <stdexcept>

using namespace wm_admittance;
using namespace wm_admittance::utilities;

std::atomic<WMAdmittance*> WMAdmittance::aInstance;
std::mutex WMAdmittance::aMutex;

namespace std
{
    /**
     * \brief Function template qui permet de créer une instance unique.
     * \param pArgs Paramètre variadic pour passer au constructeur de l'instance avec
     *        la méthode du «perfect_fowarding»
     * \tparam InstanceType Le type de l'instance
     * \tparam Args Les arguments à passer au constructeur de l'instance
     * \return Retourne l'instance unique.
     */
    template<typename InstanceType, typename... Args>
    unique_ptr<InstanceType> make_unique(Args&&... pArgs)
    {
        return unique_ptr<InstanceType>(new InstanceType(forward<Args>(pArgs)...));
    }
}

namespace
{
    const std::string gPackageName("wm_admittance");
}

/**
 * \brief Constructeur par défaut qui initialize les configurations
 *         nécessaires pour l'admittance
 */
WMAdmittance::WMAdmittance() :
    ConfigManager("Admittance")
{
    // Start the reconfigure server
    init();

    // Retrieve joint names configuration
    aAdmittanceNode.getParam("wm_admittance/joint_names", aJointNames);

    // Retrieve link names for each joint
    std::vector<std::string> lLinkNames;
    aAdmittanceNode.getParam("wm_admittance/link_names", lLinkNames);

    // Create unique instance of gravity model
    aGravityModel = std::make_unique<WMGravityModel>(lLinkNames, lLinkNames.size());

    // Create transfer function
    aDiscreteTF = std::make_unique<DiscreteTransferFunction>(
       ros::package::getPath(gPackageName) + "/config/filter_admittance.yaml");

    // Initialize public admittance velocity
    for (const auto& lJointName : aJointNames)
    {
        aAdmittanceVelocityMap.emplace(lJointName, 0.0f);
    }

    // Subscribe to the joint_state to obtain current efforts applied on joints
    aJointStateSub = aAdmittanceNode.subscribe("joint_states", 1, &WMAdmittance::jointStateCallback, this);

    aDebugSub = aAdmittanceNode.advertise<geometry_msgs::Pose>("/debugVelocity", 1000);
}

/**
 * \brief Récupère l'instance unique «singleton» de la classe WMAdmittance.
 * \return Retourne l'instance unique
 */
WMAdmittance* WMAdmittance::getInstance()
{
    WMAdmittance* lInstance = aInstance.load(std::memory_order_acquire);
    if (!lInstance)
    {
        std::lock_guard<std::mutex> lLock(aMutex);
        lInstance = aInstance.load(std::memory_order_relaxed);
        if(!lInstance)
        {
            lInstance = new WMAdmittance();
            aInstance.store(lInstance, std::memory_order_release);
        }
    }
    return lInstance;
}

/**
 * \brief Récupère la vitesse calculée par l'admittance d'un joint.
 * \param[in] pJointName Le nom du joint
 * \return Retourne la vitesse de l'admittance
 */
double WMAdmittance::getAdmittanceVelocityFromJoint(const std::string& pJointName) const
{
    double lVelocity = 0.0f;
    const auto& lIter = aAdmittanceVelocityMap.find(pJointName);
    if (lIter != aAdmittanceVelocityMap.end())
    {
        lVelocity = lIter->second;
    }
    return lVelocity;
}

/**
 * \brief Fonction principale qui doit être appelée pour calculer l'admittance.
 *
 * \note Cette fonction doit être appelée en boucle avec l'horloge principale de ROS
 *       pour une meilleure synchronisation.
 */
void WMAdmittance::process()
{
    if (isAdmittanceEnabled())
    {
        std::vector<double> lAdmittanceTorque =
            calculateAdmittanceTorque(aGravityModel->process());

        updateAdmittanceVelocity(
            aDiscreteTF->updateVector(Eigen::Map<Eigen::VectorXd>(lAdmittanceTorque.data(), lAdmittanceTorque.size())));

        if (aVerboseMode)
        {
            geometry_msgs::Pose lMsg;
            for (const auto& lJointName : aJointNames)
            {
                ROS_INFO("Velocity of %s: %lf", lJointName.c_str(), getAdmittanceVelocityFromJoint(lJointName));
            }

            lMsg.position.x = getAdmittanceVelocityFromJoint("right_shoulder_roll_joint");
            lMsg.position.y = getAdmittanceVelocityFromJoint("right_shoulder_pitch_joint");
            lMsg.position.z = getAdmittanceVelocityFromJoint("right_shoulder_yaw_joint");
            lMsg.orientation.x = getAdmittanceVelocityFromJoint("right_elbow_pitch_joint");
            lMsg.orientation.y = getAdmittanceVelocityFromJoint("right_elbow_yaw_joint");
            aDebugSub.publish(lMsg);

        }
    }
}

/**
 * \brief Fonction de «callback» pour récupérer les changements de configuration
 *        dynamiquement au «runtime»
 * \param[in] pConfig Instance contenant les valeurs de chaque configuration.
 *
 * \note Les paramètres sont générés avec le fichier wm_admittance.cfg
 */
void WMAdmittance::onDynamicReconfigureChange(const AdmittanceConfig& pConfig)
{
    aEnableAdmittance = pConfig.enableAdmittance;
    aVerboseMode = pConfig.verboseMode;
}

/**
 * \brief Fonction de «callback» pour sauvegarder les changements de configuration
 *        dynamiquement au «runtime»
 * \param[in] pConfig Instance contenant les valeurs de chaque configuration.
 *
 * \note Non utilisé, car les parmaêtres ne retrouvent pas dans un fichier yaml.
 */
void WMAdmittance::writeConfigFile(const AdmittanceConfig& pConfig)
{
    (void) pConfig;
    // No logic body
}

/**
 * \brief Fonction de «callback» pour lire les paramètres de configuration
 *        dynamiquement au «runtime»
 * \param[in] pConfig Instance contenant les valeurs de chaque configuration.
 *
 * \note Non utilisé, car les parmaêtres ne retrouvent pas dans un fichier yaml.
 */
void WMAdmittance::readConfigFile(AdmittanceConfig& pConfig)
{
    (void) pConfig;
    // No logic body
}

/**
 * \brief Fonction de «callback» pour récupérer les torques courants sur les joints
 * \param[in] pMsg Instance contenant les informations sur les joints
 *
 * \throw runtime_error Si un joint fourni dans le fichier de configuration
 *                      n'apparaît pas dans la liste des joints du message de ROS.
 *
 * \note Cette fonction est enregistré via ROS avec le message «joint_states»
 */
void WMAdmittance::jointStateCallback(const sensor_msgs::JointState& pMsg)
{
    if (!aFirstCheck)
    {
        aFirstCheck = true;
        for (const auto& lMapItem : aAdmittanceVelocityMap)
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
                ROS_ERROR("WMAdmittance: Joint '%s' is not present in topic /joint_states.", lMapItem.first.c_str());
                throw std::runtime_error("WMAdmittance: Joint '" + lMapItem.first + "' is not present in topic /joint_states.");
            }
        }
    }

    aJointState = pMsg;
}

/**
 * \brief Mets à jour les vitesses d'admittance dans le conteneur.
 * \param[in] pAdmittanceVelocity La liste des vitesses
 */
void WMAdmittance::updateAdmittanceVelocity(const std::vector<double>& pAdmittanceVelocity)
{
    size_t lIndex = 0;
    for (const auto& lJointName : aJointNames)
    {
        aAdmittanceVelocityMap[lJointName] = pAdmittanceVelocity[lIndex] * 57.295779513;
        ++lIndex;
    }
}

/**
 * \brief Calcules les torques d'admittance avec les torques compensés
 * \param[in] pCompensatedTorque La liste des torques compensés
 * \return Retourne une liste des torques d'admittance
 */
std::vector<double> WMAdmittance::calculateAdmittanceTorque(const std::vector<double>& pCompensatedTorque)
{
    size_t lIndex = 0;
    std::vector<double> lAdmittanceTorque;
    for (const auto& lJointName : aJointNames)
    {
        double lDeltaTorque = getEffortFromJoint(lJointName) - pCompensatedTorque[lIndex];
        if (lDeltaTorque < 3.0f && lDeltaTorque > -3.0f)
        {
            lDeltaTorque = 0.0f;
        }

        lAdmittanceTorque.emplace_back(lDeltaTorque);
        ++lIndex;
    }
    return lAdmittanceTorque;
}

/**
 * \brief Récupère l'effort sur un joint
 * \param[in] pJointName Le nom du joint
 * \return Retourne l'effort du joint
 */
double WMAdmittance::getEffortFromJoint(const std::string& pJointName) const
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

    return lEffort;
}
