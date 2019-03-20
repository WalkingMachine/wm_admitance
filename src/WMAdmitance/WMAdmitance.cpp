// \file WMAdmitance.cpp
// \brief Définition de la classe WMAdmitance
// \author Kevin Blackburn
// \author Olivier Lavoie

#include "WMAdmitance.h"

#include <ros/ros.h>
#include <stdexcept>

using namespace wm_admitance;
using namespace wm_admitance::utilities;

std::atomic<WMAdmitance*> WMAdmitance::aInstance;
std::mutex WMAdmitance::aMutex;

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

/**
 * \brief Constructeur par défaut qui initialize les configurations
 *         nécessaires pour l'admittance
 */
WMAdmitance::WMAdmitance()
{
    // Retrieve joint names configuration
    aAdmitanceNode.getParam("sara_admitance/joint_names", aJointNames);

    // Set callback for dynamic reconfiguration
    aDynamicConfigServer.setCallback(boost::bind(&WMAdmitance::dynamicReconfigureCallback, this, _1));

    // Retrieve link names for each joint
    std::vector<std::string> lLinkNames;
    for (const std::string& lJointName : aJointNames)
    {
        std::vector<std::string> lLinkName;
        aAdmitanceNode.getParam("sara_admitance/" + lJointName, lLinkName);
        lLinkNames.emplace_back(std::move(lLinkName[0]));
    }

    // Create unique instance of gravity model
    aGravityModel = std::make_unique<WMGravityModel>(lLinkNames, 7);

    // Create discrete Transfer function
    TransferFunctionCoefficient lFunctionCoeff;
    lFunctionCoeff.aNumeratorFactor = Eigen::ArrayXXd::Zero(aJointNames.size(), 2 + 1);
    lFunctionCoeff.aDenominatorFactor = Eigen::ArrayXXd::Zero(aJointNames.size(), 2);

    for (int i = 0; i < aJointNames.size(); i++)
    {
        lFunctionCoeff.aDenominatorFactor.row(i) << -1.970052211604768, 0.970446261452074;
        lFunctionCoeff.aNumeratorFactor.row(i)   << 0.004925623091321, 0.009851246182642, 0.004925623091321;
    }

    aDiscreteTF = std::make_unique<DiscreteTransferFunction>(
            aJointNames.size(),
            std::move(lFunctionCoeff),
            2);

    // Initialize public admitance velocity
    for (const auto& lJointName : aJointNames)
    {
        aAdmitanceVelocityMap.emplace(lJointName, 0.0f);
    }

    // Subscribe to the joint_state to obtain current efforts applied on joints
    aJointStateSub = aAdmitanceNode.subscribe("joint_states", 1, &WMAdmitance::jointStateCallback, this);
}

/**
 * \brief Récupère l'instance unique «singleton» de la classe WMAdmitance.
 * \return Retourne l'instance unique
 */
WMAdmitance* WMAdmitance::getInstance()
{
    WMAdmitance* lInstance = aInstance.load(std::memory_order_acquire);
    if (!lInstance)
    {
        std::lock_guard<std::mutex> lLock(aMutex);
        lInstance = aInstance.load(std::memory_order_relaxed);
        if(!lInstance)
        {
            lInstance = new WMAdmitance();
            aInstance.store(lInstance, std::memory_order_release);
        }
    }   
    return lInstance;
}

/**
 * \brief Récupère la vitesse calculée par l'admitance d'un joint.
 * \param[in] pJointName Le nom du joint
 * \return Retourne la vitesse de l'admitance
 */
double WMAdmitance::getAdmitanceVelocityFromJoint(const std::string& pJointName) const
{
    double lVelocity = 0.0f;
    const auto& lIter = aAdmitanceVelocityMap.find(pJointName);
    if (lIter != aAdmitanceVelocityMap.end())
    {
        lVelocity = lIter->second;
    }
    return lVelocity;
}

/**
 * \brief Fonction principale qui doit être appelée pour calculer l'admitance.
 *
 * \note Cette fonction doit être appelée en boucle avec l'horloge principale de ROS
 *       pour une meilleure synchronisation.
 */
void WMAdmitance::process()
{
    if (isAdmitanceEnabled())
    {
        std::vector<double> lAdmitanceTorque =
            calculateAdmitanceTorque(aGravityModel->process());

        updateAdmitanceVelocity(
            aDiscreteTF->updateVector(Eigen::Map<Eigen::VectorXd>(lAdmitanceTorque.data(), lAdmitanceTorque.size())));

        if (aVerboseMode)
        {
            for (const auto& lJointName : aJointNames)
            {
                ROS_INFO("Velocity of %s: %lf", lJointName.c_str(), getAdmitanceVelocityFromJoint(lJointName));
            }
        }
    }
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

/**
 * \brief Fonction de «callback» pour récupérer les changements de configuration
 *        dynamiquement au «runtime»
 * \param[in] pConfig Instance contenant les valeurs de chaque configuration.
 *
 * \note Les paramètres sont générés avec le fichier sara_admitance.cfg
 */
void WMAdmitance::dynamicReconfigureCallback(sara_admitance::sara_admitanceConfig &pConfig)
{
    aEnableAdmitance = pConfig.enable_admitance;
    aVerboseMode = pConfig.verbose_mode;
}

/**
 * \brief Mets à jour les vitesses d'admittance dans le conteneur.
 * \param[in] pAdmitanceVelocity La liste des vitesses
 */
void WMAdmitance::updateAdmitanceVelocity(const std::vector<double>& pAdmitanceVelocity)
{
    size_t lIndex = 0;
    for (const auto& lJointName : aJointNames)
    {
        aAdmitanceVelocityMap[lJointName] = pAdmitanceVelocity[lIndex];
        ++lIndex;
    }
}

/**
 * \brief Calcules les torques d'admittance avec les torques compensés
 * \param[in] pCompensatedTorque La liste des torques compensés
 * \return Retourne une liste des torques d'admitance
 */
std::vector<double> WMAdmitance::calculateAdmitanceTorque(const std::vector<double>& pCompensatedTorque)
{
    size_t lIndex = 0;
    std::vector<double> lAdmitanceTorque;
    for (const auto& lJointName : aJointNames)
    {
        lAdmitanceTorque.emplace_back(getEffortFromJoint(lJointName) - pCompensatedTorque[lIndex]);
        ++lIndex;
    }
    return lAdmitanceTorque;
}

/**
 * \brief Récupère l'effort sur un joint
 * \param[in] pJointName Le nom du joint
 * \return Retourne l'effort du joint
 */
double WMAdmitance::getEffortFromJoint(const std::string& pJointName) const
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