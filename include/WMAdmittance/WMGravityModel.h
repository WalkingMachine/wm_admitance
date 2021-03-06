// \file WMGravityModel.h
// \brief Declaration of gravity module.
// \author Kevin Blackburn
// \author Olivier Lavoie

#ifndef WM_GRAVITY_MODULE_H
#define WM_GRAVITY_MODULE_H

#include "../WMUtilities/ControlTypes.h"
#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>

namespace wm_admittance
{
    /**
     * \brief Type of vector of compensated torque
     */
    using CompensatedTorqueVector = std::vector<double>;

    /**
     * \brief Une classe qui obtient les torques compensés sur les joints
     *        avec le modèle de gravité
     *
     * \details Cette calcules les torques compensés selon les noms des référentiels
     *          passés en paramètre au constructeur.
     */
    class WMGravityModel final
    {
    public:
        WMGravityModel(const std::vector<std::string>& pTFNames,
                       size_t pActuatorCount,
                       bool pVerboseMode = false);
        ~WMGravityModel()  = default;

        CompensatedTorqueVector process();

    private:

        std::vector<tf::StampedTransform> retrievePositionFromTF();
        void retrieveTransformInformation();

        const tf::TransformListener aListener;
        size_t aActuatorCount;
        bool aVerboseMode;

        const std::vector<std::string> aTFNames;
        const std::string aURDFFilePath;

        std::vector<tf::StampedTransform> aJointTransform;
        std::vector<tf::Matrix3x3>        aRotationMatrix;
        std::vector<tf::Vector3>          aTranslationVector;
        std::vector<tf::Vector3>          aAccelerationVector;
        std::vector<tf::Vector3>          aForce;
        std::vector<tf::Vector3>          aBackwardForce;
        std::vector<tf::Vector3>          aBackwardTorque;
        CompensatedTorqueVector           aCompensatedTorque;

        utilities::RobotData aRobotData;
    };
} // namespace wm_admittance
#endif // WM_GRAVITY_MODULE_H

