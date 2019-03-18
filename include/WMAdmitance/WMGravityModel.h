// \file WMGravityModel.h
// \brief Declaration of gravity module.
// Created by kevin on 07/03/2019.

#ifndef WM_GRAVITY_MODULE_H
#define WM_GRAVITY_MODULE_H

#include "../WMUtilities/ControlTypes.h"
#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <sensor_msgs/Imu.h>
#include <urdf/model.h>

namespace wm_admitance
{
    /**
     * \brief Type of vector of compensated torque
     */
    using CompensatedTorqueVector = std::vector<double>;

    // Note: Il faudra faire une classe WMAdmitance qui utiliser WMGravityModule.
    class WMGravityModel final
    {
    public:
        WMGravityModel(const std::vector<std::string>& pTFNames, const std::string& pURDFFilePath, size_t pActuatorCount);
        ~WMGravityModel()  = default;


        CompensatedTorqueVector process();

    private:

        std::vector<tf::StampedTransform> retrievePositionFromTF();
        void retrieveTransformInformation();


        const tf::TransformListener aListener;
        size_t aActuatorCount;

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
} // namespace wm_admitance
#endif // WM_GRAVITY_MODULE_H

