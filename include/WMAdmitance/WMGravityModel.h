// \file WMGravityModel.h
// \brief Declaration of gravity module.
// Created by kevin on 07/03/2019.

#ifndef WM_GRAVITY_MODULE_H
#define WM_GRAVITY_MODULE_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>

namespace wm_admitance
{
    /**
     * \brief Type of vector of compensated torque
     */
    using CompensatedTorqueVector = std::vector<double>;

    class WMGravityModel final
    {
    public:
        WMGravityModel(const std::vector<std::string>& pTFNames);
        ~WMGravityModel() noexcept = default;

        CompensatedTorqueVector process();

    private:

        std::vector<tf::Quaternion> retrievePositionFromTF();


        const tf::TransformListener aListener;
        urdf::Model aURDFModel;

        const std::vector<std::string> aTFNames;
        const std::string aURDFFilePath;
    };
} // namespace wm_admitance
#endif // WM_GRAVITY_MODULE_H

