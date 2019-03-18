//
// Created by olavoie on 15/03/19.
//

#ifndef WM_ADMITANCE_URDFHELPER_H
#define WM_ADMITANCE_URDFHELPER_H

#include "ControlTypes.h"
#include <string>
#include <urdf/model.h>

namespace wm_admitance
{
    namespace utilities
    {
        class URDFHelper
        {
        public:
            URDFHelper() = default;
            ~URDFHelper() = default;

            RobotData getRobotData(const std::string & pURDFFilePath, const size_t pActuatorCount);
            RobotData getRobotData(const size_t pActuatorCount);

        private:
            RobotData retrieveDesiredData(const size_t pActuatorCount);
            urdf::Model aURDFModel;
        };
    }
}

#endif //WM_ADMITANCE_URDFHELPER_H
