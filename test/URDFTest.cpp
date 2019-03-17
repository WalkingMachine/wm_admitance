//
// Created by olavoie on 17/03/19.
//

#include <gtest/gtest.h>
#include "URDFHelper.h"
#include "ControlTypes.h"

using namespace wm_admitance::utilities;

class URDFParse_Unit_Test : public testing::Test
{
public:
    URDFParse_Unit_Test() = default;
    virtual ~URDFParse_Unit_Test() = default;

    void SetUp() override
    {
        aURDFFilePath = "/home/olavoie/sara_ws/src/sara_description/urdf/model.urdf";
    }

    void TearDown() override
    {

    }


    RobotData   lRobotData;
    std::string aURDFFilePath;
};

TEST_F(URDFParse_Unit_Test, TestUrdfParser)
{
    urdf::Model aURDFModel;
    aURDFModel.initFile(aURDFFilePath);

    lRobotData.aLinkMass.resize(7);
    lRobotData.aInertiaTensor.resize(7);
    lRobotData.aCenterOfMass.resize(7);

    //URDFHelper  aURDFHelper;
    //aRobotData = aURDFHelper.getRobotData(aURDFFilePath, 7);

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

        lRobotData.aInertiaTensor[lActuatorCount].setValue(lLink->inertial->ixx, lLink->inertial->ixy,
                                                           lLink->inertial->ixz,
                                                           lLink->inertial->ixy, lLink->inertial->iyy,
                                                           lLink->inertial->iyz,
                                                           lLink->inertial->ixz, lLink->inertial->iyz,
                                                           lLink->inertial->izz);
        ++lActuatorCount;
    }


}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
