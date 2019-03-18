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
    URDFHelper  aURDFHelper;
    std::string aURDFFilePath;
};

TEST_F(URDFParse_Unit_Test, TestUrdfParser)
{
    lRobotData = aURDFHelper.getRobotData(aURDFFilePath, 7);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
