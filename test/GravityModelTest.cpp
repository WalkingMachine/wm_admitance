//
// Created by olavoie on 17/03/19.
//

#include <gtest/gtest.h>
#include "WMGravityModel.h"
#include "ControlTypes.h"

using namespace wm_admitance::utilities;
using namespace wm_admitance;

class GravityModel_Unit_Test : public testing::Test
{
public:
    GravityModel_Unit_Test() = default;
    virtual ~GravityModel_Unit_Test() = default;

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

TEST_F(GravityModel_Unit_Test, TestUrdfParser)
{
    std::vector<std::string> s{""};
    WMGravityModule gravityModel(s, aURDFFilePath, 7);







}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


