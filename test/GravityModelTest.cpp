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
    GravityModel_Unit_Test() : aURDFFilePath{"/home/olavoie/sara_ws/src/sara_description/urdf/model.urdf"},  aGravityModel(s, aURDFFilePath, 7) {}
    virtual ~GravityModel_Unit_Test() = default;

    void SetUp() override
    {
        aURDFFilePath = "/home/olavoie/sara_ws/src/sara_description/urdf/model.urdf";
    }

    void TearDown() override
    {

    }


    std::vector<std::string> s{""};
    std::string aURDFFilePath = "/home/olavoie/sara_ws/src/sara_description/urdf/model.urdf";
    WMGravityModel aGravityModel;
};

TEST_F(GravityModel_Unit_Test, TestGravityModel)
{

    CompensatedTorqueVector ss = aGravityModel.process();
}


int main(int argc, char **argv)
{
    std::vector<std::string> s{""};
    std::string aURDFFilePath = "/home/olavoie/sara_ws/src/sara_description/urdf/model.urdf";
    WMGravityModel aGravityModel(s, aURDFFilePath, 7);
    CompensatedTorqueVector ss = aGravityModel.process();
    //testing::InitGoogleTest(&argc, argv);
    //return RUN_ALL_TESTS();
}


