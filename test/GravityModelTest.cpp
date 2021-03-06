//
// Created by olavoie on 17/03/19.
//

#include <gtest/gtest.h>
#include <memory>
#include "WMGravityModel.h"
#include "ControlTypes.h"

using namespace wm_admittance::utilities;
using namespace wm_admittance;

class GravityModel_Unit_Test : public testing::Test
{
public:
    GravityModel_Unit_Test() = default;
    virtual ~GravityModel_Unit_Test() = default;

    void SetUp() override
    {
        aGravityModel = std::unique_ptr<WMGravityModel>(new WMGravityModel(s, 7));
    }

    void TearDown() override
    {

    }

    std::vector<std::string> s{""};
    std::unique_ptr<WMGravityModel> aGravityModel;
};

TEST_F(GravityModel_Unit_Test, TestGravityModel)
{
    CompensatedTorqueVector ss = aGravityModel->process();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "salut");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


