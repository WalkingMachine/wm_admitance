//
// Created by olavoie on 17/03/19.
//

#include <gtest/gtest.h>
#include <memory>
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
        char const* lTemp = getenv("HOME");
        if (lTemp == NULL) 
        {
            throw std::runtime_error("Couldn't retrieve environment variable HOME");
        } 
        else 
        {
            aURDFFilePath = std::string(getenv("HOME")) + "/sara_ws/src/sara_description/urdf/model.urdf";
        }
        aGravityModel = std::unique_ptr<WMGravityModel>(new WMGravityModel(s, 7));
    }

    void TearDown() override
    {

    }

    std::vector<std::string> s{""};
    std::string aURDFFilePath;
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


