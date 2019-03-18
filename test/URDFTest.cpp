//
// Created by olavoie on 17/03/19.
//

#include <gtest/gtest.h>
#include <stdexcept>
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
        char const* lTemp = getenv("HOME");
        if (lTemp == NULL) 
        {
            throw std::runtime_error("Couldn't retrieve environment variable HOME");
        } 
        else 
        {
            aURDFFilePath = std::string(getenv("HOME")) + "/sara_ws/src/sara_description/urdf/model.urdf";
        }
    }

    void TearDown() override
    {

    }


    RobotData   aRobotData;
    std::string aURDFFilePath;
};

TEST_F(URDFParse_Unit_Test, TestUrdfParser)
{
    URDFHelper lURDFHepler(aURDFFilePath);

    aRobotData = lURDFHepler.getRobotData(7);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
