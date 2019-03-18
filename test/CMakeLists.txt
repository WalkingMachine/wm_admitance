cmake_minimum_required(VERSION 3.5)

catkin_add_gtest(URDFTest URDFTest.cpp ${SOURCE_FILES})
target_link_libraries(URDFTest wm_admitance)
set_target_properties(URDFTest PROPERTIES LINKER_LANGUAGE CXX )

catkin_add_gtest(GravityModelTest GravityModelTest.cpp ${SOURCE_FILES})
target_link_libraries(GravityModelTest wm_admitance)
set_target_properties(GravityModelTest PROPERTIES LINKER_LANGUAGE CXX )