catkin_add_gtest(URDFTest URDFTest.cpp ${SOURCE_FILES})
target_link_libraries(URDFTest wm_admitance)
set_target_properties(URDFTest PROPERTIES LINKER_LANGUAGE CXX )