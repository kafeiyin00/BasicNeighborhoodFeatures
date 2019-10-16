

add_executable(test_env
        test_env.cpp)
target_link_libraries(test_env
        glog
        libbnf
        libPangoVis
        ${PCL_LIBRARIES}
        ${Pangolin_LIBRARIES})