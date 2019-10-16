

add_executable(test_env
        test_env.cpp)
target_link_libraries(test_env
        glog
        libbnf
        ${PCL_LIBRARIES})

add_executable(test_featureDetection
        test_featureDetection.cpp)
target_link_libraries(test_featureDetection
        glog
        libbnf
        ${PCL_LIBRARIES})