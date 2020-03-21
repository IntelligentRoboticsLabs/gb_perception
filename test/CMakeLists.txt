ament_add_gtest(openpose_detector_test openpose_detector_test.cpp)
ament_target_dependencies(openpose_detector_test ${dependencies})

target_link_libraries(openpose_detector_test  
  people_detector
  ${OpenCV_LIBS}
  ${OpenPose_LIBS}
)

install(DIRECTORY test_images DESTINATION share/${PROJECT_NAME})
