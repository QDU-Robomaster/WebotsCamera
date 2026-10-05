add_executable(webots_bayer_test webots_bayer_test.cpp)
target_include_directories(webots_bayer_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_link_libraries(webots_bayer_test PRIVATE xr)
target_compile_features(webots_bayer_test PRIVATE cxx_std_20)
add_test(NAME webots_bayer_test COMMAND webots_bayer_test)

# 只在 LibXR 的 webots 系统下编译链接整个驱动 / Link the whole driver only on the
# LibXR webots system.
if(WEBOTS_HOME)
  add_executable(webots_camera_link_test webots_camera_link_test.cpp)
  target_include_directories(webots_camera_link_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
  target_link_libraries(webots_camera_link_test PRIVATE xr)
  target_compile_features(webots_camera_link_test PRIVATE cxx_std_20)
  add_test(NAME webots_camera_link_test COMMAND webots_camera_link_test)
endif()
