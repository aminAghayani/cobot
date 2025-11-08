cmake_minimum_required(VERSION 3.8)
project(mycobot_280_gazebo)

find_package(ament_cmake REQUIRED)

# Runtime deps we use in launch/URDF/plugins
# (don't fail the build if some are not present at CMake time)
find_package(ros_gz_sim QUIET)
find_package(gz_ros2_control QUIET)
find_package(ros2_control QUIET)
find_package(ros2_controllers QUIET)
find_package(ros_gz_bridge QUIET)
find_package(ros_gz QUIET)
find_package(xacro QUIET)

# ---- Install data (only if the folder exists) ----
foreach(dir IN ITEMS launch urdf config worlds)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

# ---- Optional example scripts (install only if present) ----
set(SCRIPT_LIST
  scripts/move_mycobot.py
  scripts/log_joint_data.py
  scripts/move_single_joint.py
  scripts/plot_joint_feedback.py
  scripts/multi_joint_trajectory.py
  scripts/log_rms_tracking.py
)
foreach(py IN LISTS SCRIPT_LIST)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${py})
    install(PROGRAMS ${py} DESTINATION lib/${PROJECT_NAME})
  endif()
endforeach()

ament_package()

