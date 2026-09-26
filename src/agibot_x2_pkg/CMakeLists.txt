cmake_minimum_required(VERSION 3.8)
project(agibot_x2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  agibot_x2_pkg/scene_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scene_publisher
)

install(PROGRAMS
  scripts/library_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME library_manager_node
)

install(DIRECTORY scripts/vision scripts/sorting scripts/input
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf scripts meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
