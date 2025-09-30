cmake_minimum_required(VERSION 3.5)
project(rby1_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  maps
  rviz2
  # urdf 폴더를 설치 목록에 추가합니다.
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
