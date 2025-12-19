cmake_minimum_required(VERSION 3.8)
project(tj_arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  meshes
  rviz
  # config  <-- 如果你有 config 文件夹，把注释打开
  DESTINATION share/${PROJECT_NAME}
)

# 3. 导出包
ament_package()