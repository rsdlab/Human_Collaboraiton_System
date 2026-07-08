cmake_minimum_required(VERSION 3.8)
project(collaboration_manipulation_module)

# 必要なパッケージ
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(collaboration_manipulation_message REQUIRED)

# Pythonモジュール（setup.pyを使う場合はこの行が必要）
ament_python_install_package(${PROJECT_NAME})

# 依存のエクスポート
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs action_msgs collaboration_manipulation_message)

# パッケージのインストール設定
ament_package()
