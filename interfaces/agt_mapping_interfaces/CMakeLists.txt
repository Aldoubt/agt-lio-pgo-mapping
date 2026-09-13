cmake_minimum_required(VERSION 3.8)
project(agt_mapping_interfaces)

find_package(ament_cmake REQUIRED)

# Commit 1 intentionally declares no ROSIDL types. Stable mapping interfaces
# are added only when their contract has received human approval.
ament_package()
