cmake_minimum_required(VERSION 3.20)
project(usb_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(usb_msgs REQUIRED)
find_package(usb_driver_srvs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED libusb-1.0)


include_directories(include SYSTEM)

add_library(usb_driver_components SHARED
  src/usb.cpp
  src/usb_node.cpp
)

target_compile_definitions(usb_driver_components
  PRIVATE "USB_DRIVER_BUILDING_DLL"
)

target_link_libraries(usb_driver_components PUBLIC
  ${std_msgs_TARGETS}
  ${usb_msgs_TARGETS}
  ${usb_driver_srvs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  usb-1.0
)

rclcpp_components_register_node(usb_driver_components
  PLUGIN "usb_driver::UsbNode"
  EXECUTABLE usb_node)

install(TARGETS
  usb_driver_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
