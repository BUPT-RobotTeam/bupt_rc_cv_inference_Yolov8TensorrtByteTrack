cmake_minimum_required(VERSION 3.10)
project(bupt_rc_cv_inference_Yolov8TensorrtBytetrack)

add_definitions(-std=c++17)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories("/usr/include/eigen3")

# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# ============= cuda ============
find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

# ============= tensorrt ============
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("Embed_platform on")
  include_directories(/usr/include/aarch64-linux-gnu)
  link_directories(/usr/lib/aarch64-linux-gnu)
else()
  message("Embed_platform off")
  include_directories(/usr/include/x86_64-linux-gnu)
  link_directories(/usr/lib/x86_64-linux-gnu)
endif()

# ============ opencv ============
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# =========== bytetrack lib ===========
include_directories(${PROJECT_SOURCE_DIR}/bytetrack/include)
file(GLOB_RECURSE SRCS01 ${PROJECT_SOURCE_DIR}/bytetrack/src/*.cpp)
add_library(bytetrack SHARED ${SRCS01})
target_link_libraries(bytetrack cudart nvinfer ${OpenCV_LIBS})

# ============= yolov8 lib =============
include_directories(${PROJECT_SOURCE_DIR}/yolo/include)
include_directories(${PROJECT_SOURCE_DIR}/yolo/plugin)
file(GLOB_RECURSE SRCS02 ${PROJECT_SOURCE_DIR}/yolo/src/*.cpp ${PROJECT_SOURCE_DIR}/yolo/src/*.cu ${PROJECT_SOURCE_DIR}/yolo/plugin/*.cu)
cuda_add_library(yolo_infer SHARED ${SRCS02})
target_link_libraries(yolo_infer nvinfer cudart ${OpenCV_LIBS})

# ============= yaml-cpp =============
find_package(yaml-cpp REQUIRED)


# ============= about ros =============
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bupt_rc_cv_interfaces REQUIRED)

# ======== main execute file ========
# add_executable(main ${PROJECT_SOURCE_DIR}/main.cpp)
# target_link_libraries(main bytetrack yolo_infer)
# target_link_libraries(main ${OpenCV_LIBS} yaml-cpp::yaml-cpp)

# inference publisher
add_executable(inference_publisher
    publisher.cpp
)

target_link_libraries(inference_publisher bytetrack yolo_infer)
target_link_libraries(inference_publisher ${OpenCV_LIBS} yaml-cpp::yaml-cpp)
ament_target_dependencies(inference_publisher rclcpp bupt_rc_cv_interfaces)

install(TARGETS inference_publisher
    DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS yolo_infer
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bytetrack
    DESTINATION lib/${PROJECT_NAME}
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
