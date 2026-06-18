cmake_minimum_required(VERSION 3.8)
project(srvs_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 필수 의존성 패키지 로드
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. 서비스 파일 등록 (srv 폴더 안에 파일이 있어야 함)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetTargetPose.srv"
)

# 3. 테스트 설정 (기본값)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 4. 패키지 설치 및 등록
ament_package()