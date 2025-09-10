cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

include(cmake/tesseract_open3d_macros.cmake)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Open3D CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    /*/install/*
    /*/devel/*
    /*/gtest/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

add_library(
  ${PROJECT_NAME}
  src/world.cpp
  src/sensors.cpp
  src/simulator.cpp
  src/utils.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen Open3D::Open3D)
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS} ${TESSERACT_BACKTRACE_DEFINITION})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PUBLIC
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

add_executable(${PROJECT_NAME}_demo src/multi_sensor_demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo PRIVATE ${PROJECT_NAME} Eigen3::Eigen Open3D::Open3D)
target_compile_options(${PROJECT_NAME}_demo PRIVATE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_demo PRIVATE ${TESSERACT_COMPILE_DEFINITIONS}
                                                        ${TESSERACT_BACKTRACE_DEFINITION})
target_compile_definitions(${PROJECT_NAME}_demo PUBLIC OPEN3D_RESOURCES_DIR="${CMAKE_INSTALL_PREFIX}")
target_clang_tidy(${PROJECT_NAME}_demo ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_demo PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_demo
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_demo PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                        "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_demo
                  CFG_EXTRAS cmake/tesseract_open3d_macros.cmake)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

# if(TESSERACT_ENABLE_TESTING OR TESSERACT_COMMON_ENABLE_TESTING) enable_testing() add_run_tests_target(ENABLE
# ${TESSERACT_ENABLE_RUN_TESTING}) add_subdirectory(test) endif()

if(TESSERACT_PACKAGE)
  cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS "libeigen3-dev" "libopen3d-dev"
    WINDOWS_DEPENDS "Eigen3" "Open3d")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
