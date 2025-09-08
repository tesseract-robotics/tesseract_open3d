#
# @file tesseract_open3d_macros.cmae @brief Common Tesseract Open3d CMake Macros
#
# @author Levi Armstrong @date October 15, 2019 @version TODO @bug No known bugs
#
# @copyright Copyright (c) 2019, Southwest Research Institute
#
# @par License Software License Agreement (Apache License) @par Licensed under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0 @par Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing permissions and limitations under the License.

macro(tesseract_variables)
  if(NOT DEFINED BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS ON)
  endif()

  if(NOT DEFINED TESSERACT_PACKAGE_SOURCE_UPLOAD)
    set(TESSERACT_PACKAGE_SOURCE_UPLOAD OFF)
  endif()

  if(NOT DEFINED TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS)
    set(TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS focal jammy)
  endif()

  if(NOT DEFINED TESSERACT_PACKAGE_SOURCE_DPUT_HOST)
    set(TESSERACT_PACKAGE_SOURCE_DPUT_HOST ppa:levi-armstrong/tesseract-robotics)
  endif()

  if(NOT DEFINED TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT)
    set(TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT 0)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_EXAMPLES)
    set(TESSERACT_ENABLE_EXAMPLES ON)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_CLANG_TIDY)
    set(TESSERACT_ENABLE_CLANG_TIDY OFF)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_CODE_COVERAGE)
    set(TESSERACT_ENABLE_CODE_COVERAGE OFF)
  elseif(TESSERACT_ENABLE_CODE_COVERAGE)
    set(TESSERACT_ENABLE_TESTING ON)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_TESTING)
    set(TESSERACT_ENABLE_TESTING OFF)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_EXAMPLES)
    set(TESSERACT_ENABLE_EXAMPLES ON)
  endif()

  if(NOT DEFINED TESSERACT_WARNINGS_AS_ERRORS)
    set(TESSERACT_WARNINGS_AS_ERRORS ${TESSERACT_ENABLE_TESTING})
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_RUN_TESTING)
    set(TESSERACT_ENABLE_RUN_TESTING OFF)
  endif()

  if(TESSERACT_ENABLE_TESTING_ALL)
    set(TESSERACT_ENABLE_TESTING ON)
    set(TESSERACT_ENABLE_CLANG_TIDY ON)
    set(TESSERACT_ENABLE_CODE_COVERAGE ON)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_BENCHMARKING)
    set(TESSERACT_ENABLE_BENCHMARKING OFF)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_RUN_BENCHMARKING)
    set(TESSERACT_ENABLE_RUN_BENCHMARKING OFF)
  endif()

  set(TESSERACT_COMPILE_DEFINITIONS "")
  set(TESSERACT_COMPILE_OPTIONS_PUBLIC "")
  set(TESSERACT_COMPILE_OPTIONS_PRIVATE "")
  if(NOT TESSERACT_WARNINGS_AS_ERRORS)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE
          -Wall
          -Wextra
          -Wconversion
          -Wsign-conversion
          -Wno-sign-compare
          -Wnon-virtual-dtor)
      execute_process(COMMAND uname -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
      if(NOT
         CMAKE_SYSTEM_NAME2
         MATCHES
         "aarch64"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "armv7l"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "unknown")
        set(TESSERACT_COMPILE_OPTIONS_PUBLIC -mno-avx)
      endif()
    elseif(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang.*")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE
          -Wall
          -Wextra
          -Wconversion
          -Wsign-conversion)
      set(TESSERACT_COMPILE_DEFINITIONS "BOOST_STACKTRACE_GNU_SOURCE_NOT_REQUIRED")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      set(TESSERACT_COMPILE_DEFINITIONS "_USE_MATH_DEFINES=ON")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} Unsupported compiler detected.")
    endif()
  else()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE
          -Werror=all
          -Werror=extra
          -Werror=conversion
          -Werror=sign-conversion
          -Wno-sign-compare
          -Werror=non-virtual-dtor)
      execute_process(COMMAND uname -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
      if(NOT
         CMAKE_SYSTEM_NAME2
         MATCHES
         "aarch64"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "armv7l"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "unknown")
        set(TESSERACT_COMPILE_OPTIONS_PUBLIC -mno-avx)
      endif()
    elseif(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang.*")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE
          -Werror=all
          -Werror=extra
          -Werror=conversion
          -Werror=sign-conversion)
      set(TESSERACT_COMPILE_DEFINITIONS "BOOST_STACKTRACE_GNU_SOURCE_NOT_REQUIRED")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      set(TESSERACT_COMPILE_DEFINITIONS "_USE_MATH_DEFINES=ON")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} Unsupported compiler detected.")
    endif()
  endif()

  set(TESSERACT_CXX_VERSION 17)
endmacro()
