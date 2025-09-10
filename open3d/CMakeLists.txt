cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
project(Open3D VERSION 0.19.0 LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

include(GNUInstallDirs)
include(ExternalProject)

set(OPEN3D_URL
    "https://github.com/isl-org/Open3D/releases/download/v0.19.0/open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz"
    CACHE STRING "Path or URL to Open3D prebuilt C++ archive (.tar.xz)")
set(OPEN3D_URL_HASH "" CACHE STRING "Optional SHA256 of the archive (SHA256=<hash>)")

# Extract into the build tree; we'll install from there.
set(OPEN3D_STAGE "${CMAKE_BINARY_DIR}/open3d_stage")

ExternalProject_Add(
  ${PROJECT_NAME}
  URL "${OPEN3D_URL}"
  URL_HASH "${OPEN3D_URL_HASH}"
  DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/_downloads"
  SOURCE_DIR "${OPEN3D_STAGE}"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  TLS_VERIFY ON)

# ---- Install headers, libs, and the Open3D CMake package -------------------
# The prebuilt archive layout is: include/, lib/libOpen3D.so, lib/cmake/Open3D/... Installing lib/ copies both the .so
# and the Config.cmake into <prefix>/lib and <prefix>/lib/cmake/Open3D.
install(DIRECTORY "${OPEN3D_STAGE}/include/open3d" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} USE_SOURCE_PERMISSIONS)
install(DIRECTORY "${OPEN3D_STAGE}/lib/" DESTINATION ${CMAKE_INSTALL_LIBDIR} USE_SOURCE_PERMISSIONS)
install(DIRECTORY "${OPEN3D_STAGE}/share/" DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
