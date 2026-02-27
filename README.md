# tesseract_open3d

A multi-sensor simulator built on Open3D that enables efficient simulation of various sensor types in 3D environments. This package provides tools for ray-casting based sensor simulation including spinning LiDAR, solid-state LiDAR, and depth cameras.

## Overview

`tesseract_open3d` is a robotics simulation toolkit that leverages Open3D's RaycastingScene for fast sensor simulation. It allows you to:

- **Manage dynamic 3D scenes** with static and dynamic mesh instances
- **Simulate multiple sensor types** in a unified framework
- **Update poses and geometries** efficiently each timestep
- **Capture sensor readings** from arbitrary viewpoints

## Features

### World Management
- Register both static and dynamic mesh instances
- Update instance poses and geometries per-timestep
- Leverage Open3D's RaycastingScene for efficient ray-casting operations

### Sensor Types
- **Spinning LiDAR**: Multi-beam lidar with configurable rings and resolution
- **Solid-State LiDAR**: Fixed-pattern 3D sensor
- **Depth Camera**: Pinhole camera model for depth image capture

### Simulator
- Orchestrate world updates and sensor captures
- Manage multiple sensors with independent poses
- Single-call per-frame capture of all registered sensors

## Build Status

Platform             | CI Status
---------------------|:---------
Linux                | [![Build Status](https://github.com/tesseract-robotics/tesseract_open3d/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_open3d/actions/workflows/ubuntu.yml)
Lint  (Clang-Format) | [![Build Status](https://github.com/tesseract-robotics/tesseract_open3d/actions/workflows/clang_format.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_open3d/actions/workflows/clang_format.yml)
Lint  (CMake-Format) | [![Build Status](https://github.com/tesseract-robotics/tesseract_open3d/actions/workflows/cmake_format.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_open3d/actions/workflows/cmake_format.yml)
Lint  (Clang-Tidy)   | [![Build Status](https://github.com/tesseract-robotics/tesseract_open3d/actions/workflows/code_quality.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_open3d/actions/workflows/code_quality.yml)

[![Github Issues](https://img.shields.io/github/issues/tesseract-robotics/tesseract_open3d.svg)](http://github.com/tesseract-robotics/tesseract_open3d/issues)

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

[![support level: consortium](https://img.shields.io/badge/support%20level-consortium-brightgreen.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
