#ifndef TESSERACT_OPEN3D_SIMULATOR_H
#define TESSERACT_OPEN3D_SIMULATOR_H

/**
 * @file simulator.h
 * @brief Multi-sensor simulator on top of Open3D RaycastingScene.
 *
 * This header defines a small framework to:
 *  - maintain a world composed of a static mesh plus dynamic instances,
 *  - register multiple sensors (spinning LiDAR, solid-state LiDAR, depth camera),
 *  - update sensor poses and instance poses each timestep,
 *  - rebuild the raycasting scene once per frame and capture all sensors.
 *
 * @note RaycastingScene (Open3D 0.19) does not expose per-geometry transforms.
 *       To animate, we rebuild the scene each frame with pretransformed copies.
 */

#include <tesseract_open3d/world.h>
#include <tesseract_open3d/sensors.h>

// STD
#include <map>
#include <memory>
#include <cstdint>

// Open3D
namespace open3d::core
{
class Device;
}

namespace tesseract_o3d
{
/**
 * @brief Multi-sensor simulator orchestrating world rebuilds and captures.
 *
 * Usage per timestep:
 *   1) Update world instance poses/meshes.
 *   2) Update each sensor pose.
 *   3) Call Tick(t) to rebuild the scene once and capture all sensors.
 */
class Simulator
{
public:
  Simulator(int64_t n_threads = 0, const open3d::core::Device& device = open3d::core::Device("CPU:0"));

  /** @brief Access the world (mutable). */
  World& world();

  /** @brief Access the world (const). */
  const World& world() const;

  /**
   * @brief Adds a sensor to the simulator.
   * @param sensor Owning pointer to a concrete Sensor.
   * @return Integer sensor id for subsequent calls.
   */
  int addSensor(std::unique_ptr<Sensor> sensor);

  /**
   * @brief Sets the pose of a sensor for this frame.
   * @param sensor_id Sensor identifier returned by AddSensor().
   * @param pose Sensor pose (world <- sensor).
   */
  void setSensorPose(int sensor_id, const Eigen::Isometry3d& pose);

  /**
   * @brief Rebuilds the scene and captures all registered sensors.
   * @param t Timestamp (seconds) passed to each sensor capture.
   * @return Map: sensor_id -> SensorOutput.
   */
  std::map<int, SensorOutput> tick(double t);

private:
  World world_;
  int next_sensor_id_ = 1;
  std::map<int, std::unique_ptr<Sensor>> sensors_;
};

}  // namespace tesseract_o3d

#endif  // TESSERACT_OPEN3D_SIMULATOR_H
