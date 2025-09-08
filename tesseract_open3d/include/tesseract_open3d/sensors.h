#ifndef TESSERACT_OPEN3D_SENSORS_H
#define TESSERACT_OPEN3D_SENSORS_H

// STD
#include <memory>
#include <vector>
#include <optional>
#include <string>

// Open3d
#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>

namespace tesseract_o3d
{
class World;

/** @brief Sensor kind enumeration. */
enum class SensorType
{
  kSpinningLidar,
  kSolidStateLidar,
  kDepthCamera
};

/** @brief Intrinsics for a pinhole camera (OpenCV convention). */
struct DepthIntrinsics
{
  int width{ 640 };   /**< Image width in pixels. */
  int height{ 480 };  /**< Image height in pixels. */
  float fx{ 525.0F }; /**< Focal length fx in pixels. */
  float fy{ 525.0F }; /**< Focal length fy in pixels. */
  float cx{ 319.5F }; /**< Principal point x in pixels. */
  float cy{ 239.5F }; /**< Principal point y in pixels. */
};

/** @brief Depth capture result. */
struct DepthFrame
{
  open3d::core::Tensor depth;          /**< (H×W) float meters; inf = no hit. */
  open3d::t::geometry::PointCloud pcd; /**< Back-projected points (may be empty). */
};

/** @brief Configuration for a spinning multi-beam LiDAR. */
struct SpinningLidarConfig
{
  int rings{ 64 };                        /**< Vertical channels. */
  int cols{ 2048 };                       /**< Horizontal samples per revolution. */
  float min_range{ 1.0F };                /**< Minimum valid range (meters). */
  float max_range{ 100.0F };              /**< Maximum valid range (meters). */
  float fov_azimuth_deg{ 360.0F };        /**< Azimuth FOV in degrees. */
  std::vector<float> vertical_angles_deg; /**< Optional: per-ring elevation (deg). */
  double spin_rate_hz{ 10.0 };            /**< Revolutions per second. */
  float range_noise_std{ 0.01F };         /**< Gaussian sigma on range (m). */
  float azimuth_jitter_std_deg{ 0.02F };  /**< Gaussian sigma on azimuth (deg). */
  float dropout_probability{ 0.01F };     /**< Per-beam drop probability. */
  std::uint32_t rng_seed{ 42 };           /**< RNG seed. */
};

/** @brief Configuration for a solid-state (rectangular FOV) LiDAR. */
struct SolidStateConfig
{
  int height{ 64 };                  /**< Rows (vertical samples). */
  int width{ 1024 };                 /**< Cols (horizontal samples). */
  float fov_h_deg{ 90.0F };          /**< Horizontal FOV (deg). */
  float fov_v_deg{ 30.0F };          /**< Vertical FOV (deg). */
  float min_range{ 0.3F };           /**< Minimum range (m). */
  float max_range{ 80.0F };          /**< Maximum range (m). */
  double frame_rate_hz{ 20.0 };      /**< Frames per second. */
  float range_noise_std{ 0.01F };    /**< Gaussian sigma on range (m). */
  float jitter_h_std_deg{ 0.0F };    /**< Horizontal angle jitter (deg). */
  float jitter_v_std_deg{ 0.0F };    /**< Vertical angle jitter (deg). */
  float dropout_probability{ 0.0F }; /**< Per-pixel drop probability. */
  std::uint32_t rng_seed{ 123 };     /**< RNG seed. */
};

/** @brief Configuration for a pinhole depth camera. */
struct DepthConfig
{
  DepthIntrinsics intrinsics;        /**< Pinhole intrinsics. */
  float min_range{ 0.2F };           /**< Minimum valid depth (m). */
  float max_range{ 20.0F };          /**< Maximum valid depth (m). */
  double frame_rate_hz{ 30.0 };      /**< Frames per second. */
  float depth_noise_std{ 0.0F };     /**< Gaussian sigma on depth (m). */
  float dropout_probability{ 0.0F }; /**< Per-pixel drop probability. */
  std::uint32_t rng_seed{ 7 };       /**< RNG seed. */
};

/**
 * @brief Output from a sensor capture.
 *
 * For LiDARs, `point_cloud` is set. For depth cameras, `depth` is set (and
 * `depth->pcd` contains the back-projected points).
 */
struct SensorOutput
{
  std::optional<open3d::t::geometry::PointCloud> point_cloud; /**< LiDAR result. */
  std::optional<DepthFrame> depth;                            /**< Depth result. */
};

/**
 * @brief Abstract sensor type; concrete sensors implement Capture().
 *
 * Typical usage:
 * @code
 * auto sensor = MakeSpinningLidar("lidar", cfg);
 * int id = sim.AddSensor(std::move(sensor));
 * sim.SetSensorPose(id, pose);
 * @endcode
 */
class Sensor
{
public:
  virtual ~Sensor();

  /** @brief Sets the current sensor pose (world <- sensor). */
  void setPose(const Eigen::Isometry3d& pose);

  /** @brief Returns the sensor type. */
  SensorType type() const;

  /** @brief Returns the sensor name. */
  const std::string& name() const;

  /**
   * @brief Captures a frame at time @p t using the world's current scene.
   * @param world World after CommitFrame().
   * @param t capture timestamp (seconds).
   * @return SensorOutput containing either a point cloud or a depth frame.
   */
  virtual SensorOutput capture(const World& world, double t) = 0;

protected:
  /**
   * @brief Constructor
   * @param name The name of the sensor
   * @param type The type of sensor
   */
  explicit Sensor(std::string name, SensorType type);

  /** @brief The sensor pose */
  Eigen::Isometry3d pose_{ Eigen::Isometry3d::Identity() };

private:
  /** @brief The sensor name  */
  std::string name_;

  /** @brief The sensor type */
  SensorType type_;
};

/** @brief Factory: creates a spinning LiDAR sensor. */
std::unique_ptr<Sensor> makeSpinningLidar(const std::string& name, const SpinningLidarConfig& cfg);

/** @brief Factory: creates a solid-state LiDAR sensor. */
std::unique_ptr<Sensor> makeSolidStateLidar(const std::string& name, const SolidStateConfig& cfg, bool rolling_shutter);

/** @brief Factory: creates a depth camera sensor. */
std::unique_ptr<Sensor> makeDepthCamera(const std::string& name, const DepthConfig& cfg, bool keep_organized_points);
}  // namespace tesseract_o3d

#endif  // TESSERACT_OPEN3D_SENSORS_H
