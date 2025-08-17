#ifndef TESSERACT_OPEN3D_SENSORS_H
#define TESSERACT_OPEN3D_SENSORS_H

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

#include <Eigen/Geometry>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <open3d/t/geometry/TriangleMesh.h>

namespace tesseract_o3d
{

/** @brief Time-stamped rigid pose (world <- sensor). */
struct TimedPose
{
    double t;                 /**< Seconds (absolute/sim time). */
    Eigen::Isometry3d pose;   /**< SE(3): world <- sensor. */
};

/** @brief Intrinsics for a pinhole camera (OpenCV convention). */
struct DepthIntrinsics
{
    int width = 640;   /**< Image width in pixels. */
    int height = 480;  /**< Image height in pixels. */
    float fx = 525.0f; /**< Focal length fx in pixels. */
    float fy = 525.0f; /**< Focal length fy in pixels. */
    float cx = 319.5f; /**< Principal point x in pixels. */
    float cy = 239.5f; /**< Principal point y in pixels. */
};

/** @brief Depth capture result. */
struct DepthFrame
{
    open3d::core::Tensor depth;              /**< (H×W) float meters; inf = no hit. */
    open3d::t::geometry::PointCloud pcd;     /**< Back-projected points (may be empty). */
};

/**
 * @brief World/scene container rebuilt once per frame.
 *
 * A world is the combination of:
 *  - a static environment mesh already expressed in world frame, and
 *  - any number of dynamic instances, each with its own base mesh (local frame)
 *    and a per-frame world pose.
 *
 * Call CommitFrame() once per timestep after updating poses/meshes.
 */
class World
{
public:
    /** @brief Constructs an empty world. */
    World();

    /**
   * @brief Sets/replaces the static environment mesh (in world frame).
   * @param mesh Tensor mesh to copy into the world.
   */
    void setStaticMesh(const open3d::t::geometry::TriangleMesh& mesh);

    /**
   * @brief Registers a dynamic instance.
   * @param base_mesh Mesh in the instance's local frame (shared_ptr kept).
   * @return Integer instance id to address this object later.
   *
   * @warning The mesh pointer is held; do not mutate concurrently.
   */
    int registerInstance(std::shared_ptr<open3d::t::geometry::TriangleMesh> base_mesh);

    /**
   * @brief Updates an instance's world pose to be used on the next CommitFrame().
   * @param instance_id Identifier returned by RegisterInstance().
   * @param pose New pose (world <- instance).
   */
    void updateInstancePose(int instance_id, const Eigen::Isometry3d& pose);

    /**
   * @brief Replaces an instance's base mesh (local frame).
   * @param instance_id Identifier returned by RegisterInstance().
   * @param base_mesh New mesh in instance local frame (shared_ptr kept).
   */
    void updateInstanceMesh(int instance_id,
                            std::shared_ptr<open3d::t::geometry::TriangleMesh> base_mesh);

    /**
   * @brief Rebuilds the internal RaycastingScene for the current frame.
   *
   * Applies all current poses to dynamic instances, combines with the static mesh,
   * and constructs a fresh RaycastingScene. Call once per timestep before capturing.
   */
    void commitFrame();

    /** @brief Access the current frame's raycasting scene. */
    const open3d::t::geometry::RaycastingScene& scene() const { return *scene_; }

private:
    struct Instance
    {
        std::shared_ptr<open3d::t::geometry::TriangleMesh> base;  // untransformed
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    };

    open3d::t::geometry::TriangleMesh static_mesh_;
    std::map<int, Instance> instances_;
    int next_instance_id_ = 1;

    std::unique_ptr<open3d::t::geometry::RaycastingScene> scene_;  // rebuilt in CommitFrame()
};

/** @brief Sensor kind enumeration. */
enum class SensorKind { kSpinningLidar, kSolidStateLidar, kDepthCamera };

/** @brief Configuration for a spinning multi-beam LiDAR. */
struct SpinningLidarConfig {
    int rings = 64;                        /**< Vertical channels. */
    int cols = 2048;                       /**< Horizontal samples per revolution. */
    float min_range = 1.0f;                /**< Minimum valid range (meters). */
    float max_range = 100.0f;              /**< Maximum valid range (meters). */
    float fov_azimuth_deg = 360.0f;        /**< Azimuth FOV in degrees. */
    std::vector<float> vertical_angles_deg;/**< Optional: per-ring elevation (deg). */
    double spin_rate_hz = 10.0;            /**< Revolutions per second. */
    float range_noise_std = 0.01f;         /**< Gaussian sigma on range (m). */
    float azimuth_jitter_std_deg = 0.02f;  /**< Gaussian sigma on azimuth (deg). */
    float dropout_probability = 0.01f;     /**< Per-beam drop probability. */
    uint32_t rng_seed = 42;                /**< RNG seed. */
};

/** @brief Configuration for a solid-state (rectangular FOV) LiDAR. */
struct SolidStateConfig {
    int height = 64;            /**< Rows (vertical samples). */
    int width = 1024;           /**< Cols (horizontal samples). */
    float fov_h_deg = 90.0f;    /**< Horizontal FOV (deg). */
    float fov_v_deg = 30.0f;    /**< Vertical FOV (deg). */
    float min_range = 0.3f;     /**< Minimum range (m). */
    float max_range = 80.0f;    /**< Maximum range (m). */
    double frame_rate_hz = 20.0;/**< Frames per second. */
    float range_noise_std = 0.01f; /**< Gaussian sigma on range (m). */
    float jitter_h_std_deg = 0.0f; /**< Horizontal angle jitter (deg). */
    float jitter_v_std_deg = 0.0f; /**< Vertical angle jitter (deg). */
    float dropout_probability = 0.0f; /**< Per-pixel drop probability. */
    uint32_t rng_seed = 123;     /**< RNG seed. */
};

/** @brief Configuration for a pinhole depth camera. */
struct DepthConfig {
    DepthIntrinsics K;              /**< Pinhole intrinsics. */
    float min_range = 0.2f;         /**< Minimum valid depth (m). */
    float max_range = 20.0f;        /**< Maximum valid depth (m). */
    double frame_rate_hz = 30.0;    /**< Frames per second. */
    float depth_noise_std = 0.0f;   /**< Gaussian sigma on depth (m). */
    float dropout_probability = 0.0f;/**< Per-pixel drop probability. */
    uint32_t rng_seed = 7;          /**< RNG seed. */
};

/**
 * @brief Output from a sensor capture.
 *
 * For LiDARs, `point_cloud` is set. For depth cameras, `depth` is set (and
 * `depth->pcd` contains the back-projected points).
 */
struct SensorOutput {
    std::optional<open3d::t::geometry::PointCloud> point_cloud; /**< LiDAR result. */
    std::optional<DepthFrame> depth;                             /**< Depth result. */
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
class Sensor {
public:
    virtual ~Sensor() = default;

    /** @brief Sets the current sensor pose (world <- sensor). */
    void setPose(const Eigen::Isometry3d& pose) { pose_ = pose; }

    /** @brief Returns the sensor kind. */
    SensorKind kind() const { return kind_; }

    /** @brief Returns the sensor name. */
    const std::string& name() const { return name_; }

    /**
   * @brief Captures a frame at time @p t using the world's current scene.
   * @param world World after CommitFrame().
   * @param t capture timestamp (seconds).
   * @return SensorOutput containing either a point cloud or a depth frame.
   */
    virtual SensorOutput capture(const World& world, double t) = 0;

protected:
    explicit Sensor(std::string name, SensorKind kind)
        : name_(std::move(name)), kind_(kind) {}

    Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();

private:
    std::string name_;
    SensorKind kind_;
};

/** @brief Factory: creates a spinning LiDAR sensor. */
std::unique_ptr<Sensor> makeSpinningLidar(const std::string& name,
                                          const SpinningLidarConfig& cfg);
/** @brief Factory: creates a solid-state LiDAR sensor. */
std::unique_ptr<Sensor> makeSolidStateLidar(const std::string& name,
                                            const SolidStateConfig& cfg,
                                            bool rolling_shutter);
/** @brief Factory: creates a depth camera sensor. */
std::unique_ptr<Sensor> makeDepthCamera(const std::string& name,
                                        const DepthConfig& cfg,
                                        bool keep_organized_points);

/**
 * @brief Multi-sensor simulator orchestrating world rebuilds and captures.
 *
 * Usage per timestep:
 *   1) Update world instance poses/meshes.
 *   2) Update each sensor pose.
 *   3) Call Tick(t) to rebuild the scene once and capture all sensors.
 */
class Simulator {
public:
    /** @brief Access the world (mutable). */
    World& world() { return world_; }
    /** @brief Access the world (const). */
    const World& world() const { return world_; }

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

#endif // TESSERACT_OPEN3D_SENSORS_H
