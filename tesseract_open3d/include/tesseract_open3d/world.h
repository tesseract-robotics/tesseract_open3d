#ifndef TESSERACT_OPEN3D_WORLD_H
#define TESSERACT_OPEN3D_WORLD_H

// STD
#include <map>
#include <memory>

// Open3d
#include <open3d/t/geometry/RaycastingScene.h>
#include <open3d/t/geometry/TriangleMesh.h>

namespace tesseract_o3d
{
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
  void updateInstanceMesh(int instance_id, std::shared_ptr<open3d::t::geometry::TriangleMesh> base_mesh);

  /**
   * @brief Rebuilds the internal RaycastingScene for the current frame.
   *
   * Applies all current poses to dynamic instances, combines with the static mesh,
   * and constructs a fresh RaycastingScene. Call once per timestep before capturing.
   */
  void commitFrame();

  /** @brief Access the current frame's raycasting scene. */
  const open3d::t::geometry::RaycastingScene& scene() const;

private:
  struct Instance
  {
    std::shared_ptr<open3d::t::geometry::TriangleMesh> base;  // untransformed
    Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  };

  open3d::t::geometry::TriangleMesh static_mesh_;
  std::map<int, Instance> instances_;
  int next_instance_id_ = 1;

  std::unique_ptr<open3d::t::geometry::RaycastingScene> scene_;  // rebuilt in CommitFrame()
};
}  // namespace tesseract_o3d

#endif  // TESSERACT_OPEN3D_WORLD_H
