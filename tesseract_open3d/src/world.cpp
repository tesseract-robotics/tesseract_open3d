#include <tesseract_open3d/world.h>

// STD
#include <utility>
#include <memory>
#include <cstdint>

// Eigen SDK
#include <Eigen/Geometry>  // IWYU pragma: keep

// Open3d
#include <open3d/core/EigenConverter.h>
#include <open3d/core/Device.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <open3d/t/geometry/TriangleMesh.h>

using open3d::t::geometry::RaycastingScene;
using open3d::t::geometry::TriangleMesh;

namespace tesseract_o3d
{
namespace
{
// Apply a rigid transform to a *copy* of a tensor mesh (in place).
TriangleMesh transformCopy(const TriangleMesh& m, const Eigen::Isometry3d& t)
{
  TriangleMesh out = m;
  out.Transform(open3d::core::eigen_converter::EigenMatrixToTensor(t.matrix()));
  return out;
}

}  // namespace

// NOLINTNEXTLINE(modernize-pass-by-value)
World::Instance::Instance(std::shared_ptr<open3d::t::geometry::TriangleMesh> mesh,
                          const Eigen::Isometry3d& origin)  // NOLINT(modernize-pass-by-value)
  : base(std::move(mesh)), pose(origin)
{
}

World::World(int64_t n_threads, const open3d::core::Device& device)
  : n_threads_(n_threads), device_(device), scene_(std::make_unique<RaycastingScene>(n_threads_, device_))
{
}

int World::registerInstance(std::shared_ptr<TriangleMesh> mesh, const Eigen::Isometry3d& pose)
{
  const int id = next_instance_id_++;
  instances_[id] = Instance{ std::move(mesh), pose };
  return id;
}

void World::updateInstancePose(int instance_id, const Eigen::Isometry3d& pose)
{
  auto it = instances_.find(instance_id);
  if (it != instances_.end())
    it->second.pose = pose;
}

void World::updateInstanceMesh(int instance_id, std::shared_ptr<TriangleMesh> base_mesh)
{
  auto it = instances_.find(instance_id);
  if (it != instances_.end())
    it->second.base = std::move(base_mesh);
}

void World::removeInstance(int instance_id) { instances_.erase(instance_id); }

void World::commitFrame()
{
  // fresh BVH each frame
  scene_ = std::make_unique<RaycastingScene>(n_threads_, device_);

  // Add each dynamic instance after applying its world pose.
  for (const auto& [id, instance] : instances_)
  {
    if (!instance.base || instance.base->IsEmpty())
      continue;

    // Transform a copy into world frame, then add.
    const TriangleMesh m = transformCopy(*instance.base, instance.pose);
    (void)scene_->AddTriangles(m);
  }
}

const open3d::t::geometry::RaycastingScene& World::scene() const { return *scene_; }

}  // namespace tesseract_o3d
