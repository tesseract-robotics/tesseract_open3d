#include <tesseract_open3d/world.h>

// STD
#include <utility>
#include <memory>

// Eigen SDK
#include <Eigen/Geometry>  // IWYU pragma: keep

// Open3d
#include <open3d/core/EigenConverter.h>
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

World::World() : scene_(std::make_unique<RaycastingScene>()) {}

void World::setStaticMesh(const TriangleMesh& mesh) { static_mesh_ = mesh; }

int World::registerInstance(std::shared_ptr<TriangleMesh> base_mesh)
{
  const int id = next_instance_id_++;
  instances_[id].base = std::move(base_mesh);
  instances_[id].pose = Eigen::Isometry3d::Identity();
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

void World::commitFrame()
{
  // fresh BVH each frame
  scene_ = std::make_unique<RaycastingScene>();

  // Add the static environment (already in world frame).
  if (!static_mesh_.IsEmpty())
  {
    (void)scene_->AddTriangles(static_mesh_);
  }

  // Add each dynamic instance after applying its world pose.
  for (const auto& [id, inst] : instances_)
  {
    if (!inst.base || inst.base->IsEmpty())
      continue;

    // Transform a copy into world frame, then add.
    const TriangleMesh m = transformCopy(*inst.base, inst.pose);
    (void)scene_->AddTriangles(m);
  }
}

const open3d::t::geometry::RaycastingScene& World::scene() const { return *scene_; }

}  // namespace tesseract_o3d
