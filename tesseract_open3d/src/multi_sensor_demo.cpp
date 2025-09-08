#include <tesseract_open3d/simulator.h>
#include <tesseract_open3d/sensors.h>

// STD
#include <string>

// Open3d
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/t/geometry/PointCloud.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/utility/Logging.h>

namespace
{
// Save helper: drop NaNs/Infs and skip empty frames.
void savePCDOrSkip(const std::string& path, const open3d::t::geometry::PointCloud& tpcd)
{
  auto legacy = tpcd.ToLegacy();
  legacy.RemoveNonFinitePoints();
  if (legacy.points_.empty())
  {
    open3d::utility::LogWarning("Skip {}: no valid points.", path);
    return;
  }
  if (!open3d::io::WritePointCloud(path, legacy, /*write_ascii=*/true))
  {
    open3d::utility::LogWarning("WritePointCloud failed for {}", path);
  }
}

int countPoints(const open3d::t::geometry::PointCloud& pc)
{
  return pc.IsEmpty() ? 0 : static_cast<int>(pc.GetPointPositions().GetShape(0));
}

// Make a pose that “looks at” a target with +Z forward (camera-like).
// This matches the updated LiDAR convention (boresight +Z) and the depth camera.
Eigen::Isometry3d makeLookAtZForward(const Eigen::Vector3d& pos,
                                     const Eigen::Vector3d& target,
                                     const Eigen::Vector3d& up_world)
{
  const Eigen::Vector3d f = (target - pos).normalized();     // forward (+Z)
  const Eigen::Vector3d x = f.cross(up_world).normalized();  // right (+X)
  const Eigen::Vector3d y = x.cross(f).normalized();         // down-ish (+Y)
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear().col(0) = x;
  t.linear().col(1) = y;
  t.linear().col(2) = f;
  t.translation() = pos;
  return t;
}

}  // namespace

int main()
{
  using open3d::t::geometry::TriangleMesh;

  // --- Build a single 1 m box centered at (0,0,0.5) -------------------------
  auto box_legacy = open3d::geometry::TriangleMesh::CreateBox(1.0, 1.0, 1.0);
  // Base at z=0, center at z=0.5:
  box_legacy->Translate(Eigen::Vector3d(-0.5, -0.5, 0.0));
  box_legacy->Translate(Eigen::Vector3d(0.0, 0.0, 0.5));
  const TriangleMesh box = TriangleMesh::FromLegacy(*box_legacy);

  // --- Simulator/world -------------------------------------------------------
  tesseract_o3d::Simulator sim;
  sim.world().setStaticMesh(box);  // ONLY object in the scene

  // --- Add sensors (all +Z-forward) -----------------------------------------
  // Spinning LiDAR
  tesseract_o3d::SpinningLidarConfig spin_cfg;
  spin_cfg.rings = 64;
  spin_cfg.cols = 2048;
  spin_cfg.min_range = 0.05F;
  spin_cfg.max_range = 200.0F;
  spin_cfg.fov_azimuth_deg = 360.0F;
  // Disable noise
  spin_cfg.range_noise_std = 0.0F;
  spin_cfg.azimuth_jitter_std_deg = 0.0F;
  spin_cfg.dropout_probability = 0.0F;
  // With +Z forward, keep elevations around 0° (slightly up/down).
  spin_cfg.vertical_angles_deg.resize(spin_cfg.rings);
  for (int i = 0; i < spin_cfg.rings; ++i)
  {
    const float u = (static_cast<float>(i) + 0.5F) / static_cast<float>(spin_cfg.rings);
    spin_cfg.vertical_angles_deg[i] = -15.0F + u * 30.0F;  // [-15°, +15°]
  }
  const int spin_id = sim.addSensor(tesseract_o3d::makeSpinningLidar("spin", spin_cfg));

  // Solid-state LiDAR
  tesseract_o3d::SolidStateConfig ss_cfg;
  ss_cfg.height = 64;
  ss_cfg.width = 1024;
  ss_cfg.fov_h_deg = 90.0F;
  ss_cfg.fov_v_deg = 60.0F;  // +/-30°
  ss_cfg.min_range = 0.05F;
  ss_cfg.max_range = 200.0F;
  // Disable noise
  ss_cfg.range_noise_std = 0.0F;
  ss_cfg.jitter_h_std_deg = 0.0F;
  ss_cfg.jitter_v_std_deg = 0.0F;
  ss_cfg.dropout_probability = 0.0F;
  const int ss_id = sim.addSensor(tesseract_o3d::makeSolidStateLidar("solid",
                                                                     ss_cfg,
                                                                     /*rolling_shutter=*/true));

  // Depth camera
  tesseract_o3d::DepthConfig dcfg;
  dcfg.intrinsics = { 640, 480, 525.F, 525.F, 319.5F, 239.5F };
  dcfg.min_range = 0.05F;
  dcfg.max_range = 50.0F;
  // Disable noise
  dcfg.depth_noise_std = 0.0F;
  dcfg.dropout_probability = 0.0F;
  const int cam_id = sim.addSensor(tesseract_o3d::makeDepthCamera("cam",
                                                                  dcfg,
                                                                  /*keep_organized_points=*/false));

  // --- Pose all sensors to look directly at the box center -------------------
  const Eigen::Vector3d box_center(0.0, 0.0, 0.5);
  const Eigen::Vector3d up(0.0, 0.0, 1.0);

  sim.setSensorPose(spin_id, makeLookAtZForward(/*pos=*/{ -3.0, 0.0, 1.2 }, box_center, up));
  sim.setSensorPose(ss_id, makeLookAtZForward(/*pos=*/{ 0.0, -3.0, 1.0 }, box_center, up));
  sim.setSensorPose(cam_id, makeLookAtZForward(/*pos=*/{ 3.0, 0.0, 1.0 }, box_center, up));

  // --- Simulate a single frame ----------------------------------------------
  auto out = sim.tick(/*t=*/0.0);

  // --- Report counts and save ------------------------------------------------
  if (out[spin_id].point_cloud.has_value())
  {
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
    const open3d::t::geometry::PointCloud& out_pcd = out[spin_id].point_cloud.value();
    open3d::utility::LogInfo("Spinning LiDAR points: {}", countPoints(out_pcd));
    savePCDOrSkip("/tmp/box_spin.pcd", out_pcd);
  }
  else
  {
    open3d::utility::LogWarning("Spinning LiDAR produced no cloud.");
  }

  if (out[ss_id].point_cloud.has_value())
  {
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
    const open3d::t::geometry::PointCloud& out_pcd = out[ss_id].point_cloud.value();
    open3d::utility::LogInfo("Solid-state LiDAR points: {}", countPoints(out_pcd));
    savePCDOrSkip("/tmp/box_solid.pcd", out_pcd);
  }
  else
  {
    open3d::utility::LogWarning("Solid-state LiDAR produced no cloud.");
  }

  if (out[cam_id].depth.has_value())
  {
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
    const tesseract_o3d::DepthFrame& out_depth = out[cam_id].depth.value();
    open3d::utility::LogInfo("Depth camera point count: {}", countPoints(out_depth.pcd));
    savePCDOrSkip("/tmp/box_depth_points.pcd", out_depth.pcd);
  }
  else
  {
    open3d::utility::LogWarning("Depth camera produced no frame.");
  }

  open3d::utility::LogInfo("Box-only demo complete.");
  return 0;
}
