#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/t/geometry/PointCloud.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/utility/Logging.h>

#include <tesseract_open3d/simulator.h>

namespace {

// Save helper: drop NaNs/Infs and skip empty frames.
void savePCDOrSkip(const std::string& path,
                   const open3d::t::geometry::PointCloud& tpcd) {
    auto legacy = tpcd.ToLegacy();
    legacy.RemoveNonFinitePoints();
    if (legacy.points_.empty()) {
        open3d::utility::LogWarning("Skip {}: no valid points.", path);
        return;
    }
    if (!open3d::io::WritePointCloud(path, legacy, /*write_ascii=*/true)) {
        open3d::utility::LogWarning("WritePointCloud failed for {}", path);
    }
}

int countPoints(const open3d::t::geometry::PointCloud& pc) {
    return pc.IsEmpty() ? 0 : static_cast<int>(pc.GetPointPositions().GetShape(0));
}

// Make a pose that “looks at” a target with +Z forward (camera-like).
// This matches the updated LiDAR convention (boresight +Z) and the depth camera.
Eigen::Isometry3d makeLookAtZForward(const Eigen::Vector3d& pos,
                                     const Eigen::Vector3d& target,
                                     const Eigen::Vector3d& up_world) {
    Eigen::Vector3d f = (target - pos).normalized();  // forward (+Z)
    Eigen::Vector3d x = f.cross(up_world).normalized();  // right (+X)
    Eigen::Vector3d y = x.cross(f).normalized();         // down-ish (+Y)
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear().col(0) = x;
    T.linear().col(1) = y;
    T.linear().col(2) = f;
    T.translation() = pos;
    return T;
}

}  // namespace

int main() {
    using open3d::t::geometry::TriangleMesh;

    // --- Build a single 1 m box centered at (0,0,0.5) -------------------------
    auto box_legacy = open3d::geometry::TriangleMesh::CreateBox(1.0, 1.0, 1.0);
    // Base at z=0, center at z=0.5:
    box_legacy->Translate(Eigen::Vector3d(-0.5, -0.5, 0.0));
    box_legacy->Translate(Eigen::Vector3d(0.0, 0.0, 0.5));
    TriangleMesh box = TriangleMesh::FromLegacy(*box_legacy);

    // --- Simulator/world -------------------------------------------------------
    tesseract_o3d::Simulator sim;
    sim.world().setStaticMesh(box);  // ONLY object in the scene

    // --- Add sensors (all +Z-forward) -----------------------------------------
    // Spinning LiDAR
    tesseract_o3d::SpinningLidarConfig spin_cfg;
    spin_cfg.rings = 64;
    spin_cfg.cols = 2048;
    spin_cfg.min_range = 0.05f;
    spin_cfg.max_range = 200.0f;
    spin_cfg.fov_azimuth_deg = 360.0f;
    // Disable noise
    spin_cfg.range_noise_std = 0.0f;
    spin_cfg.azimuth_jitter_std_deg = 0.0f;
    spin_cfg.dropout_probability = 0.0f;
    // With +Z forward, keep elevations around 0° (slightly up/down).
    spin_cfg.vertical_angles_deg.resize(spin_cfg.rings);
    for (int i = 0; i < spin_cfg.rings; ++i) {
        float u = (i + 0.5f) / static_cast<float>(spin_cfg.rings);
        spin_cfg.vertical_angles_deg[i] = -15.0f + u * 30.0f;  // [-15°, +15°]
    }
    const int spin_id =
        sim.addSensor(tesseract_o3d::makeSpinningLidar("spin", spin_cfg));

    // Solid-state LiDAR
    tesseract_o3d::SolidStateConfig ss_cfg;
    ss_cfg.height = 64;
    ss_cfg.width = 1024;
    ss_cfg.fov_h_deg = 90.0f;
    ss_cfg.fov_v_deg = 60.0f;  // +/-30°
    ss_cfg.min_range = 0.05f;
    ss_cfg.max_range = 200.0f;
    // Disable noise
    ss_cfg.range_noise_std = 0.0f;
    ss_cfg.jitter_h_std_deg = 0.0f;
    ss_cfg.jitter_v_std_deg = 0.0f;
    ss_cfg.dropout_probability = 0.0f;
    const int ss_id =
        sim.addSensor(tesseract_o3d::makeSolidStateLidar("solid", ss_cfg,
                                                       /*rolling_shutter=*/true));

    // Depth camera
    tesseract_o3d::DepthConfig dcfg;
    dcfg.K = {640, 480, 525.f, 525.f, 319.5f, 239.5f};
    dcfg.min_range = 0.05f;
    dcfg.max_range = 50.0f;
    // Disable noise
    dcfg.depth_noise_std = 0.0f;
    dcfg.dropout_probability = 0.0f;
    const int cam_id =
        sim.addSensor(tesseract_o3d::makeDepthCamera("cam", dcfg,
                                                   /*keep_organized_points=*/false));

    // --- Pose all sensors to look directly at the box center -------------------
    const Eigen::Vector3d box_center(0.0, 0.0, 0.5);
    const Eigen::Vector3d up(0.0, 0.0, 1.0);

    sim.setSensorPose(spin_id,  makeLookAtZForward(/*pos=*/{-3.0,  0.0, 1.2}, box_center, up));
    sim.setSensorPose(ss_id,    makeLookAtZForward(/*pos=*/{ 0.0, -3.0, 1.0}, box_center, up));
    sim.setSensorPose(cam_id,   makeLookAtZForward(/*pos=*/{ 3.0,  0.0, 1.0}, box_center, up));

    // --- Simulate a single frame ----------------------------------------------
    auto out = sim.tick(/*t=*/0.0);

    // --- Report counts and save ------------------------------------------------
    if (out[spin_id].point_cloud) {
        open3d::utility::LogInfo("Spinning LiDAR points: {}", countPoints(*out[spin_id].point_cloud));
        savePCDOrSkip("/tmp/box_spin.pcd", *out[spin_id].point_cloud);
    } else {
        open3d::utility::LogWarning("Spinning LiDAR produced no cloud.");
    }

    if (out[ss_id].point_cloud) {
        open3d::utility::LogInfo("Solid-state LiDAR points: {}", countPoints(*out[ss_id].point_cloud));
        savePCDOrSkip("/tmp/box_solid.pcd", *out[ss_id].point_cloud);
    } else {
        open3d::utility::LogWarning("Solid-state LiDAR produced no cloud.");
    }

    if (out[cam_id].depth) {
        open3d::utility::LogInfo("Depth camera point count: {}", countPoints(out[cam_id].depth->pcd));
        savePCDOrSkip("/tmp/box_depth_points.pcd", out[cam_id].depth->pcd);
    } else {
        open3d::utility::LogWarning("Depth camera produced no frame.");
    }

    open3d::utility::LogInfo("Box-only demo complete.");
    return 0;
}
