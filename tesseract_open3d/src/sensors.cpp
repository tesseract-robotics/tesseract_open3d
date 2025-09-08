#include <tesseract_open3d/sensors.h>
#include <tesseract_open3d/world.h>

// STD
#include <string>
#include <limits>
#include <random>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <algorithm>
#include <utility>
#include <memory>
#include <vector>

// Eigen
#include <Eigen/Geometry>  // IWYU pragma: keep

// Open3d
#include <open3d/core/Dtype.h>
#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>
#include <open3d/t/geometry/RaycastingScene.h>

using open3d::core::Float32;
using open3d::core::Int32;
using open3d::core::Tensor;
using open3d::t::geometry::PointCloud;
using open3d::t::geometry::RaycastingScene;

constexpr double kPI = M_PI;  // NOLINT

namespace tesseract_o3d
{
// ----- Sensor ---------------------------------------------------------------
Sensor::Sensor(std::string name, SensorType type) : name_(std::move(name)), type_(type) {}

Sensor::~Sensor() = default;

void Sensor::setPose(const Eigen::Isometry3d& pose) { pose_ = pose; }

SensorType Sensor::type() const { return type_; }

const std::string& Sensor::name() const { return name_; }

//
// NOTE: In this version, both LiDARs use **+Z forward** boresight.
// Direction in sensor frame for (azimuth=az, elevation=e):
//   d_s = [ sin(az),  sin(e)*cos(az),  cos(e)*cos(az) ]
// At az=0, e=0 -> [0,0,1] (+Z).
//
// Azimuth increases to the "right" (around +Y), elevation positive "up" (around -X).
// The exact handedness is consistent with the formula above.
//

// ----- Spinning LiDAR --------------------------------------------------------
class SpinningLidar final : public Sensor
{
public:
  explicit SpinningLidar(const std::string& name, const SpinningLidarConfig& cfg)
    : Sensor(name, SensorType::kSpinningLidar)
    , cfg_(cfg)
    , rng_(cfg.rng_seed)
    , n_range_(0.0F, cfg.range_noise_std)
    , n_az_(0.0F, cfg.azimuth_jitter_std_deg * static_cast<float>(kPI / 180.0))
    , drop_(cfg.dropout_probability)
  {
    if (cfg_.vertical_angles_deg.empty())
    {
      // With +Z forward, a narrow band around 0° is a sensible default.
      cfg_.vertical_angles_deg.resize(std::max(1, cfg_.rings));
      const float vmin = -15.0F;
      const float vmax = +15.0F;
      for (int i = 0; i < cfg_.rings; ++i)
      {
        const float u = (static_cast<float>(i) + 0.5F) / static_cast<float>(cfg_.rings);
        cfg_.vertical_angles_deg[i] = vmin + u * (vmax - vmin);
      }
    }
  }

  SensorOutput capture(const World& world, double t) override
  {
    const int rings = cfg_.rings;
    const int cols = cfg_.cols;
    const int count = rings * cols;
    const double t_rev = 1.0 / std::max(1e-9, cfg_.spin_rate_hz);
    const float az_span = std::clamp(cfg_.fov_azimuth_deg, 0.0F, 360.0F) * static_cast<float>(kPI / 180.0);

    Tensor rays({ count, 6 }, Float32);
    auto* r = rays.GetDataPtr<float>();

    std::vector<int32_t> ring(count);
    std::vector<float> az(count);
    std::vector<float> t_abs(count);
    std::vector<float> elev(rings);
    for (int i = 0; i < rings; ++i)
      elev[i] = cfg_.vertical_angles_deg[i] * static_cast<float>(kPI / 180.0);

    int k = 0;
    for (int vi = 0; vi < rings; ++vi)
    {
      const float e = elev[vi];
      const float ce = std::cos(e);
      const float se = std::sin(e);
      for (int hi = 0; hi < cols; ++hi, ++k)
      {
        const float u = (static_cast<float>(hi) + 0.5F) / static_cast<float>(cols);
        const float az_i = u * az_span + n_az_(rng_);
        const float ca = std::cos(az_i);
        const float sa = std::sin(az_i);
        const double dt = static_cast<double>(u) * t_rev;
        const double t_ray = t + dt;

        const Eigen::Vector3d o = pose_.translation();
        // +Z forward convention:
        const Eigen::Vector3d d_s(sa, se * ca, ce * ca);
        const Eigen::Vector3d d_w = pose_.linear() * d_s;

        r[6 * k + 0] = static_cast<float>(o.x());
        r[6 * k + 1] = static_cast<float>(o.y());
        r[6 * k + 2] = static_cast<float>(o.z());
        r[6 * k + 3] = static_cast<float>(d_w.x());
        r[6 * k + 4] = static_cast<float>(d_w.y());
        r[6 * k + 5] = static_cast<float>(d_w.z());

        ring[k] = vi;
        az[k] = az_i;
        t_abs[k] = static_cast<float>(t_ray);
      }
    }

    const auto out = world.scene().CastRays(rays);
    const auto* t_hit = out.at("t_hit").GetDataPtr<float>();

    std::vector<float> xyz;
    xyz.reserve(static_cast<std::size_t>(count) * 3);

    std::vector<float> rng;
    rng.reserve(count);

    std::vector<int32_t> ring_ok;
    ring_ok.reserve(count);

    std::vector<float> az_ok;
    az_ok.reserve(count);

    std::vector<float> t_ok;
    t_ok.reserve(count);

    for (int i = 0; i < count; ++i)
    {
      float th = t_hit[i];
      if (!std::isfinite(th) || th < cfg_.min_range || th > cfg_.max_range)
        continue;
      if (cfg_.dropout_probability > 0.0F && drop_(rng_))
        continue;

      th = std::max(cfg_.min_range, th + n_range_(rng_));

      const float* ray = r + static_cast<std::ptrdiff_t>(6) * i;
      const Eigen::Vector3f o(ray[0], ray[1], ray[2]);
      const Eigen::Vector3f d(ray[3], ray[4], ray[5]);
      const Eigen::Vector3f p = o + th * d;

      xyz.insert(xyz.end(), { p.x(), p.y(), p.z() });
      rng.push_back(th);
      ring_ok.push_back(ring[i]);
      az_ok.push_back(az[i]);
      t_ok.push_back(t_abs[i]);
    }

    SensorOutput so;
    if (!xyz.empty())
    {
      PointCloud pc;
      const int m = static_cast<int>(xyz.size() / 3);
      Tensor pos({ m, 3 }, Float32);
      std::memcpy(pos.GetDataPtr<float>(), xyz.data(), xyz.size() * sizeof(float));
      pc.SetPointPositions(pos);
      pc.SetPointAttr("ranges", Tensor(rng, { m, 1 }, Float32));
      pc.SetPointAttr("ring", Tensor(ring_ok, { m, 1 }, Int32));
      pc.SetPointAttr("azimuth", Tensor(az_ok, { m, 1 }, Float32));
      pc.SetPointAttr("time", Tensor(t_ok, { m, 1 }, Float32));
      so.point_cloud = std::move(pc);
    }
    return so;
  }

private:
  SpinningLidarConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<float> n_range_;
  std::normal_distribution<float> n_az_;
  std::bernoulli_distribution drop_;
};

// ----- Solid-state LiDAR -----------------------------------------------------
class SolidStateLidar final : public Sensor
{
public:
  SolidStateLidar(const std::string& name, const SolidStateConfig& cfg, bool rolling)
    : Sensor(name, SensorType::kSolidStateLidar)
    , cfg_(cfg)
    , rolling_(rolling)
    , rng_(cfg.rng_seed)
    , n_range_(0.0F, cfg.range_noise_std)
    , n_jh_(0.0F, cfg.jitter_h_std_deg * static_cast<float>(kPI / 180.0))
    , n_jv_(0.0F, cfg.jitter_v_std_deg * static_cast<float>(kPI / 180.0))
    , drop_(cfg.dropout_probability)
  {
  }

  SensorOutput capture(const World& world, double t) override
  {
    const int height = cfg_.height;
    const int width = cfg_.width;
    const int count = height * width;
    const double t_frame = 1.0 / std::max(1e-9, cfg_.frame_rate_hz);
    const float fov_h = std::clamp(cfg_.fov_h_deg, 0.0F, 360.0F) * static_cast<float>(kPI / 180.0);
    const float fov_v = std::clamp(cfg_.fov_v_deg, 0.0F, 180.0F) * static_cast<float>(kPI / 180.0);

    Tensor rays({ count, 6 }, Float32);
    auto* r = rays.GetDataPtr<float>();
    std::vector<int32_t> row(count);
    std::vector<int32_t> col(count);
    std::vector<float> az(count);
    std::vector<float> el(count);
    std::vector<float> t_abs(count);

    int k = 0;
    for (int i = 0; i < height; ++i)
    {
      const float v = (static_cast<float>(i) + 0.5F) / static_cast<float>(height);
      const float el_i = (v - 0.5F) * fov_v + n_jv_(rng_);
      const float ce = std::cos(el_i);
      const float se = std::sin(el_i);

      for (int j = 0; j < width; ++j, ++k)
      {
        const float u = (static_cast<float>(j) + 0.5F) / static_cast<float>(width);
        const float az_j = (u - 0.5F) * fov_h + n_jh_(rng_);
        const float ca = std::cos(az_j);
        const float sa = std::sin(az_j);
        const double dt = rolling_ ? (u * t_frame) : 0.0;
        const double t_ray = t + dt;

        const Eigen::Vector3d o = pose_.translation();
        // +Z forward convention:
        const Eigen::Vector3d d_s(sa, se * ca, ce * ca);
        const Eigen::Vector3d d_w = pose_.linear() * d_s;

        r[6 * k + 0] = static_cast<float>(o.x());
        r[6 * k + 1] = static_cast<float>(o.y());
        r[6 * k + 2] = static_cast<float>(o.z());
        r[6 * k + 3] = static_cast<float>(d_w.x());
        r[6 * k + 4] = static_cast<float>(d_w.y());
        r[6 * k + 5] = static_cast<float>(d_w.z());

        row[k] = i;
        col[k] = j;
        az[k] = az_j;
        el[k] = el_i;
        t_abs[k] = static_cast<float>(t_ray);
      }
    }

    const auto out = world.scene().CastRays(rays);
    const auto* t_hit = out.at("t_hit").GetDataPtr<float>();

    std::vector<float> xyz;
    xyz.reserve(static_cast<std::size_t>(count) * 3);
    std::vector<float> rng;
    rng.reserve(count);
    std::vector<std::int32_t> row_ok;
    row_ok.reserve(count);
    std::vector<std::int32_t> col_ok;
    col_ok.reserve(count);
    std::vector<float> az_ok;
    az_ok.reserve(count);
    std::vector<float> el_ok;
    el_ok.reserve(count);
    std::vector<float> t_ok;
    t_ok.reserve(count);

    for (int i = 0; i < count; ++i)
    {
      float th = t_hit[i];
      if (!std::isfinite(th) || th < cfg_.min_range || th > cfg_.max_range)
        continue;
      if (cfg_.dropout_probability > 0.0F && drop_(rng_))
        continue;

      th = std::max(cfg_.min_range, th + n_range_(rng_));
      const float* ray = r + static_cast<std::ptrdiff_t>(6) * i;
      const Eigen::Vector3f o(ray[0], ray[1], ray[2]);
      const Eigen::Vector3f d(ray[3], ray[4], ray[5]);
      const Eigen::Vector3f p = o + th * d;

      xyz.insert(xyz.end(), { p.x(), p.y(), p.z() });
      rng.push_back(th);
      row_ok.push_back(row[i]);
      col_ok.push_back(col[i]);
      az_ok.push_back(az[i]);
      el_ok.push_back(el[i]);
      t_ok.push_back(t_abs[i]);
    }

    SensorOutput so;
    if (!xyz.empty())
    {
      PointCloud pc;
      const int m = static_cast<int>(xyz.size() / 3);
      Tensor pos({ m, 3 }, Float32);
      std::memcpy(pos.GetDataPtr<float>(), xyz.data(), xyz.size() * sizeof(float));
      pc.SetPointPositions(pos);
      pc.SetPointAttr("ranges", Tensor(rng, { m, 1 }, Float32));
      pc.SetPointAttr("row", Tensor(row_ok, { m, 1 }, Int32));
      pc.SetPointAttr("col", Tensor(col_ok, { m, 1 }, Int32));
      pc.SetPointAttr("azimuth", Tensor(az_ok, { m, 1 }, Float32));
      pc.SetPointAttr("elevation", Tensor(el_ok, { m, 1 }, Float32));
      pc.SetPointAttr("time", Tensor(t_ok, { m, 1 }, Float32));
      so.point_cloud = std::move(pc);
    }
    return so;
  }

private:
  SolidStateConfig cfg_;
  bool rolling_ = true;
  std::mt19937 rng_;
  std::normal_distribution<float> n_range_, n_jh_, n_jv_;
  std::bernoulli_distribution drop_;
};

// ----- Depth camera ----------------------------------------------------------
class DepthCamera final : public Sensor
{
public:
  DepthCamera(const std::string& name, const DepthConfig& cfg, bool keep_org)
    : Sensor(name, SensorType::kDepthCamera), cfg_(cfg), keep_org_(keep_org)
  {
  }

  SensorOutput capture(const World& world, double /*t*/) override
  {
    const int height = cfg_.intrinsics.height;
    const int width = cfg_.intrinsics.width;

    // Compute horizontal FOV (deg) from fx, width
    const double fov_x_deg = 2.0 * std::atan(static_cast<double>(width) / (2.0 * cfg_.intrinsics.fx)) * 180.0 / kPI;

    // Camera position/orientation from pose_
    const Eigen::Vector3f eye = pose_.translation().cast<float>();
    const Eigen::Vector3f fwd = (pose_.linear() * Eigen::Vector3d::UnitZ()).cast<float>();  // +Z forward
    const Eigen::Vector3f up = (pose_.linear() * Eigen::Vector3d::UnitY()).cast<float>();   // +Y down in our basis
    const Eigen::Vector3f center = eye + fwd;

    const Tensor center_t(std::vector<float>{ center.x(), center.y(), center.z() }, { 3 }, Float32);
    const Tensor eye_t(std::vector<float>{ eye.x(), eye.y(), eye.z() }, { 3 }, Float32);
    const Tensor up_t(std::vector<float>{ up.x(), up.y(), up.z() }, { 3 }, Float32);

    const Tensor rays = RaycastingScene::CreateRaysPinhole(
        /*fov_deg=*/fov_x_deg, center_t, eye_t, up_t, cfg_.intrinsics.width, cfg_.intrinsics.height);

    const auto out = world.scene().CastRays(rays);
    const auto* t_hit = out.at("t_hit").GetDataPtr<float>();

    // Depth (meters) with noise/dropouts.
    Tensor depth({ height, width }, Float32);
    auto* z = depth.GetDataPtr<float>();
    std::mt19937 rng(cfg_.rng_seed + 12345);
    std::normal_distribution<float> nd(0.0F, cfg_.depth_noise_std);
    std::bernoulli_distribution drop(cfg_.dropout_probability);

    for (int i = 0; i < height * width; ++i)
    {
      const float d = t_hit[i];
      if (!std::isfinite(d))
      {
        z[i] = std::numeric_limits<float>::infinity();
      }
      else if (d < cfg_.min_range || d > cfg_.max_range)
      {
        z[i] = std::numeric_limits<float>::infinity();
      }
      else if (drop(rng))
      {
        z[i] = std::numeric_limits<float>::infinity();
      }
      else
      {
        z[i] = std::max(cfg_.min_range, d + nd(rng));
      }
    }

    // Back-project to points (organized optional).
    std::vector<float> xyz;
    if (keep_org_)
    {
      xyz.resize(static_cast<long>(height * width) * 3, std::numeric_limits<float>::quiet_NaN());
    }
    else
    {
      xyz.reserve(static_cast<long>(height * width) * 3);
    }

    const float fx = cfg_.intrinsics.fx;
    const float fy = cfg_.intrinsics.fy;
    const float cx = cfg_.intrinsics.cx;
    const float cy = cfg_.intrinsics.cy;
    const Eigen::Matrix3f r = pose_.linear().cast<float>();
    const Eigen::Vector3f ow = pose_.translation().cast<float>();

    auto write_pt = [&](int idx, const Eigen::Vector3f& pw) {
      if (keep_org_)
      {
        xyz[idx * 3 + 0] = pw.x();
        xyz[idx * 3 + 1] = pw.y();
        xyz[idx * 3 + 2] = pw.z();
      }
      else
      {
        xyz.insert(xyz.end(), { pw.x(), pw.y(), pw.z() });
      }
    };

    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        const int idx = y * width + x;
        const float d = z[idx];
        if (!std::isfinite(d))
          continue;
        const float px = (static_cast<float>(x) - cx) / fx;
        const float py = (static_cast<float>(y) - cy) / fy;
        const Eigen::Vector3f pc(px * d, py * d, d);  // camera frame (+Z forward)
        const Eigen::Vector3f pw = ow + r * pc;       // world frame
        write_pt(idx, pw);
      }
    }

    PointCloud pc;
    if (!xyz.empty())
    {
      const int m = keep_org_ ? (height * width) : static_cast<int>(xyz.size() / 3);
      Tensor pos({ m, 3 }, Float32);
      std::memcpy(pos.GetDataPtr<float>(), xyz.data(), static_cast<std::size_t>(m) * 3 * sizeof(float));
      pc.SetPointPositions(pos);
    }

    SensorOutput so;
    so.depth = DepthFrame{ depth, pc };
    return so;
  }

private:
  DepthConfig cfg_;
  bool keep_org_ = false;
};

// ----- Sensor Factories --------------------------------------------------------

std::unique_ptr<Sensor> makeSpinningLidar(const std::string& name, const SpinningLidarConfig& cfg)
{
  return std::make_unique<SpinningLidar>(name, cfg);
}

std::unique_ptr<Sensor> makeSolidStateLidar(const std::string& name, const SolidStateConfig& cfg, bool rolling_shutter)
{
  return std::make_unique<SolidStateLidar>(name, cfg, rolling_shutter);
}

std::unique_ptr<Sensor> makeDepthCamera(const std::string& name, const DepthConfig& cfg, bool keep_organized_points)
{
  return std::make_unique<DepthCamera>(name, cfg, keep_organized_points);
}

}  // namespace tesseract_o3d
