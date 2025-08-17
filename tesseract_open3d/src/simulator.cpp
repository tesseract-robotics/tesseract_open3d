/**
 * @file simulator.cc
 * @brief Implementation of the o3d_sensors multi-sensor simulator.
 */

#include <tesseract_open3d/simulator.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

using open3d::core::Float32;
using open3d::core::Int32;
using open3d::core::Tensor;
using open3d::t::geometry::PointCloud;
using open3d::t::geometry::RaycastingScene;
using open3d::t::geometry::TriangleMesh;

namespace tesseract_o3d
{
namespace {

// Rotation-only quaternion from an isometry.
Eigen::Quaterniond ToQuat(const Eigen::Isometry3d& T) {
    return Eigen::Quaterniond(T.linear());
}

// Apply a rigid transform to a *copy* of a tensor mesh (in place).
TriangleMesh TransformCopy(const TriangleMesh& m, const Eigen::Isometry3d& T) {
    TriangleMesh out = m;
    const Eigen::Matrix4f tf = T.matrix().cast<float>();
    // RaycastingScene expects t::geometry transforms as core::Tensor (4x4).
    Tensor T_core(std::vector<float>(tf.data(), tf.data() + 16),
                  {4, 4}, Float32);
    out.Transform(T_core);
    return out;
}

}  // namespace

// ============================== World ========================================

World::World() : scene_(std::make_unique<RaycastingScene>()) {}

void World::setStaticMesh(const TriangleMesh& mesh) { static_mesh_ = mesh; }

int World::registerInstance(std::shared_ptr<TriangleMesh> base_mesh) {
    const int id = next_instance_id_++;
    instances_[id].base = std::move(base_mesh);
    instances_[id].pose = Eigen::Isometry3d::Identity();
    return id;
}

void World::updateInstancePose(int instance_id, const Eigen::Isometry3d& pose) {
    auto it = instances_.find(instance_id);
    if (it != instances_.end()) it->second.pose = pose;
}

void World::updateInstanceMesh(
    int instance_id,
    std::shared_ptr<TriangleMesh> base_mesh) {
    auto it = instances_.find(instance_id);
    if (it != instances_.end()) it->second.base = std::move(base_mesh);
}

void World::commitFrame() {

    // fresh BVH each frame
    scene_ = std::make_unique<RaycastingScene>();

    // Add the static environment (already in world frame).
    if (!static_mesh_.IsEmpty()) {
        (void)scene_->AddTriangles(static_mesh_);
    }

    // Add each dynamic instance after applying its world pose.
    for (const auto& [id, inst] : instances_) {
        if (!inst.base || inst.base->IsEmpty()) continue;

        // Transform a copy into world frame, then add.
        TriangleMesh m = TransformCopy(*inst.base, inst.pose);
        (void)scene_->AddTriangles(m);
    }
}

// ============================== Sensors ======================================
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
class SpinningLidar final : public Sensor {
public:
    explicit SpinningLidar(const std::string& name, const SpinningLidarConfig& cfg)
        : Sensor(name, SensorKind::kSpinningLidar), cfg_(cfg),
        rng_(cfg.rng_seed),
        n_range_(0.0f, cfg.range_noise_std),
        n_az_(0.0f, cfg.azimuth_jitter_std_deg * static_cast<float>(M_PI / 180.0)),
        drop_(cfg.dropout_probability) {
        if (cfg_.vertical_angles_deg.empty()) {
            // With +Z forward, a narrow band around 0° is a sensible default.
            cfg_.vertical_angles_deg.resize(std::max(1, cfg_.rings));
            const float vmin = -15.0f, vmax = +15.0f;
            for (int i = 0; i < cfg_.rings; ++i) {
                const float u = (i + 0.5f) / static_cast<float>(cfg_.rings);
                cfg_.vertical_angles_deg[i] = vmin + u * (vmax - vmin);
            }
        }
    }

    SensorOutput capture(const World& world, double t) override {
        const int V = cfg_.rings, H = cfg_.cols, N = V * H;
        const double T_rev = 1.0 / std::max(1e-9, cfg_.spin_rate_hz);
        const float az_span =
            std::clamp(cfg_.fov_azimuth_deg, 0.0f, 360.0f) * static_cast<float>(M_PI / 180.0);

        Tensor rays({N, 6}, Float32);
        float* r = rays.GetDataPtr<float>();

        std::vector<int32_t> ring(N);
        std::vector<float> az(N), t_abs(N), elev(V);
        for (int i = 0; i < V; ++i) elev[i] = cfg_.vertical_angles_deg[i] * static_cast<float>(M_PI / 180.0);

        int k = 0;
        for (int vi = 0; vi < V; ++vi) {
            const float e = elev[vi];
            const float ce = std::cos(e), se = std::sin(e);
            for (int hi = 0; hi < H; ++hi, ++k) {
                const float u = (hi + 0.5f) / static_cast<float>(H);
                float az_i = u * az_span + n_az_(rng_);
                const float ca = std::cos(az_i), sa = std::sin(az_i);
                const double dt = static_cast<double>(u) * T_rev;
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
        const float* t_hit = out.at("t_hit").GetDataPtr<float>();

        std::vector<float> xyz; xyz.reserve(static_cast<size_t>(N) * 3);
        std::vector<float> rng; rng.reserve(N);
        std::vector<int32_t> ring_ok; ring_ok.reserve(N);
        std::vector<float> az_ok, t_ok; az_ok.reserve(N); t_ok.reserve(N);

        for (int i = 0; i < N; ++i) {
            float th = t_hit[i];
            if (!std::isfinite(th) || th < cfg_.min_range || th > cfg_.max_range) continue;
            if (cfg_.dropout_probability > 0.0f && drop_(rng_)) continue;

            th = std::max(cfg_.min_range, th + n_range_(rng_));

            const float* ray = r + 6 * i;
            const Eigen::Vector3f o(ray[0], ray[1], ray[2]);
            const Eigen::Vector3f d(ray[3], ray[4], ray[5]);
            const Eigen::Vector3f p = o + th * d;

            xyz.insert(xyz.end(), {p.x(), p.y(), p.z()});
            rng.push_back(th);
            ring_ok.push_back(ring[i]);
            az_ok.push_back(az[i]);
            t_ok.push_back(t_abs[i]);
        }

        SensorOutput so;
        if (!xyz.empty()) {
            PointCloud pc;
            const int M = static_cast<int>(xyz.size() / 3);
            Tensor pos({M, 3}, Float32);
            std::memcpy(pos.GetDataPtr<float>(), xyz.data(), xyz.size() * sizeof(float));
            pc.SetPointPositions(pos);
            pc.SetPointAttr("ranges", Tensor(rng, {M, 1}, Float32));
            pc.SetPointAttr("ring",   Tensor(ring_ok, {M, 1}, Int32));
            pc.SetPointAttr("azimuth",Tensor(az_ok,   {M, 1}, Float32));
            pc.SetPointAttr("time",   Tensor(t_ok,    {M, 1}, Float32));
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
class SolidStateLidar final : public Sensor {
public:
    SolidStateLidar(const std::string& name, const SolidStateConfig& cfg, bool rolling)
        : Sensor(name, SensorKind::kSolidStateLidar),
        cfg_(cfg),
        rolling_(rolling),
        rng_(cfg.rng_seed),
        n_range_(0.0f, cfg.range_noise_std),
        n_jh_(0.0f, cfg.jitter_h_std_deg * static_cast<float>(M_PI / 180.0)),
        n_jv_(0.0f, cfg.jitter_v_std_deg * static_cast<float>(M_PI / 180.0)),
        drop_(cfg.dropout_probability) {}

    SensorOutput capture(const World& world, double t) override {
        const int H = cfg_.height, W = cfg_.width, N = H * W;
        const double t_frame = 1.0 / std::max(1e-9, cfg_.frame_rate_hz);
        const float fov_h =
            std::clamp(cfg_.fov_h_deg, 0.0f, 360.0f) * static_cast<float>(M_PI / 180.0);
        const float fov_v =
            std::clamp(cfg_.fov_v_deg, 0.0f, 180.0f) * static_cast<float>(M_PI / 180.0);

        Tensor rays({N, 6}, Float32);
        float* r = rays.GetDataPtr<float>();
        std::vector<int32_t> row(N), col(N);
        std::vector<float> az(N), el(N), t_abs(N);

        int k = 0;
        for (int i = 0; i < H; ++i) {
            const float v = (i + 0.5f) / static_cast<float>(H);
            float el_i = (v - 0.5f) * fov_v + n_jv_(rng_);
            const float ce = std::cos(el_i), se = std::sin(el_i);

            for (int j = 0; j < W; ++j, ++k) {
                const float u = (j + 0.5f) / static_cast<float>(W);
                float az_j = (u - 0.5f) * fov_h + n_jh_(rng_);
                const float ca = std::cos(az_j), sa = std::sin(az_j);
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

                row[k] = i; col[k] = j; az[k] = az_j; el[k] = el_i; t_abs[k] = static_cast<float>(t_ray);
            }
        }

        const auto out = world.scene().CastRays(rays);
        const float* t_hit = out.at("t_hit").GetDataPtr<float>();

        std::vector<float> xyz; xyz.reserve(static_cast<size_t>(N) * 3);
        std::vector<float> rng; rng.reserve(N);
        std::vector<int32_t> row_ok, col_ok; row_ok.reserve(N); col_ok.reserve(N);
        std::vector<float> az_ok, el_ok, t_ok; az_ok.reserve(N); el_ok.reserve(N); t_ok.reserve(N);

        for (int i = 0; i < N; ++i) {
            float th = t_hit[i];
            if (!std::isfinite(th) || th < cfg_.min_range || th > cfg_.max_range) continue;
            if (cfg_.dropout_probability > 0.0f && drop_(rng_)) continue;

            th = std::max(cfg_.min_range, th + n_range_(rng_));
            const float* ray = r + 6 * i;
            const Eigen::Vector3f o(ray[0], ray[1], ray[2]);
            const Eigen::Vector3f d(ray[3], ray[4], ray[5]);
            const Eigen::Vector3f p = o + th * d;

            xyz.insert(xyz.end(), {p.x(), p.y(), p.z()});
            rng.push_back(th);
            row_ok.push_back(row[i]); col_ok.push_back(col[i]);
            az_ok.push_back(az[i]); el_ok.push_back(el[i]); t_ok.push_back(t_abs[i]);
        }

        SensorOutput so;
        if (!xyz.empty()) {
            PointCloud pc;
            const int M = static_cast<int>(xyz.size() / 3);
            Tensor pos({M, 3}, Float32);
            std::memcpy(pos.GetDataPtr<float>(), xyz.data(), xyz.size() * sizeof(float));
            pc.SetPointPositions(pos);
            pc.SetPointAttr("ranges", Tensor(rng, {M, 1}, Float32));
            pc.SetPointAttr("row",    Tensor(row_ok, {M, 1}, Int32));
            pc.SetPointAttr("col",    Tensor(col_ok, {M, 1}, Int32));
            pc.SetPointAttr("azimuth",Tensor(az_ok,   {M, 1}, Float32));
            pc.SetPointAttr("elevation",Tensor(el_ok, {M, 1}, Float32));
            pc.SetPointAttr("time",   Tensor(t_ok,    {M, 1}, Float32));
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
class DepthCamera final : public Sensor {
public:
    DepthCamera(const std::string& name, const DepthConfig& cfg, bool keep_org)
        : Sensor(name, SensorKind::kDepthCamera), cfg_(cfg), keep_org_(keep_org) {}

    SensorOutput capture(const World& world, double /*t*/) override {
        const int H = cfg_.K.height, W = cfg_.K.width;

        // Compute horizontal FOV (deg) from fx, width
        const double fov_x_deg = 2.0 * std::atan(static_cast<double>(cfg_.K.width) / (2.0 * cfg_.K.fx))
                                 * 180.0 / M_PI;

        // Camera position/orientation from pose_
        const Eigen::Vector3f eye   = pose_.translation().cast<float>();
        const Eigen::Vector3f fwd   = (pose_.linear() * Eigen::Vector3d::UnitZ()).cast<float>(); // +Z forward
        const Eigen::Vector3f up    = (pose_.linear() * Eigen::Vector3d::UnitY()).cast<float>(); // +Y down in our basis
        const Eigen::Vector3f center = eye + fwd;

        Tensor center_t(std::vector<float>{center.x(),center.y(),center.z()}, {3}, Float32);
        Tensor eye_t   (std::vector<float>{eye.x(),eye.y(),eye.z()},           {3}, Float32);
        Tensor up_t    (std::vector<float>{up.x(),up.y(),up.z()},              {3}, Float32);

        Tensor rays = RaycastingScene::CreateRaysPinhole(
            /*fov_deg=*/fov_x_deg, center_t, eye_t, up_t, cfg_.K.width, cfg_.K.height);

        const auto out = world.scene().CastRays(rays);
        const float* t_hit = out.at("t_hit").GetDataPtr<float>();

        // Depth (meters) with noise/dropouts.
        Tensor depth({H, W}, Float32);
        float* z = depth.GetDataPtr<float>();
        std::mt19937 rng(cfg_.rng_seed + 12345);
        std::normal_distribution<float> nd(0.0f, cfg_.depth_noise_std);
        std::bernoulli_distribution drop(cfg_.dropout_probability);

        for (int i = 0; i < H * W; ++i) {
            float d = t_hit[i];
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
        if (keep_org_) xyz.resize(static_cast<size_t>(H) * W * 3,
                       std::numeric_limits<float>::quiet_NaN());
        else xyz.reserve(static_cast<size_t>(H) * W * 3);

        const float fx = cfg_.K.fx, fy = cfg_.K.fy, cx = cfg_.K.cx, cy = cfg_.K.cy;
        const Eigen::Matrix3f R = pose_.linear().cast<float>();
        const Eigen::Vector3f Ow = pose_.translation().cast<float>();

        auto write_pt = [&](int idx, const Eigen::Vector3f& Pw) {
            if (keep_org_) {
                xyz[idx * 3 + 0] = Pw.x();
                xyz[idx * 3 + 1] = Pw.y();
                xyz[idx * 3 + 2] = Pw.z();
            } else {
                xyz.insert(xyz.end(), {Pw.x(), Pw.y(), Pw.z()});
            }
        };

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const int idx = y * W + x;
                const float d = z[idx];
                if (!std::isfinite(d)) continue;
                const float px = (static_cast<float>(x) - cx) / fx;
                const float py = (static_cast<float>(y) - cy) / fy;
                const Eigen::Vector3f Pc(px * d, py * d, d);  // camera frame (+Z forward)
                const Eigen::Vector3f Pw = Ow + R * Pc;       // world frame
                write_pt(idx, Pw);
            }
        }

        PointCloud pc;
        if (!xyz.empty()) {
            const int M = keep_org_ ? (H * W) : static_cast<int>(xyz.size() / 3);
            Tensor pos({M, 3}, Float32);
            std::memcpy(pos.GetDataPtr<float>(), xyz.data(), static_cast<size_t>(M) * 3 * sizeof(float));
            pc.SetPointPositions(pos);
        }

        SensorOutput so;
        so.depth = DepthFrame{depth, pc};
        return so;
    }

private:
    DepthConfig cfg_;
    bool keep_org_ = false;
};

// ============================== Factories ====================================

std::unique_ptr<Sensor> makeSpinningLidar(const std::string& name,
                                          const SpinningLidarConfig& cfg) {
    return std::make_unique<SpinningLidar>(name, cfg);
}

std::unique_ptr<Sensor> makeSolidStateLidar(const std::string& name,
                                            const SolidStateConfig& cfg,
                                            bool rolling_shutter) {
    return std::make_unique<SolidStateLidar>(name, cfg, rolling_shutter);
}

std::unique_ptr<Sensor> makeDepthCamera(const std::string& name,
                                        const DepthConfig& cfg,
                                        bool keep_organized_points) {
    return std::make_unique<DepthCamera>(name, cfg, keep_organized_points);
}

// ============================== Simulator ====================================

int Simulator::addSensor(std::unique_ptr<Sensor> sensor) {
    const int id = next_sensor_id_++;
    sensors_[id] = std::move(sensor);
    return id;
}

void Simulator::setSensorPose(int sensor_id, const Eigen::Isometry3d& pose) {
    std::cout << pose.matrix() << "\n";
    auto it = sensors_.find(sensor_id);
    if (it != sensors_.end()) it->second->setPose(pose);
}

std::map<int, SensorOutput> Simulator::tick(double t) {
    world_.commitFrame();
    std::map<int, SensorOutput> out;
    for (auto& [id, sensor] : sensors_) {
        out[id] = sensor->capture(world_, t);
    }
    return out;
}

}  // namespace tesseract_o3d
