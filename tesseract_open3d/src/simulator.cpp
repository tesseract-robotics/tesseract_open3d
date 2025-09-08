#include <tesseract_open3d/simulator.h>
#include <tesseract_open3d/sensors.h>

// STD
#include <memory>
#include <utility>
#include <map>

namespace tesseract_o3d
{
World& Simulator::world() { return world_; }

const World& Simulator::world() const { return world_; }

int Simulator::addSensor(std::unique_ptr<Sensor> sensor)
{
  const int id = next_sensor_id_++;
  sensors_[id] = std::move(sensor);
  return id;
}

void Simulator::setSensorPose(int sensor_id, const Eigen::Isometry3d& pose)
{
  auto it = sensors_.find(sensor_id);
  if (it != sensors_.end())
    it->second->setPose(pose);
}

std::map<int, SensorOutput> Simulator::tick(double t)
{
  world_.commitFrame();
  std::map<int, SensorOutput> out;
  for (auto& [id, sensor] : sensors_)
  {
    out[id] = sensor->capture(world_, t);
  }
  return out;
}

}  // namespace tesseract_o3d
