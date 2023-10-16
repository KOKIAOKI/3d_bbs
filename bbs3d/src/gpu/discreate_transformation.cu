#include <gpu/bbs3d.cuh>

namespace gpu {
DiscreteTransformation::DiscreteTransformation() {}

DiscreteTransformation::DiscreteTransformation(int score) {
  this->score = score;
}

DiscreteTransformation::DiscreteTransformation(
  int score,
  int level,
  float resolution,
  float x,
  float y,
  float z,
  float roll,
  float pitch,
  float yaw) {
  this->score = score;
  this->level = level;
  this->resolution = resolution;
  this->x = x;
  this->y = y;
  this->z = z;
  this->roll = roll;
  this->pitch = pitch;
  this->yaw = yaw;
}

DiscreteTransformation::~DiscreteTransformation() {}

bool DiscreteTransformation::operator<(const DiscreteTransformation& rhs) const {
  return score < rhs.score;
}

bool DiscreteTransformation::is_leaf() const {
  return level == 0;
}

Eigen::Matrix4f DiscreteTransformation::create_matrix() {
  Eigen::Translation3f translation(x, y, z);
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
  return (translation * yawAngle * pitchAngle * rollAngle).matrix();
}

void DiscreteTransformation::branch(std::vector<DiscreteTransformation>& b, const int child_level, const AngularInfo& ang_info) {
  float child_res = resolution * 0.5f;
  for (int i = 0; i < ang_info.num_division.x(); i++) {
    for (int j = 0; j < ang_info.num_division.y(); j++) {
      for (int k = 0; k < ang_info.num_division.z(); k++) {
        b.emplace_back(DiscreteTransformation(
          0,
          child_level,
          child_res,
          x,
          y,
          z,
          roll + i * ang_info.rpy_res.x() + ang_info.min_rpy.x(),
          pitch + j * ang_info.rpy_res.y() + ang_info.min_rpy.y(),
          yaw + k * ang_info.rpy_res.z() + ang_info.min_rpy.z()));
        b.emplace_back(DiscreteTransformation(
          0,
          child_level,
          child_res,
          x + child_res,
          y,
          z,
          roll + i * ang_info.rpy_res.x() + ang_info.min_rpy.x(),
          pitch + j * ang_info.rpy_res.y() + ang_info.min_rpy.y(),
          yaw + k * ang_info.rpy_res.z() + ang_info.min_rpy.z()));
        b.emplace_back(DiscreteTransformation(
          0,
          child_level,
          child_res,
          x,
          y + child_res,
          z,
          roll + i * ang_info.rpy_res.x() + ang_info.min_rpy.x(),
          pitch + j * ang_info.rpy_res.y() + ang_info.min_rpy.y(),
          yaw + k * ang_info.rpy_res.z() + ang_info.min_rpy.z()));
        b.emplace_back(DiscreteTransformation(
          0,
          child_level,
          child_res,
          x + child_res,
          y + child_res,
          z,
          roll + i * ang_info.rpy_res.x() + ang_info.min_rpy.x(),
          pitch + j * ang_info.rpy_res.y() + ang_info.min_rpy.y(),
          yaw + k * ang_info.rpy_res.z() + ang_info.min_rpy.z()));
        b.emplace_back(DiscreteTransformation(
          0,
          child_level,
          child_res,
          x,
          y,
          z + child_res,
          roll + i * ang_info.rpy_res.x() + ang_info.min_rpy.x(),
          pitch + j * ang_info.rpy_res.y() + ang_info.min_rpy.y(),
          yaw + k * ang_info.rpy_res.z() + ang_info.min_rpy.z()));
        b.emplace_back(DiscreteTransformation(
          0,
          child_level,
          child_res,
          x + child_res,
          y,
          z + child_res,
          roll + i * ang_info.rpy_res.x() + ang_info.min_rpy.x(),
          pitch + j * ang_info.rpy_res.y() + ang_info.min_rpy.y(),
          yaw + k * ang_info.rpy_res.z() + ang_info.min_rpy.z()));
        b.emplace_back(DiscreteTransformation(
          0,
          child_level,
          child_res,
          x,
          y + child_res,
          z + child_res,
          roll + i * ang_info.rpy_res.x() + ang_info.min_rpy.x(),
          pitch + j * ang_info.rpy_res.y() + ang_info.min_rpy.y(),
          yaw + k * ang_info.rpy_res.z() + ang_info.min_rpy.z()));
        b.emplace_back(DiscreteTransformation(
          0,
          child_level,
          child_res,
          x + child_res,
          y + child_res,
          z + child_res,
          roll + i * ang_info.rpy_res.x() + ang_info.min_rpy.x(),
          pitch + j * ang_info.rpy_res.y() + ang_info.min_rpy.y(),
          yaw + k * ang_info.rpy_res.z() + ang_info.min_rpy.z()));
      }
    }
  }
}
}  // namespace gpu