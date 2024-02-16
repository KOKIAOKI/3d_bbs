#include <gpu_bbs3d/bbs3d.cuh>

namespace gpu {
DiscreteTransformation::DiscreteTransformation() {}

DiscreteTransformation::DiscreteTransformation(int score) {
  this->score = score;
  this->level = 0;
  this->x = 0;
  this->y = 0;
  this->z = 0;
  this->roll = 0;
  this->pitch = 0;
  this->yaw = 0;
}

DiscreteTransformation::DiscreteTransformation(int score, int level, int x, int y, int z, int roll, int pitch, int yaw) {
  this->score = score;
  this->level = level;
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

Eigen::Matrix4f DiscreteTransformation::create_matrix(const float trans_res, const Eigen::Vector3f& rpy_res, const Eigen::Vector3f& min_rpy) {
  Eigen::Translation3f translation(x * trans_res, y * trans_res, z * trans_res);
  Eigen::AngleAxisf rollAngle(roll * rpy_res.x() + min_rpy.x(), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch * rpy_res.y() + min_rpy.y(), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw * rpy_res.z() + min_rpy.z(), Eigen::Vector3f::UnitZ());
  return (translation * yawAngle * pitchAngle * rollAngle).matrix();
}

void DiscreteTransformation::branch(
  std::vector<DiscreteTransformation>& b,
  const int child_level,
  const int v_rate,
  const Eigen::Vector3i& num_division) {
  for (int i = 0; i < v_rate; i++) {
    for (int j = 0; j < v_rate; j++) {
      for (int k = 0; k < v_rate; k++) {
        for (int l = 0; l < num_division.x(); l++) {
          for (int m = 0; m < num_division.y(); m++) {
            for (int n = 0; n < num_division.z(); n++) {
              b.emplace_back(DiscreteTransformation(
                0,
                child_level,
                x * v_rate + i,
                y * v_rate + j,
                z * v_rate + k,
                roll * num_division.x() + l,
                pitch * num_division.y() + m,
                yaw * num_division.z() + n));
            }
          }
        }
      }
    }
  }
}
}  // namespace gpu