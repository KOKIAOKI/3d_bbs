#include <gpu_bbs3d/bbs3d.cuh>

namespace gpu {
DiscreteTransformation::DiscreteTransformation() {}

DiscreteTransformation::DiscreteTransformation(int score) {
  this->score = score;
  this->level = 0;
  this->x = 0;
  this->y = 0;
  this->z = 0;
  this->roll = 0.0f;
  this->pitch = 0.0f;
  this->yaw = 0.0f;
}

DiscreteTransformation::DiscreteTransformation(int score, int level, int x, int y, int z, float roll, float pitch, float yaw) {
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

Eigen::Matrix4f DiscreteTransformation::create_matrix(const float trans_res) {
  Eigen::Translation3f translation(x * trans_res, y * trans_res, z * trans_res);
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
  return (translation * yawAngle * pitchAngle * rollAngle).matrix();
}

void DiscreteTransformation::branch(std::vector<DiscreteTransformation>& b, const int child_level, const int v_rate, const AngularInfo& ang_info) {
  for (int i = 0; i < v_rate; i++) {
    for (int j = 0; j < v_rate; j++) {
      for (int k = 0; k < v_rate; k++) {
        for (int l = 0; l < ang_info.num_division.x(); l++) {
          for (int m = 0; m < ang_info.num_division.y(); m++) {
            for (int n = 0; n < ang_info.num_division.z(); n++) {
              b.emplace_back(DiscreteTransformation(
                0,
                child_level,
                x * v_rate + i,
                y * v_rate + j,
                z * v_rate + k,
                roll + l * ang_info.rpy_res.x() + ang_info.min_rpy.x(),
                pitch + m * ang_info.rpy_res.y() + ang_info.min_rpy.y(),
                yaw + n * ang_info.rpy_res.z() + ang_info.min_rpy.z()));
            }
          }
        }
      }
    }
  }
}
}  // namespace gpu