#include <cpu_bbs3d/bbs3d.hpp>

namespace cpu {
DiscreteTransformation::DiscreteTransformation() {}

DiscreteTransformation::DiscreteTransformation(int score) {
  this->score = score;
  this->level = 0;
  this->x = 0;
  this->y = 0;
  this->z = 0;
  this->roll = 0.0;
  this->pitch = 0.0;
  this->yaw = 0.0;
}

DiscreteTransformation::DiscreteTransformation(int score, int level, int x, int y, int z, double roll, double pitch, double yaw) {
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

Eigen::Matrix4d DiscreteTransformation::create_matrix(const double trans_res) const {
  Eigen::Translation3d translation(x * trans_res, y * trans_res, z * trans_res);
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  return (translation * yawAngle * pitchAngle * rollAngle).matrix();
}

std::vector<DiscreteTransformation> DiscreteTransformation::branch(const int child_level, const int v_rate, const AngularInfo& ang_info) const {
  std::vector<DiscreteTransformation> b;
  b.reserve(v_rate * v_rate * v_rate * ang_info.num_division.x() * ang_info.num_division.y() * ang_info.num_division.z());

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
  return b;
}

void DiscreteTransformation::calc_score(
  const std::vector<Eigen::Vector4i>& buckets,
  const double trans_res,
  const int max_bucket_scan_count,
  const std::vector<Eigen::Vector3d>& points) {
  const int num_buckets = buckets.size();
  const double inv_res = 1.0 / trans_res;
  Eigen::Transform<double, 3, Eigen::Affine> transform;
  transform = create_matrix(trans_res);

  for (int i = 0; i < points.size(); i++) {
    const Eigen::Vector3d transed_point = transform * points[i];
    const Eigen::Vector3i coord = (transed_point.array() * inv_res).floor().cast<int>();
    const std::uint32_t hash = (coord[0] * 73856093) ^ (coord[1] * 19349669) ^ (coord[2] * 83492791);

    for (int j = 0; j < max_bucket_scan_count; j++) {
      const std::uint32_t bucket_index = (hash + j) % num_buckets;
      const Eigen::Vector4i& bucket = buckets[bucket_index];

      if (bucket.x() != coord.x() || bucket.y() != coord.y() || bucket.z() != coord.z()) {
        continue;
      }

      if (bucket.w() == 1) {
        score++;
        break;
      }
    }
  }
}
}  // namespace cpu