#include <cpu_bbs3d/bbs3d.hpp>

namespace cpu {
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

Eigen::Matrix4d DiscreteTransformation::create_matrix(const double trans_res, const Eigen::Vector3d& rpy_res, const Eigen::Vector3d& min_rpy) const {
  Eigen::Translation3d translation(x * trans_res, y * trans_res, z * trans_res);
  Eigen::AngleAxisd rollAngle(roll * rpy_res.x() + min_rpy.x(), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch * rpy_res.y() + min_rpy.y(), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw * rpy_res.z() + min_rpy.z(), Eigen::Vector3d::UnitZ());
  return (translation * yawAngle * pitchAngle * rollAngle).matrix();
}

std::vector<DiscreteTransformation> DiscreteTransformation::branch(const int child_level, const int v_rate, const Eigen::Vector3i& num_division)
  const {
  std::vector<DiscreteTransformation> b;
  b.reserve(v_rate * v_rate * v_rate * num_division.x() * num_division.y() * num_division.z());

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
  return b;
}

void DiscreteTransformation::calc_score(
  const std::vector<Eigen::Vector4i>& buckets,
  const double trans_res,
  const Eigen::Vector3d& rpy_res,
  const Eigen::Vector3d& min_rpy,
  const int max_bucket_scan_count,
  const std::vector<Eigen::Vector3d>& points) {
  const int num_buckets = buckets.size();
  const double inv_res = 1.0 / trans_res;
  Eigen::Transform<double, 3, Eigen::Affine> transform;
  transform = create_matrix(trans_res, rpy_res, min_rpy);

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