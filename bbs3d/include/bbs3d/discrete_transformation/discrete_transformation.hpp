#pragma once
#include <Eigen/Core>

class DiscreteTransformation {
public:
  DiscreteTransformation() : score(0), level(0), x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}

  DiscreteTransformation(int score) : score(score), level(0), x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}

  DiscreteTransformation(int score, int level, int x, int y, int z, int roll, int pitch, int yaw)
  : score(score),
    level(level),
    x(x),
    y(y),
    z(z),
    roll(roll),
    pitch(pitch),
    yaw(yaw) {}

  ~DiscreteTransformation() {}

  bool operator<(const DiscreteTransformation& rhs) const { return level < rhs.level; }

  bool is_leaf() const { return level == 0; }

  void print() {
    std::cout << "[DiscreteTransformation] score: " << score << " level: " << level << " x: " << x << " y: " << y << " z: " << z << " roll: " << roll
              << "pitch; " << pitch << "yaw: " << yaw << std::endl;
  }

  template <typename T>
  Eigen::Matrix<T, 4, 4> create_matrix(const T trans_res, const Eigen::Matrix<T, 3, 1>& rpy_res, const Eigen::Matrix<T, 3, 1>& min_rpy) {
    Eigen::Translation<T, 3> translation(x * trans_res, y * trans_res, z * trans_res);
    Eigen::AngleAxis<T> x_axis(roll * rpy_res.x() + min_rpy.x(), Eigen::Matrix<T, 3, 1>::UnitX());
    Eigen::AngleAxis<T> y_axis(pitch * rpy_res.y() + min_rpy.y(), Eigen::Matrix<T, 3, 1>::UnitY());
    Eigen::AngleAxis<T> z_axis(yaw * rpy_res.z() + min_rpy.z(), Eigen::Matrix<T, 3, 1>::UnitZ());
    return (translation * z_axis * y_axis * x_axis).matrix();
  }

  void branch(std::vector<DiscreteTransformation>& b, const int child_level, const Eigen::Vector3i& num_division) {
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          for (int l = 0; l < num_division.x(); l++) {
            for (int m = 0; m < num_division.y(); m++) {
              for (int n = 0; n < num_division.z(); n++) {
                b.emplace_back(DiscreteTransformation(
                  0,
                  child_level,
                  x * 2 + i,
                  y * 2 + j,
                  z * 2 + k,
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

  std::vector<DiscreteTransformation> branch(const int child_level, const Eigen::Vector3i& num_division) {
    std::vector<DiscreteTransformation> b;
    b.reserve(8 * num_division.x() * num_division.y() * num_division.z());
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          for (int l = 0; l < num_division.x(); l++) {
            for (int m = 0; m < num_division.y(); m++) {
              for (int n = 0; n < num_division.z(); n++) {
                b.emplace_back(DiscreteTransformation(
                  0,
                  child_level,
                  x * 2 + i,
                  y * 2 + j,
                  z * 2 + k,
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

public:
  int score;
  int level;
  int x;
  int y;
  int z;
  int roll;
  int pitch;
  int yaw;
};
