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

  // BFS
  bool operator<(const DiscreteTransformation& rhs) const { return score < rhs.score; }

  // DFS
  // bool operator<(const DiscreteTransformation& rhs) const {
  //   if (level == rhs.level) {
  //     return score < rhs.score;
  //   }
  //   return level > rhs.level;
  // }

  bool is_leaf() const { return level == 0; }

  void print() {
    std::cout << "[DiscreteTransformation] score: " << score << " level: " << level << " x: " << x << " y: " << y << " z: " << z << " roll: " << roll
              << "pitch; " << pitch << "yaw: " << yaw << std::endl;
  }

  template <typename T>
  Eigen::Matrix<T, 4, 4> create_matrix(const T trans_res, const T rpy_res) {
    Eigen::Translation<T, 3> translation(x * trans_res, y * trans_res, z * trans_res);
    Eigen::AngleAxis<T> x_axis(roll * rpy_res, Eigen::Matrix<T, 3, 1>::UnitX());
    Eigen::AngleAxis<T> y_axis(pitch * rpy_res, Eigen::Matrix<T, 3, 1>::UnitY());
    Eigen::AngleAxis<T> z_axis(yaw * rpy_res, Eigen::Matrix<T, 3, 1>::UnitZ());
    return (translation * z_axis * y_axis * x_axis).matrix();
  }

  void branch(std::vector<DiscreteTransformation>& b, const int child_level) {
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          b.emplace_back(DiscreteTransformation(0, child_level, x * 2 + i, y * 2 + j, z * 2 + k, roll, pitch, yaw));
        }
      }
    }
  }

  std::vector<DiscreteTransformation> branch(const int child_level) {
    std::vector<DiscreteTransformation> b;
    b.reserve(8);
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          b.emplace_back(DiscreteTransformation(0, child_level, x * 2 + i, y * 2 + j, z * 2 + k, roll, pitch, yaw));
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
