#pragma once
#include <Eigen/Dense>

namespace hash {
std::uint32_t coord_to_hash(const Eigen::Vector3i& coord) {
  return (coord[0] * 73856093) ^ (coord[1] * 19349669) ^ (coord[2] * 83492791);
}
}  // namespace hash