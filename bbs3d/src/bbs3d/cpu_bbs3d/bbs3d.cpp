#include "bbs3d/cpu_bbs3d/bbs3d.hpp"
#include "bbs3d/hash/hash.hpp"

namespace cpu {
BBSResult BBS3D::localize(cpu::VoxelMaps<double>& voxelmaps, const std::vector<Eigen::Vector3d>& src_points) {
  BBSResult result;

  // Calc BBS time limit
  const auto start_time = std::chrono::system_clock::now();
  const auto time_limit = start_time + std::chrono::milliseconds(timeout_duration_msec);

  // Score threshold
  const int score_threshold = std::floor(src_points.size() * score_threshold_percentage);
  DiscreteTransformation<double> best_trans(score_threshold);

  // Calc angular info
  if (calc_ang_info) {
    const auto max_norm = calc_max_norm(src_points);
    voxelmaps.calc_angular_info(max_norm, min_rpy, max_rpy);
  }

  // Preapre initial transset
  auto init_transset = create_init_transset(voxelmaps);

  // Calc initial transset scores
#pragma omp parallel for num_threads(num_threads)
  for (int i = 0; i < init_transset.size(); i++) {
    calc_score(init_transset[i], voxelmaps, src_points);
  }

  // Main loop
  std::priority_queue<DiscreteTransformation<double>> trans_queue(init_transset.begin(), init_transset.end());
  while (!trans_queue.empty()) {
    if (use_timeout && std::chrono::system_clock::now() > time_limit) {
      result.timed_out = true;
      break;
    }

    auto trans = trans_queue.top();
    trans_queue.pop();

    // pruning
    if (trans.score < result.best_score) {
      continue;
    }

    if (trans.is_leaf()) {
      best_trans = trans;
      result.best_score = trans.score;
    } else {
      // branch
      const int child_level = trans.level - 1;
      auto children = trans.branch(child_level, voxelmaps.v_rate(), voxelmaps.ang_info_vec[child_level].num_division);

      // calc score
#pragma omp parallel for num_threads(num_threads)
      for (int i = 0; i < children.size(); i++) {
        calc_score(children[i], voxelmaps, src_points);
      }

      // pruning or push child to queue
      for (const auto& child : children) {
        if (child.score < result.best_score) continue;

        trans_queue.push(child);
      }
    }
  }

  // Calc localization elapsed time
  const auto end_time = std::chrono::system_clock::now();
  result.elapsed_time_msec = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1e6;

  // Not localized
  if (result.best_score == score_threshold || result.timed_out) {
    result.localized = false;
    return result;
  }

  result.global_pose = best_trans.create_matrix(voxelmaps.pose_to_matrix_tool(0));
  result.localized = true;
  result.timed_out = false;

  return result;
}

double BBS3D::calc_max_norm(const std::vector<Eigen::Vector3d>& src_points) {
  double max_norm = src_points[0].norm();
  for (const auto& point : src_points) {
    float norm = point.norm();
    if (norm > max_norm) {
      max_norm = norm;
    }
  }
  return max_norm;
}

std::vector<DiscreteTransformation<double>> BBS3D::create_init_transset(const cpu::VoxelMaps<double>& voxelmaps) {
  std::pair<int, int> init_tx_range, init_ty_range, init_tz_range;
  if (search_entire_map) {
    init_tx_range = voxelmaps.top_tx_range();
    init_ty_range = voxelmaps.top_ty_range();
    init_tz_range = voxelmaps.top_tz_range();
  } else {
    double top_res = voxelmaps.max_res();
    init_tx_range = std::make_pair<int, int>(std::floor(min_xyz.x() / top_res), std::ceil(max_xyz.x() / top_res));
    init_ty_range = std::make_pair<int, int>(std::floor(min_xyz.y() / top_res), std::ceil(max_xyz.y() / top_res));
    init_tz_range = std::make_pair<int, int>(std::floor(min_xyz.z() / top_res), std::ceil(max_xyz.z() / top_res));
  }

  const int max_level = voxelmaps.max_level();
  const auto& ang_num_division = voxelmaps.ang_info_vec[max_level].num_division;

  const int init_transset_size = (init_tx_range.second - init_tx_range.first + 1) * (init_ty_range.second - init_ty_range.first + 1) *
                                 (init_tz_range.second - init_tz_range.first + 1) * (ang_num_division.x()) * (ang_num_division.y()) *
                                 (ang_num_division.z());

  std::vector<DiscreteTransformation<double>> transset;
  transset.reserve(init_transset_size);
  for (int tx = init_tx_range.first; tx <= init_tx_range.second; tx++) {
    for (int ty = init_ty_range.first; ty <= init_ty_range.second; ty++) {
      for (int tz = init_tz_range.first; tz <= init_tz_range.second; tz++) {
        for (int roll = 0; roll < ang_num_division.x(); roll++) {
          for (int pitch = 0; pitch < ang_num_division.y(); pitch++) {
            for (int yaw = 0; yaw < ang_num_division.z(); yaw++) {
              transset.emplace_back(DiscreteTransformation<double>(0, max_level, tx, ty, tz, roll, pitch, yaw));
            }
          }
        }
      }
    }
  }
  return transset;
}

void BBS3D::calc_score(DiscreteTransformation<double>& trans, const cpu::VoxelMaps<double>& voxelmaps, const std::vector<Eigen::Vector3d>& points) {
  const auto& buckets = voxelmaps.buckets_vec[trans.level];
  const auto& voxel_info = voxelmaps.info_vec[trans.level];

  Eigen::Transform<double, 3, Eigen::Affine> transform;
  transform = trans.create_matrix(voxelmaps.pose_to_matrix_tool(trans.level));

  for (int i = 0; i < points.size(); i++) {
    const Eigen::Vector3d transed_point = transform * points[i];
    const Eigen::Vector3i coord = (transed_point.array() * voxel_info.inv_res).floor().cast<int>();
    const std::uint32_t hash = hash::coord_to_hash(coord);

    for (int j = 0; j < voxel_info.max_bucket_scan_count; j++) {
      const std::uint32_t bucket_index = (hash + j) % voxel_info.num_buckets;
      const Eigen::Vector4i& bucket = buckets[bucket_index];

      if (bucket.x() != coord.x() || bucket.y() != coord.y() || bucket.z() != coord.z()) {
        continue;
      }

      if (bucket.w() == 1) {
        trans.score++;  // TODO reduction
        break;
      }
    }
  }
}

}  // namespace cpu
