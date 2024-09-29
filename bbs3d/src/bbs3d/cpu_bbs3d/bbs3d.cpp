#include "bbs3d/cpu_bbs3d/bbs3d.hpp"
#include "bbs3d/hash/hash.hpp"

namespace cpu {
BBSResult BBS3D::localize(const VoxelMaps<double>& voxelmaps, const std::vector<Eigen::Vector3d>& src_points) {
  BBSResult result;

  // Calc BBS time limit
  const auto start_time = std::chrono::system_clock::now();
  const auto time_limit = start_time + std::chrono::milliseconds(timeout_duration_msec);

  // Score threshold
  const int score_threshold = std::floor(src_points.size() * score_threshold_percentage);
  DiscreteTransformation best_trans(score_threshold);

  // Calc angular info
  double ang_res = 0;
  if (calc_ang_info) {
    auto max_norm_iter =
      std::max_element(src_points.begin(), src_points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.norm() < b.norm(); });

    const double cosine = 1 - (std::pow(voxelmaps.min_res(), 2) / std::pow(max_norm_iter->norm(), 2)) * 0.5;
    ang_res = std::acos(std::max(cosine, static_cast<double>(-1.0)));
    ang_res = std::floor(ang_res * 10000) / 10000;
  }
  // Preapre initial transset
  auto init_transset = create_init_transset(voxelmaps, ang_res);
  // Calc initial transset scores
  size_t max_level = voxelmaps.max_level();
  const auto& top_buckets = voxelmaps.buckets_vec[max_level];
  const auto& top_voxel_info = voxelmaps.info_vec[max_level];
#pragma omp parallel for num_threads(num_threads)
  for (int i = 0; i < init_transset.size(); i++) {
    calc_score(init_transset[i], top_buckets, top_voxel_info, ang_res, src_points);
  }

  // Main loop
  std::priority_queue<DiscreteTransformation> trans_queue(init_transset.begin(), init_transset.end());
  while (!trans_queue.empty()) {
    if (use_timeout && std::chrono::system_clock::now() > time_limit) {
      result.timed_out = true;
      break;
    }

    auto trans = trans_queue.top();
    trans_queue.pop();

    // pruning
    if (trans.score < result.best_score) continue;

    if (trans.is_leaf()) {
      best_trans = trans;
      result.best_score = trans.score;
    } else {
      // branch
      const int child_level = trans.level - 1;
      auto children = trans.branch(child_level);

      // calc score
      const auto& buckets = voxelmaps.buckets_vec[child_level];
      const auto& voxel_info = voxelmaps.info_vec[child_level];
#pragma omp parallel for num_threads(num_threads)
      for (int i = 0; i < children.size(); i++) {
        calc_score(children[i], buckets, voxel_info, ang_res, src_points);
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

  result.global_pose = best_trans.create_matrix(voxelmaps.info_vec[0].res, ang_res);
  result.localized = true;
  result.timed_out = false;

  return result;
}

std::vector<DiscreteTransformation> BBS3D::create_init_transset(const VoxelMaps<double>& voxelmaps, const double ang_res) {
  std::pair<int, int> init_tx_range, init_ty_range, init_tz_range, init_roll_range, init_pitch_range, init_yaw_range;
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

  init_roll_range = std::make_pair<int, int>(std::floor(min_rpy.x() / ang_res), std::ceil(max_rpy.x() / ang_res));
  init_pitch_range = std::make_pair<int, int>(std::floor(min_rpy.y() / ang_res), std::ceil(max_rpy.y() / ang_res));
  init_yaw_range = std::make_pair<int, int>(std::floor(min_rpy.z() / ang_res), std::ceil(max_rpy.z() / ang_res));

  const int init_transset_size = (init_tx_range.second - init_tx_range.first + 1) * (init_ty_range.second - init_ty_range.first + 1) *
                                 (init_tz_range.second - init_tz_range.first + 1) * (init_roll_range.second - init_roll_range.first + 1) *
                                 (init_pitch_range.second - init_pitch_range.first + 1) * (init_yaw_range.second - init_yaw_range.first + 1);

  int max_level = voxelmaps.max_level();
  std::vector<DiscreteTransformation> transset;
  transset.reserve(init_transset_size);
  for (int tx = init_tx_range.first; tx <= init_tx_range.second; tx++) {
    for (int ty = init_ty_range.first; ty <= init_ty_range.second; ty++) {
      for (int tz = init_tz_range.first; tz <= init_tz_range.second; tz++) {
        for (int roll = init_roll_range.first; roll <= init_roll_range.second; roll++) {
          for (int pitch = init_pitch_range.first; pitch <= init_pitch_range.second; pitch++) {
            for (int yaw = init_yaw_range.first; yaw <= init_yaw_range.second; yaw++) {
              transset.emplace_back(DiscreteTransformation(0, max_level, tx, ty, tz, roll, pitch, yaw));
            }
          }
        }
      }
    }
  }
  return transset;
}

void BBS3D::calc_score(
  DiscreteTransformation& trans,
  const std::vector<Eigen::Vector4i>& buckets,
  const VoxelMapInfo<double>& voxel_info,
  const double ang_res,
  const std::vector<Eigen::Vector3d>& points) {
  Eigen::Isometry3d transform;
  transform.matrix() = trans.create_matrix(voxel_info.res, ang_res);

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
        trans.score++;
        break;
      }
    }
  }
}
}  // namespace cpu
