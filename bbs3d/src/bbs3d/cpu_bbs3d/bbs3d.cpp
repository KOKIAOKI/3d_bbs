#include "bbs3d/cpu_bbs3d/bbs3d.hpp"
#include "bbs3d/hash/hash.hpp"

namespace cpu {
BBSResult BBS3D::localize(const VoxelMaps<double>& voxelmaps, const std::vector<Eigen::Vector3d>& src_points) {
  BBSResult result;

  // Score threshold
  const int score_threshold = std::floor(src_points.size() * score_threshold_percentage);
  DiscreteTransformation best_trans(score_threshold);

  // Calc angular info
  if (calc_ang_info) {
    auto max_norm_iter =
      std::max_element(src_points.begin(), src_points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.norm() < b.norm(); });
    calc_angular_info(voxelmaps, max_norm_iter->norm());
  }

  // Calc BBS time limit
  const auto start_time = std::chrono::system_clock::now();
  const auto time_limit = start_time + std::chrono::milliseconds(timeout_duration_msec);

  // Preapre initial transset
  auto init_transset = create_init_transset(voxelmaps);

  // Calc initial transset scores
  size_t max_level = voxelmaps.max_level();
  const auto& top_buckets = voxelmaps.buckets_vec[max_level];
  const auto& top_voxel_info = voxelmaps.info_vec[max_level];
  const auto& top_ang_info = ang_info_vec_[max_level];
#pragma omp parallel for num_threads(num_threads)
  for (int i = 0; i < init_transset.size(); i++) {
    calc_score(init_transset[i], top_buckets, top_voxel_info, top_ang_info, src_points);
  }

  // Main loop
  int failed_upperbound_estimate = 0;
  int count = 0;
  double percentage = 0.0;
  double error_percentage_sum = 0.0;
  double error_percentage_ave = 0.0;
  std::priority_queue<DiscreteTransformation> trans_queue(init_transset.begin(), init_transset.end());
  while (!trans_queue.empty()) {
    if (use_timeout && std::chrono::system_clock::now() > time_limit) {
      result.timed_out = true;
      break;
    }

    auto trans = trans_queue.top();
    trans_queue.pop();

    // pruning
    // if (trans.score < result.best_score) continue;

    if (trans.is_leaf()) {
      best_trans = trans;
      result.best_score = trans.score;
    } else {
      count++;

      // branch
      const int child_level = trans.level - 1;
      auto children = trans.branch(child_level, ang_info_vec_[child_level].num_division);

      // calc score
      const auto& buckets = voxelmaps.buckets_vec[child_level];
      const auto& voxel_info = voxelmaps.info_vec[child_level];
      const auto& ang_info = ang_info_vec_[child_level];
#pragma omp parallel for num_threads(num_threads)
      for (int i = 0; i < children.size(); i++) {
        calc_score(children[i], buckets, voxel_info, ang_info, src_points);
      }

      int children_max_score =
        std::max_element(children.begin(), children.end(), [](const DiscreteTransformation& a, const DiscreteTransformation& b) {
          return a.score < b.score;
        })->score;

      if (trans.score < children_max_score) {
        failed_upperbound_estimate++;
        percentage = (double)failed_upperbound_estimate / (double)count;

        // std::cout << trans.level << "  children score: " << children_max_score << " trans score: " << trans.score << std::endl;
        int error = children_max_score - trans.score;
        double error_percentage = (double)error / (double)children_max_score;
        error_percentage_sum += error_percentage;
        error_percentage_ave = error_percentage_sum / (double)failed_upperbound_estimate;
      }

      // pruning or push child to queue
      for (const auto& child : children) {
        trans_queue.push(child);
      }

      std::cout << "failed: " << percentage * 100.0 << "%" << " error_ave: " << error_percentage_ave * 100.0 << "%" << std::endl;
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

  result.global_pose = best_trans.create_matrix(voxelmaps.info_vec[0].res, ang_info_vec_[0].rpy_res, ang_info_vec_[0].min_rpy);
  result.localized = true;
  result.timed_out = false;

  return result;
}

void BBS3D::calc_angular_info(const VoxelMaps<double>& voxelmaps, const double max_norm) {
  const size_t max_level = voxelmaps.max_level();
  ang_info_vec_.clear();
  ang_info_vec_.resize(max_level + 1);

  for (int i = max_level; i >= 0; i--) {
    const double cosine = 1 - (std::pow(voxelmaps.info_vec[i].res, 2) / std::pow(max_norm, 2)) * 0.5;
    double ori_res = std::acos(std::max(cosine, static_cast<double>(-1.0)));
    ori_res = std::floor(ori_res * 10000) / 10000;
    Eigen::Vector3d rpy_res_temp;
    rpy_res_temp.x() = ori_res <= (max_rpy.x() - min_rpy.x()) ? ori_res : 0.0;
    rpy_res_temp.y() = ori_res <= (max_rpy.y() - min_rpy.y()) ? ori_res : 0.0;
    rpy_res_temp.z() = ori_res <= (max_rpy.z() - min_rpy.z()) ? ori_res : 0.0;

    Eigen::Vector3d max_rpypiece;
    if (i == max_level) {
      max_rpypiece = max_rpy - min_rpy;
    } else {
      max_rpypiece.x() = ang_info_vec_[i + 1].rpy_res.x() != 0.0 ? ang_info_vec_[i + 1].rpy_res.x() : max_rpy.x() - min_rpy.x();
      max_rpypiece.y() = ang_info_vec_[i + 1].rpy_res.y() != 0.0 ? ang_info_vec_[i + 1].rpy_res.y() : max_rpy.y() - min_rpy.y();
      max_rpypiece.z() = ang_info_vec_[i + 1].rpy_res.z() != 0.0 ? ang_info_vec_[i + 1].rpy_res.z() : max_rpy.z() - min_rpy.z();
    }

    // Angle division number
    Eigen::Vector3i num_division;
    num_division.x() = rpy_res_temp.x() != 0.0 ? std::ceil(max_rpypiece.x() / rpy_res_temp.x()) : 1;
    num_division.y() = rpy_res_temp.y() != 0.0 ? std::ceil(max_rpypiece.y() / rpy_res_temp.y()) : 1;
    num_division.z() = rpy_res_temp.z() != 0.0 ? std::ceil(max_rpypiece.z() / rpy_res_temp.z()) : 1;
    ang_info_vec_[i].num_division = num_division;

    // Bisect an angle
    ang_info_vec_[i].rpy_res.x() = num_division.x() != 1 ? max_rpypiece.x() / num_division.x() : 0.0;
    ang_info_vec_[i].rpy_res.y() = num_division.y() != 1 ? max_rpypiece.y() / num_division.y() : 0.0;
    ang_info_vec_[i].rpy_res.z() = num_division.z() != 1 ? max_rpypiece.z() / num_division.z() : 0.0;

    ang_info_vec_[i].min_rpy.x() = ang_info_vec_[i].rpy_res.x() != 0.0 && ang_info_vec_[i + 1].rpy_res.x() == 0.0 ? min_rpy.x() : 0.0;
    ang_info_vec_[i].min_rpy.y() = ang_info_vec_[i].rpy_res.y() != 0.0 && ang_info_vec_[i + 1].rpy_res.y() == 0.0 ? min_rpy.y() : 0.0;
    ang_info_vec_[i].min_rpy.z() = ang_info_vec_[i].rpy_res.z() != 0.0 && ang_info_vec_[i + 1].rpy_res.z() == 0.0 ? min_rpy.z() : 0.0;
  }
}

std::vector<DiscreteTransformation> BBS3D::create_init_transset(const VoxelMaps<double>& voxelmaps) {
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
  const auto& ang_num_division = ang_info_vec_[max_level].num_division;

  const int init_transset_size = (init_tx_range.second - init_tx_range.first + 1) * (init_ty_range.second - init_ty_range.first + 1) *
                                 (init_tz_range.second - init_tz_range.first + 1) * (ang_num_division.x()) * (ang_num_division.y()) *
                                 (ang_num_division.z());

  std::vector<DiscreteTransformation> transset;
  transset.reserve(init_transset_size);
  for (int tx = init_tx_range.first; tx <= init_tx_range.second; tx++) {
    for (int ty = init_ty_range.first; ty <= init_ty_range.second; ty++) {
      for (int tz = init_tz_range.first; tz <= init_tz_range.second; tz++) {
        for (int roll = 0; roll < ang_num_division.x(); roll++) {
          for (int pitch = 0; pitch < ang_num_division.y(); pitch++) {
            for (int yaw = 0; yaw < ang_num_division.z(); yaw++) {
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
  const AngularInfo& ang_info,
  const std::vector<Eigen::Vector3d>& points) {
  Eigen::Isometry3d transform;
  transform.matrix() = trans.create_matrix(voxel_info.res, ang_info.rpy_res, ang_info.min_rpy);

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
