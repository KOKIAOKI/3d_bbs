#include <gpu_bbs3d/voxelmaps.cuh>
#include <gpu_bbs3d/bbs3d.cuh>
#include <gpu_bbs3d/stream_manager/check_error.cuh>

namespace gpu {
BBS3D::BBS3D()
: v_rate_(2.0f),
  branch_copy_size_(10000),
  score_threshold_percentage_(0.0),
  src_size_(-1),
  has_localized_(false),
  voxelmaps_folder_name_("voxelmaps_coords") {
  check_error << cudaStreamCreate(&stream);

  inv_v_rate_ = 1.0f / v_rate_;
  min_rpy_ << -0.02f, -0.02f, 0.0f;
  max_rpy_ << 0.02f, 0.02f, 2 * M_PI;
}

BBS3D::~BBS3D() {
  check_error << cudaStreamDestroy(stream);
}

void BBS3D::set_tar_points(const std::vector<Eigen::Vector3f>& points, float min_level_res, int max_level) {
  voxelmaps_ptr_.reset(new VoxelMaps);
  voxelmaps_ptr_->set_min_res(min_level_res);
  voxelmaps_ptr_->set_max_level(max_level);
  voxelmaps_ptr_->set_voxel_expantion_rate(v_rate_);
  voxelmaps_ptr_->create_voxelmaps(points, stream);

  // Detect translation range from target points
  set_trans_search_range(points);
}

void BBS3D::set_src_points(const std::vector<Eigen::Vector3f>& points) {
  src_size_ = points.size();
  src_points_.clear();
  src_points_.shrink_to_fit();
  src_points_.resize(src_size_);
  std::copy(points.begin(), points.end(), src_points_.begin());

  d_src_points_.clear();
  d_src_points_.shrink_to_fit();
  d_src_points_.resize(src_size_);
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_src_points_.data()),
    points.data(),
    sizeof(Eigen::Vector3f) * src_size_,
    cudaMemcpyHostToDevice,
    stream);
}

void BBS3D::set_trans_search_range(const std::vector<Eigen::Vector3f>& points) {
  // The minimum and maximum x, y, z values are selected from the 3D coordinate vector.
  Eigen::Vector3f min_xyz = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f max_xyz = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

  for (const auto& point : points) {
    min_xyz = min_xyz.cwiseMin(point);
    max_xyz = max_xyz.cwiseMax(point);
  }

  const int max_level = voxelmaps_ptr_->get_max_level();
  const float top_res = voxelmaps_ptr_->voxelmaps_info_[max_level].res;
  init_tx_range_ = std::make_pair<int, int>(std::floor(min_xyz.x() / top_res), std::ceil(max_xyz.x() / top_res));
  init_ty_range_ = std::make_pair<int, int>(std::floor(min_xyz.y() / top_res), std::ceil(max_xyz.y() / top_res));
  init_tz_range_ = std::make_pair<int, int>(std::floor(min_xyz.z() / top_res), std::ceil(max_xyz.z() / top_res));
}

void BBS3D::calc_angluar_info(std::vector<AngularInfo>& ang_info_vec) {
  float max_norm = src_points_[0].norm();
  for (const auto& point : src_points_) {
    float norm = point.norm();
    if (norm > max_norm) {
      max_norm = norm;
    }
  }

  const int max_level = voxelmaps_ptr_->get_max_level();
  for (int i = max_level; i >= 0; i--) {
    const float cosine = 1 - (std::pow(voxelmaps_ptr_->voxelmaps_info_[i].res, 2) / std::pow(max_norm, 2)) * 0.5f;
    float ori_res = std::acos(max(cosine, -1.0f));
    ori_res = std::floor(ori_res * 10000) / 10000;
    Eigen::Vector3f rpy_res_temp;
    rpy_res_temp.x() = ori_res <= (max_rpy_.x() - min_rpy_.x()) ? ori_res : 0.0f;
    rpy_res_temp.y() = ori_res <= (max_rpy_.y() - min_rpy_.y()) ? ori_res : 0.0f;
    rpy_res_temp.z() = ori_res <= (max_rpy_.z() - min_rpy_.z()) ? ori_res : 0.0f;

    Eigen::Vector3f max_rpy_piece;
    if (i == max_level) {
      max_rpy_piece = max_rpy_ - min_rpy_;
    } else {
      max_rpy_piece.x() = ang_info_vec[i + 1].rpy_res.x() != 0.0f ? ang_info_vec[i + 1].rpy_res.x() : max_rpy_.x() - min_rpy_.x();
      max_rpy_piece.y() = ang_info_vec[i + 1].rpy_res.y() != 0.0f ? ang_info_vec[i + 1].rpy_res.y() : max_rpy_.y() - min_rpy_.y();
      max_rpy_piece.z() = ang_info_vec[i + 1].rpy_res.z() != 0.0f ? ang_info_vec[i + 1].rpy_res.z() : max_rpy_.z() - min_rpy_.z();
    }

    // Angle division number
    Eigen::Vector3i num_division;
    num_division.x() = rpy_res_temp.x() != 0.0f ? std::ceil(max_rpy_piece.x() / rpy_res_temp.x()) : 1;
    num_division.y() = rpy_res_temp.y() != 0.0f ? std::ceil(max_rpy_piece.y() / rpy_res_temp.y()) : 1;
    num_division.z() = rpy_res_temp.z() != 0.0f ? std::ceil(max_rpy_piece.z() / rpy_res_temp.z()) : 1;
    ang_info_vec[i].num_division = num_division;

    // Bisect an angle
    ang_info_vec[i].rpy_res.x() = num_division.x() != 1 ? max_rpy_piece.x() / num_division.x() : 0.0f;
    ang_info_vec[i].rpy_res.y() = num_division.y() != 1 ? max_rpy_piece.y() / num_division.y() : 0.0f;
    ang_info_vec[i].rpy_res.z() = num_division.z() != 1 ? max_rpy_piece.z() / num_division.z() : 0.0f;

    ang_info_vec[i].min_rpy.x() = ang_info_vec[i].rpy_res.x() != 0.0f && ang_info_vec[i + 1].rpy_res.x() == 0.0f ? min_rpy_.x() : 0.0f;
    ang_info_vec[i].min_rpy.y() = ang_info_vec[i].rpy_res.y() != 0.0f && ang_info_vec[i + 1].rpy_res.y() == 0.0f ? min_rpy_.y() : 0.0f;
    ang_info_vec[i].min_rpy.z() = ang_info_vec[i].rpy_res.z() != 0.0f && ang_info_vec[i + 1].rpy_res.z() == 0.0f ? min_rpy_.z() : 0.0f;
  }
}

std::vector<DiscreteTransformation> BBS3D::create_init_transset(const AngularInfo& init_ang_info) {
  const int init_transset_size = (init_tx_range_.second - init_tx_range_.first + 1) * (init_ty_range_.second - init_ty_range_.first + 1) *
                                 (init_tz_range_.second - init_tz_range_.first + 1) * (init_ang_info.num_division.x()) *
                                 (init_ang_info.num_division.y()) * (init_ang_info.num_division.z());

  const int max_level = voxelmaps_ptr_->get_max_level();
  const float init_trans_res = voxelmaps_ptr_->voxelmaps_info_[max_level].res;

  std::vector<DiscreteTransformation> transset;
  transset.reserve(init_transset_size);
  for (int tx = init_tx_range_.first; tx <= init_tx_range_.second; tx++) {
    for (int ty = init_ty_range_.first; ty <= init_ty_range_.second; ty++) {
      for (int tz = init_tz_range_.first; tz <= init_tz_range_.second; tz++) {
        for (int roll = 0; roll < init_ang_info.num_division.x(); roll++) {
          for (int pitch = 0; pitch < init_ang_info.num_division.y(); pitch++) {
            for (int yaw = 0; yaw < init_ang_info.num_division.z(); yaw++) {
              transset.emplace_back(DiscreteTransformation(
                0,
                max_level,
                init_trans_res,
                tx * init_trans_res,
                ty * init_trans_res,
                tz * init_trans_res,
                roll * init_ang_info.rpy_res.x() + init_ang_info.min_rpy.x(),
                pitch * init_ang_info.rpy_res.y() + init_ang_info.min_rpy.y(),
                yaw * init_ang_info.rpy_res.z() + init_ang_info.min_rpy.z()));
            }
          }
        }
      }
    }
  }
  return transset;
}

void BBS3D::localize() {
  best_score_ = 0;
  const int score_threshold = std::floor(src_size_ * score_threshold_percentage_);
  int best_score = score_threshold;
  DiscreteTransformation best_trans(best_score);

  // Preapre initial transset
  const int max_level = voxelmaps_ptr_->get_max_level();
  std::vector<AngularInfo> ang_info_vec(max_level + 1);
  calc_angluar_info(ang_info_vec);
  const auto init_transset = create_init_transset(ang_info_vec[max_level]);

  // Calc initial transset scores
  const auto init_transset_output = calc_scores(init_transset);

  std::priority_queue<DiscreteTransformation> trans_queue(init_transset_output.begin(), init_transset_output.end());

  std::vector<DiscreteTransformation> branch_stock;
  branch_stock.reserve(branch_copy_size_);
  while (!trans_queue.empty()) {
    auto trans = trans_queue.top();
    trans_queue.pop();

    // Calculate remaining branch_stock when queue is empty
    if (trans_queue.empty() && !branch_stock.empty()) {
      const auto transset_output = calc_scores(branch_stock);
      for (const auto& output : transset_output) {
        if (output.score < best_score) continue;  // pruning
        trans_queue.push(output);
      }
      branch_stock.clear();
    }

    // pruning
    if (trans.score < best_score) {
      continue;
    }

    if (trans.is_leaf()) {
      best_trans = trans;
      best_score = trans.score;
    } else {
      const int child_level = trans.level - 1;
      const float child_res = trans.resolution * inv_v_rate_;
      trans.branch(branch_stock, child_level, child_res, static_cast<int>(v_rate_), ang_info_vec[child_level]);
    }

    if (branch_stock.size() >= branch_copy_size_) {
      const auto transset_output = calc_scores(branch_stock);
      for (const auto& output : transset_output) {
        if (output.score < best_score) continue;  // pruning
        trans_queue.push(output);
      }
      branch_stock.clear();
    }
  }

  if (best_score == score_threshold) {
    has_localized_ = false;
    return;
  }

  global_pose_ = best_trans.create_matrix();
  best_score_ = best_score;
  has_localized_ = true;
}
}  // namespace gpu