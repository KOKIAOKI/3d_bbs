#include <gpu_bbs3d/bbs3d.cuh>
#include <gpu_bbs3d/stream_manager/check_error.cuh>

namespace gpu {
int gcd(int a, int b) {
  if (b == 0) {
    return a;
  }
  return gcd(b, a % b);
}

int lcm(int a, int b) {
  int gcd_result = gcd(a, b);
  return (a * b) / gcd_result;
}

BBS3D::BBS3D() : score_threshold_percentage_(0.0), src_size_in_graph_(-1), has_localized_(false) {
  check_error << cudaStreamCreate(&stream);

  stream_buffer_ptr_.reset(new StreamTempBufferRoundRobin);
  num_streams_ = stream_buffer_ptr_->init_num_streams;

  set_branch_copy_size(10000);
  d_counts_ = nullptr;

  min_rpy_ << -0.02f, -0.02f, 0.0f;
  max_rpy_ << 0.02f, 0.02f, 2 * M_PI;
}

BBS3D::~BBS3D() {
  check_error << cudaFreeAsync(d_counts_, stream);
  check_error << cudaStreamDestroy(stream);
}

void BBS3D::set_branch_copy_size(const int branch_copy_size) {
  branch_copy_size_ = branch_copy_size - branch_copy_size % lcm(num_streams_, 8);
  graph_size_ = branch_copy_size_ / num_streams_;  // Remainder of　this division is 0
}

void BBS3D::set_tar_points(const std::vector<Eigen::Vector3f>& points, float min_level_res, int max_level) {
  voxelmaps_ptr_.reset(new VoxelMaps);

  tar_points_.clear();
  tar_points_.shrink_to_fit();
  tar_points_.resize(points.size());
  std::copy(points.begin(), points.end(), tar_points_.begin());

  voxelmaps_ptr_->set_min_res(min_level_res);
  voxelmaps_ptr_->set_max_level(max_level);
  voxelmaps_ptr_->create_voxelmaps(points, stream);

  trans_search_range();
}

void BBS3D::set_src_points(const std::vector<Eigen::Vector3f>& points) {
  const int src_size = points.size();
  src_points_.clear();
  src_points_.shrink_to_fit();
  src_points_.resize(src_size);
  std::copy(points.begin(), points.end(), src_points_.begin());

  d_src_points_.clear();
  d_src_points_.shrink_to_fit();
  d_src_points_.resize(src_size);
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_src_points_.data()),
    points.data(),
    sizeof(Eigen::Vector3f) * src_size,
    cudaMemcpyHostToDevice,
    stream);

  if (src_size_in_graph_ != src_size) {
    src_size_in_graph_ = src_size;
    // Create source points index
    std::vector<int> h_counts;
    h_counts.resize(src_size);
    for (int i = 0; i < src_size; i++) {
      h_counts[i] = i;
    }
    if (d_counts_ != nullptr) check_error << cudaFreeAsync(d_counts_, stream);
    check_error << cudaMallocAsync((void**)&d_counts_, sizeof(int) * src_size, stream);
    check_error << cudaMemcpyAsync(d_counts_, h_counts.data(), sizeof(int) * src_size, cudaMemcpyHostToDevice, stream);
    check_error << cudaStreamSynchronize(stream);
    create_cuda_graphs();
  }
}

void BBS3D::trans_search_range() {
  // Detect translation range from target points
  Eigen::Vector3f min_xyz = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f max_xyz = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

  for (const auto& point : tar_points_) {
    min_xyz = min_xyz.cwiseMin(point);
    max_xyz = max_xyz.cwiseMax(point);
  }

  const int max_level = voxelmaps_ptr_->get_max_level();
  const float top_res = voxelmaps_ptr_->voxelmaps_info_[max_level].res;
  init_tx_range_ = std::make_pair<int, int>(std::floor(min_xyz.x() / top_res), std::ceil(max_xyz.x() / top_res));
  init_ty_range_ = std::make_pair<int, int>(std::floor(min_xyz.y() / top_res), std::ceil(max_xyz.y() / top_res));
  init_tz_range_ = std::make_pair<int, int>(std::floor(min_xyz.z() / top_res), std::ceil(max_xyz.z() / top_res));
}

void BBS3D::angluar_search_range(std::vector<AngularInfo>& ang_info_vec) {
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
  const int score_threshold = std::floor(src_size_in_graph_ * score_threshold_percentage_);
  int best_score = score_threshold;
  DiscreteTransformation best_trans(best_score);

  // Preapre initial transset
  const int max_level = voxelmaps_ptr_->get_max_level();
  std::vector<AngularInfo> ang_info_vec(max_level + 1);
  angluar_search_range(ang_info_vec);
  const auto init_transset = create_init_transset(ang_info_vec[max_level]);

  // Calc initial transset scores
  const auto init_transset_output = calc_scores(init_transset);  // TODO: calc scores by graph

  std::priority_queue<DiscreteTransformation> trans_queue(init_transset_output.begin(), init_transset_output.end());

  std::vector<DiscreteTransformation> branch_stock;
  branch_stock.reserve(branch_copy_size_);
  while (!trans_queue.empty()) {
    auto trans = trans_queue.top();
    trans_queue.pop();

    if (trans_queue.empty() && !branch_stock.empty()) {
      const auto transset_output = calc_scores(branch_stock);
      for (const auto& output : transset_output) {
        trans_queue.push(output);
      }
      branch_stock.clear();
    }

    if (trans.score < best_score) {
      continue;
    }

    if (trans.is_leaf()) {
      best_trans = trans;
      best_score = trans.score;
    } else {
      const int child_level = trans.level - 1;
      trans.branch(branch_stock, child_level, ang_info_vec[child_level]);
    }

    if (branch_stock.size() >= branch_copy_size_) {
      // TODO: fix cuda_graph calculation (Temporarily not using cuda_graph)
      const auto transset_output = calc_scores(branch_stock);
      for (const auto& output : transset_output) {
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