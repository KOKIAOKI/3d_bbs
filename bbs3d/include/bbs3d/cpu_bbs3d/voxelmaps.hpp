#pragma once
#include "bbs3d/pointcloud_iof/find_point_cloud_files.hpp"
#include "bbs3d/pointcloud_iof/pcd_io.hpp"
#include "bbs3d/hash/hash.hpp"

#include <algorithm>
#include <unordered_map>
#include <Eigen/Dense>
#include <boost/functional/hash/hash.hpp>

namespace cpu {
template <typename T>
struct VoxelMapInfo {
  int num_buckets;
  int max_bucket_scan_count;
  T res;      // voxel resolution
  T inv_res;  // inverse of voxel resolution
};

template <typename T>
struct AngularInfo {
  using Vector3 = Eigen::Matrix<T, 3, 1>;

  Eigen::Vector3i num_division;
  Vector3 rpy_res;
  Vector3 min_rpy;
};

// hash map
struct VectorHash {
  size_t operator()(const Eigen::Vector3i& x) const {
    size_t seed = 0;
    boost::hash_combine(seed, x[0]);
    boost::hash_combine(seed, x[1]);
    boost::hash_combine(seed, x[2]);
    return seed;
  }
};

struct VctorEqual {
  bool operator()(const Eigen::Vector3i& v1, const Eigen::Vector3i& v2) const { return v1 == v2; }
};

template <typename T>
class VoxelMaps {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using UnorderedVoxelMap = std::unordered_map<Eigen::Vector3i, int, VectorHash, VctorEqual>;
  using Buckets = std::vector<Eigen::Vector4i>;

public:
  using Ptr = std::shared_ptr<VoxelMaps>;
  using ConstPtr = std::shared_ptr<const VoxelMaps>;

  // public member variables
  std::vector<Buckets> buckets_vec;
  std::vector<VoxelMapInfo<T>> info_vec;
  std::vector<AngularInfo<T>> ang_info_vec;

  // get member variables
  T min_res() const { return min_res_; }
  T max_res() const { return max_res_; }
  size_t max_level() const { return max_level_; }
  size_t v_rate() const { return v_rate_; }
  size_t max_bucket_scan_count() const { return max_bucket_scan_count_; }
  std::string voxelmaps_folder_name() const { return voxelmaps_folder_name_; }
  std::pair<int, int> top_tx_range() const { return top_tx_range_; }
  std::pair<int, int> top_ty_range() const { return top_ty_range_; }
  std::pair<int, int> top_tz_range() const { return top_tz_range_; }

  std::tuple<T, Vector3, Vector3> pose_to_matrix_tool(const int level) const {
    const auto& ang_info = ang_info_vec[level];
    return std::make_tuple(info_vec[level].res, ang_info.rpy_res, ang_info.min_rpy);
  }

  void print() const {
    std::cout << "----------------------- VoxelMaps  parameters -----------------------" << std::endl;
    std::cout << "min_res: " << min_res_ << std::endl;
    std::cout << "max_res: " << max_res_ << std::endl;
    std::cout << "max_level: " << max_level_ << std::endl;
    std::cout << "v_rate: " << v_rate_ << std::endl;
    std::cout << "max_bucket_scan_count: " << max_bucket_scan_count_ << std::endl;
    std::cout << "voxelmaps_folder_name: " << voxelmaps_folder_name_ << std::endl;
  }

  VoxelMaps() {}
  ~VoxelMaps() {}

  // voxelmap coords IO
  bool set_voxelmaps(const std::string& folder_path) {
    const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
    if (!std::filesystem::exists(voxelmaps_folder_path)) {
      return false;
    }
    if (!load_voxel_params(voxelmaps_folder_path)) return false;
    if (!set_multi_buckets(voxelmaps_folder_path)) return false;
    return true;
  }

  bool save_voxelmaps(const std::string& folder_path) {
    if (!save_voxel_params(folder_path)) return false;
    if (!save_voxelmaps_pcd(folder_path)) return false;
    return true;
  }

  void create_voxelmaps(const std::vector<Vector3>& points, const T min_res, const int max_level) {
    min_res_ = min_res;
    max_level_ = max_level;

    const int buckets_vec_size = max_level_ + 1;
    buckets_vec.clear();
    info_vec.clear();
    buckets_vec.resize(buckets_vec_size);
    info_vec.resize(buckets_vec_size);

    for (int i = 0; i < buckets_vec_size; i++) {
      UnorderedVoxelMap unordered_voxelmap;
      T res = min_res_ * std::pow(v_rate_, i);

      std::vector<Eigen::Vector3i> coords(points.size());
      std::transform(points.begin(), points.end(), coords.begin(), [&](const Vector3& point) {
        return (point.array() / res).floor().template cast<int>();
      });

      // 0: empty, 1: voxel, -1: neighbor
      for (const auto& coord : coords) {
        if (unordered_voxelmap.count(coord) == 0) {
          // if the coord is not in the map
          unordered_voxelmap[coord] = 1;
        } else if (unordered_voxelmap[coord] == -1) {
          // if the coord is exist as a neighbor
          unordered_voxelmap[coord] = 1;
        } else if (unordered_voxelmap[coord] == 1) {
          // if the coord is exist as a voxel
          continue;
        }

        const auto neighbor_coords = create_neighbor_coords(coord);
        for (const auto& neighbor_coord : neighbor_coords) {
          if (unordered_voxelmap.count(neighbor_coord) == 0) {
            // add neighbor coords
            unordered_voxelmap[neighbor_coord] = -1;
          }
        }
      }

      const Buckets& buckets = create_hash_buckets(unordered_voxelmap);

      VoxelMapInfo<T> info;
      info.num_buckets = buckets.size();
      info.max_bucket_scan_count = max_bucket_scan_count_;
      info.res = res;
      info.inv_res = 1.0 / res;

      buckets_vec[i] = buckets;
      info_vec[i] = info;
    }

    calc_top_trans_range();
    max_res_ = info_vec[max_level_].res;
  }

  void calc_angular_info(T max_norm, const Vector3& min_rpy, const Vector3& max_rpy) {
    ang_info_vec.clear();
    ang_info_vec.resize(max_level_ + 1);

    for (int i = max_level_; i >= 0; i--) {
      const T cosine = 1 - (std::pow(info_vec[i].res, 2) / std::pow(max_norm, 2)) * 0.5;
      T ori_res = std::acos(std::max(cosine, static_cast<T>(-1.0)));
      ori_res = std::floor(ori_res * 10000) / 10000;
      Vector3 rpy_res_temp;
      rpy_res_temp.x() = ori_res <= (max_rpy.x() - min_rpy.x()) ? ori_res : 0.0;
      rpy_res_temp.y() = ori_res <= (max_rpy.y() - min_rpy.y()) ? ori_res : 0.0;
      rpy_res_temp.z() = ori_res <= (max_rpy.z() - min_rpy.z()) ? ori_res : 0.0;

      Vector3 max_rpypiece;
      if (i == max_level_) {
        max_rpypiece = max_rpy - min_rpy;
      } else {
        max_rpypiece.x() = ang_info_vec[i + 1].rpy_res.x() != 0.0 ? ang_info_vec[i + 1].rpy_res.x() : max_rpy.x() - min_rpy.x();
        max_rpypiece.y() = ang_info_vec[i + 1].rpy_res.y() != 0.0 ? ang_info_vec[i + 1].rpy_res.y() : max_rpy.y() - min_rpy.y();
        max_rpypiece.z() = ang_info_vec[i + 1].rpy_res.z() != 0.0 ? ang_info_vec[i + 1].rpy_res.z() : max_rpy.z() - min_rpy.z();
      }

      // Angle division number
      Eigen::Vector3i num_division;
      num_division.x() = rpy_res_temp.x() != 0.0 ? std::ceil(max_rpypiece.x() / rpy_res_temp.x()) : 1;
      num_division.y() = rpy_res_temp.y() != 0.0 ? std::ceil(max_rpypiece.y() / rpy_res_temp.y()) : 1;
      num_division.z() = rpy_res_temp.z() != 0.0 ? std::ceil(max_rpypiece.z() / rpy_res_temp.z()) : 1;
      ang_info_vec[i].num_division = num_division;

      // Bisect an angle
      ang_info_vec[i].rpy_res.x() = num_division.x() != 1 ? max_rpypiece.x() / num_division.x() : 0.0;
      ang_info_vec[i].rpy_res.y() = num_division.y() != 1 ? max_rpypiece.y() / num_division.y() : 0.0;
      ang_info_vec[i].rpy_res.z() = num_division.z() != 1 ? max_rpypiece.z() / num_division.z() : 0.0;

      ang_info_vec[i].min_rpy.x() = ang_info_vec[i].rpy_res.x() != 0.0 && ang_info_vec[i + 1].rpy_res.x() == 0.0 ? min_rpy.x() : 0.0;
      ang_info_vec[i].min_rpy.y() = ang_info_vec[i].rpy_res.y() != 0.0 && ang_info_vec[i + 1].rpy_res.y() == 0.0 ? min_rpy.y() : 0.0;
      ang_info_vec[i].min_rpy.z() = ang_info_vec[i].rpy_res.z() != 0.0 && ang_info_vec[i + 1].rpy_res.z() == 0.0 ? min_rpy.z() : 0.0;
    }
  }

private:
  T min_res_, max_res_;
  size_t max_level_;
  size_t v_rate_ = 2;
  size_t max_bucket_scan_count_ = 10;
  std::string voxelmaps_folder_name_ = "voxelmaps_coords";
  double success_rate_threshold_ = 0.999;
  std::pair<int, int> top_tx_range_, top_ty_range_, top_tz_range_;

  std::vector<Eigen::Vector3i> create_neighbor_coords(const Eigen::Vector3i& vec) {
    std::vector<Eigen::Vector3i> neighbor_coord;
    neighbor_coord.reserve(7);
    neighbor_coord.emplace_back(vec.x() - 1, vec.y() - 1, vec.z());
    neighbor_coord.emplace_back(vec.x() - 1, vec.y(), vec.z());
    neighbor_coord.emplace_back(vec.x(), vec.y() - 1, vec.z());
    neighbor_coord.emplace_back(vec.x() - 1, vec.y() - 1, vec.z() - 1);
    neighbor_coord.emplace_back(vec.x() - 1, vec.y(), vec.z() - 1);
    neighbor_coord.emplace_back(vec.x(), vec.y() - 1, vec.z() - 1);
    neighbor_coord.emplace_back(vec.x(), vec.y(), vec.z() - 1);
    return neighbor_coord;
  }

  Buckets create_hash_buckets(const UnorderedVoxelMap& unordered_voxelmap) {
    // Loop until the success rate exceeds the threshold
    std::vector<Eigen::Vector4i> buckets;
    for (int num_buckets = unordered_voxelmap.size(); num_buckets <= unordered_voxelmap.size() * 16; num_buckets *= 2) {
      buckets.resize(num_buckets);
      std::fill(buckets.begin(), buckets.end(), Eigen::Vector4i::Zero());

      int success_count = 0;
      for (const auto& voxel : unordered_voxelmap) {
        Eigen::Vector4i coord;
        coord << voxel.first.x(), voxel.first.y(), voxel.first.z(), 1;
        const std::uint32_t hash = hash::coord_to_hash(voxel.first);

        // Find empty bucket
        for (int i = 0; i < max_bucket_scan_count_; i++) {
          const std::uint32_t bucket_index = (hash + i) % num_buckets;

          if (buckets[bucket_index].w() == 0) {
            buckets[bucket_index] = coord;
            success_count++;
            break;
          }
        }
      }

      const double success_rate = static_cast<double>(success_count) / unordered_voxelmap.size();
      if (success_rate > success_rate_threshold_) {
        break;
      }
    }

    return buckets;
  }

  void calc_top_trans_range() {
    // The minimum and maximum x, y, z values are selected from the 3D coordinate vector.
    Eigen::Vector4i min_xyz = Eigen::Vector4i::Constant(std::numeric_limits<int>::max());
    Eigen::Vector4i max_xyz = Eigen::Vector4i::Constant(std::numeric_limits<int>::lowest());
    const auto& top_buckets = buckets_vec[max_level_];
    for (const auto& bucket : top_buckets) {
      min_xyz = min_xyz.cwiseMin(bucket);
      max_xyz = max_xyz.cwiseMax(bucket);
    }
    top_tx_range_ = std::make_pair(min_xyz.x(), max_xyz.x());
    top_ty_range_ = std::make_pair(min_xyz.y(), max_xyz.y());
    top_tz_range_ = std::make_pair(min_xyz.z(), max_xyz.z());
  }

  bool load_voxel_params(const std::string& voxelmaps_folder_path) {
    std::ifstream ifs(voxelmaps_folder_path + "/voxel_params.txt");
    if (!ifs) return false;
    std::string str;
    std::getline(ifs, str);
    min_res_ = std::stod(str.substr(str.find(" ") + 1));

    std::getline(ifs, str);
    max_level_ = std::stoi(str.substr(str.find(" ") + 1));

    std::getline(ifs, str);
    v_rate_ = (std::stod(str.substr(str.find(" ") + 1)));

    return true;
  }

  bool set_multi_buckets(const std::string& voxelmaps_folder_path) {
    auto pcd_files = pciof::find_point_cloud_files(voxelmaps_folder_path);
    if (pciof::can_convert_to_int(pcd_files)) {
      std::sort(pcd_files.begin(), pcd_files.end(), [](const std::string& a, const std::string& b) {
        return std::stoi(std::filesystem::path(a).stem().string()) < std::stoi(std::filesystem::path(b).stem().string());
      });
    } else {
      return false;
    }

    buckets_vec.resize(pcd_files.size());
    info_vec.resize(pcd_files.size());

    for (int i = 0; i < pcd_files.size(); i++) {
      const auto coords = pciof::read_pcd<int>(pcd_files[i]);

      if (coords.empty()) return false;

      std::vector<Eigen::Vector4i> buckets(coords.size());
      for (int j = 0; j < coords.size(); j++) {
        buckets[j] << coords[j], 1;
      }

      VoxelMapInfo<T> info;
      info.num_buckets = buckets.size();
      info.max_bucket_scan_count = max_bucket_scan_count_;
      info.res = min_res_ * std::pow(v_rate_, i);
      info.inv_res = 1.0 / info.res;

      buckets_vec[i] = buckets;
      info_vec[i] = info;
    }

    calc_top_trans_range();
    max_res_ = info_vec[max_level_].res;
    return true;
  }

  bool save_voxel_params(const std::string& folder_path) {
    std::filesystem::path dir(folder_path);
    if (!std::filesystem::exists(dir)) {
      std::cout << "[ERROR] Can not open folder" << std::endl;
      return false;
    }

    // create voxelmaps coords folder
    const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
    std::filesystem::path voxelmaps_coords_dir(voxelmaps_folder_path);
    if (!std::filesystem::exists(voxelmaps_coords_dir)) {
      std::filesystem::create_directory(voxelmaps_coords_dir);
    }

    std::ofstream ofs(voxelmaps_folder_path + "/voxel_params.txt");
    ofs << "min_res " << min_res_ << std::endl;
    ofs << "max_level " << max_level_ << std::endl;
    ofs << "v_rate " << v_rate_ << std::endl;
    ofs.close();

    return true;
  }

  bool save_voxelmaps_pcd(const std::string& folder_path) {
    std::filesystem::path dir(folder_path);
    if (!std::filesystem::exists(dir)) {
      std::cerr << "[ERROR] Can not open folder" << std::endl;
      return false;
    }

    // create voxelmaps coords folder
    const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
    std::filesystem::path voxelmaps_coords_dir(voxelmaps_folder_path);
    if (!std::filesystem::exists(voxelmaps_coords_dir)) {
      std::filesystem::create_directory(voxelmaps_coords_dir);
    }

    for (int i = 0; i < max_level_ + 1; ++i) {
      const std::string file_path = voxelmaps_folder_path + "/" + std::to_string(i) + ".pcd";
      const auto& coords = buckets_vec[i];
      if (coords.empty()) {
        std::cerr << "[ERROR] Can not get coords" << std::endl;
        return false;
      }

      std::vector<Eigen::Vector3i> coords3i(coords.size());
      for (int i = 0; i < coords.size(); ++i) {
        coords3i[i] = coords[i].head<3>();
      }

      if (!pciof::save_pcd<int>(file_path, coords3i)) {
        std::cerr << "[ERROR] Can not save pcd file: " << file_path << std::endl;
        return false;
      }
    }
    return true;
  }
};
}  // namespace cpu