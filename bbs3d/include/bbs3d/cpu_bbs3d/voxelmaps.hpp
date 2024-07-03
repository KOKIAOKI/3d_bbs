#pragma once
#include "bbs3d/pointcloud_iof/pcd_loader_without_pcl.hpp"
#include "bbs3d/hash/hash.hpp"
#include <unordered_map>
#include <Eigen/Dense>
#include <boost/functional/hash/hash.hpp>
#include <boost/filesystem.hpp>

namespace cpu {
template <typename T>
struct VoxelMapInfo {
  int num_buckets;
  int max_bucket_scan_count;
  T res;      // voxel resolution
  T inv_res;  // inverse of voxel resolution
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

public:
  using Ptr = std::shared_ptr<VoxelMaps>;
  using ConstPtr = std::shared_ptr<const VoxelMaps>;

  using UnorderedVoxelMap = std::unordered_map<Eigen::Vector3i, int, VectorHash, VctorEqual>;
  using Buckets = std::vector<Eigen::Vector4i>;

  // public member variables
  std::vector<Buckets> buckets_vec_;
  std::vector<VoxelMapInfo<T>> info_vec_;

  // get member variables
  T min_res() const { return min_res_; }
  size_t max_level() const { return max_level_; }
  size_t v_rate() const { return v_rate_; }
  size_t max_bucket_scan_count() const { return max_bucket_scan_count_; }
  std::string voxelmaps_folder_name() const { return voxelmaps_folder_name_; }

  // constructor
  // constructor 1: create voxelmaps from points
  VoxelMaps() : v_rate_(2), max_bucket_scan_count_(10), success_rate_threshold_(0.999), voxelmaps_folder_name_("voxelmaps_coords") {}

  // constructor 2: create voxelmaps from points
  VoxelMaps(const std::vector<Vector3>& points, T min_res, size_t max_level)
  : min_res_(min_res),
    max_level_(max_level),
    v_rate_(2),
    max_bucket_scan_count_(10),
    success_rate_threshold_(0.999),
    voxelmaps_folder_name_("voxelmaps_coords") {
    create_voxelmaps(points);
  }

  // constructor 3: If cannot load voxelmap coords, create voxelmaps from points and save them
  VoxelMaps(const std::string& path, T min_res, size_t max_level, bool overwrite = false)
  : min_res_(min_res),
    max_level_(max_level),
    v_rate_(2),
    max_bucket_scan_count_(10),
    success_rate_threshold_(0.999),
    voxelmaps_folder_name_("voxelmaps_coords") {
    if (!set_voxelmaps_coords(path) || overwrite) {
      std::vector<Vector3> tar_points;
      pciof::load_tar_points<T>(path, 0.0, tar_points);
      create_voxelmaps(tar_points);

      if (!save_voxelmaps(path)) exit(1);
    }
  }

  ~VoxelMaps() {}

  // voxelmap coords IO
  bool set_voxelmaps_coords(const std::string& folder_path) {
    const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
    if (!boost::filesystem::exists(voxelmaps_folder_path)) {
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

private:
  T min_res_;
  size_t max_level_;
  size_t v_rate_;
  size_t max_bucket_scan_count_;
  std::string voxelmaps_folder_name_;
  double success_rate_threshold_;

  void create_voxelmaps(const std::vector<Vector3>& points) {
    const int buckets_vec_size = max_level_ + 1;
    buckets_vec_.resize(buckets_vec_size);
    info_vec_.resize(buckets_vec_size);

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

      buckets_vec_[i] = buckets;
      info_vec_[i] = info;
    }
  }

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
    const auto pcd_files = pciof::load_pcd_file_paths(voxelmaps_folder_path);
    buckets_vec_.resize(pcd_files.size());
    info_vec_.resize(pcd_files.size());

    for (int i = 0; i < pcd_files.size(); i++) {
      const auto& file = pcd_files[i];
      const auto coords = pciof::read_pcd<int>(file.first);

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

      buckets_vec_[i] = buckets;
      info_vec_[i] = info;
    }

    return true;
  }

  bool save_voxel_params(const std::string& folder_path) {
    boost::filesystem::path dir(folder_path);
    if (!boost::filesystem::exists(dir)) {
      std::cout << "[ERROR] Can not open folder" << std::endl;
      return false;
    }

    // create voxelmaps coords folder
    const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
    boost::filesystem::path voxelmaps_coords_dir(voxelmaps_folder_path);
    if (!boost::filesystem::exists(voxelmaps_coords_dir)) {
      boost::filesystem::create_directory(voxelmaps_coords_dir);
    }

    std::ofstream ofs(voxelmaps_folder_path + "/voxel_params.txt");
    ofs << "min_res " << min_res_ << std::endl;
    ofs << "max_level " << max_level_ << std::endl;
    ofs << "v_rate " << v_rate_ << std::endl;
    ofs.close();

    return true;
  }

  bool save_voxelmaps_pcd(const std::string& folder_path) {
    boost::filesystem::path dir(folder_path);
    if (!boost::filesystem::exists(dir)) {
      std::cerr << "[ERROR] Can not open folder" << std::endl;
      return false;
    }

    // create voxelmaps coords folder
    const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
    boost::filesystem::path voxelmaps_coords_dir(voxelmaps_folder_path);
    if (!boost::filesystem::exists(voxelmaps_coords_dir)) {
      boost::filesystem::create_directory(voxelmaps_coords_dir);
    }

    for (int i = 0; i < max_level_ + 1; ++i) {
      const std::string file_path = voxelmaps_folder_path + "/" + std::to_string(i) + ".pcd";
      const auto& coords = buckets_vec_[i];
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