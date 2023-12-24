#include <gpu_bbs3d/bbs3d.cuh>
#include <gpu_bbs3d/voxelmaps.cuh>
#include <pointcloud_iof/io.hpp>
#include <pointcloud_iof/load.hpp>

#include <boost/filesystem.hpp>

namespace gpu {

bool BBS3D::set_voxelmaps_coords(const std::string& folder_path) {
  const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
  if (!boost::filesystem::exists(voxelmaps_folder_path)) {
    return false;
  }

  if (!load_voxel_params(voxelmaps_folder_path)) return false;

  const auto buckets = load_buckets(voxelmaps_folder_path);
  if (buckets.empty()) return false;

  voxelmaps_ptr_->set_buckets_on_device(buckets, stream);

  return true;
}

bool BBS3D::load_voxel_params(const std::string& voxelmaps_folder_path) {
  std::ifstream ifs(voxelmaps_folder_path + "/voxel_params.txt");
  if (!ifs) return false;
  std::string str;
  std::getline(ifs, str);
  const float min_level_res = std::stof(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  const int max_level = std::stoi(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  const float v_rate = std::stof(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  const int min_x = std::stoi(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  const int max_x = std::stoi(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  const int min_y = std::stoi(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  const int max_y = std::stoi(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  const int min_z = std::stoi(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  const int max_z = std::stoi(str.substr(str.find(" ") + 1));

  voxelmaps_ptr_.reset(new VoxelMaps);
  voxelmaps_ptr_->set_min_res(min_level_res);
  voxelmaps_ptr_->set_max_level(max_level);
  voxelmaps_ptr_->set_voxel_expantion_rate(v_rate);

  init_tx_range_ = std::make_pair(min_x, max_x);
  init_ty_range_ = std::make_pair(min_y, max_y);
  init_tz_range_ = std::make_pair(min_z, max_z);

  return true;
}

std::vector<std::vector<Eigen::Vector4i>> BBS3D::load_buckets(std::string voxelmaps_folder_path) {
  const auto pcd_files = pciof::load_pcd_file_paths(voxelmaps_folder_path);

  std::vector<std::vector<Eigen::Vector4i>> buckets;
  buckets.reserve(pcd_files.size());

  for (const auto& file : pcd_files) {
    const auto coords = pciof::read_pcd<int>(file.first);
    if (coords.empty()) return {};
    std::vector<Eigen::Vector4i> coords4i(coords.size());
    for (int i = 0; i < coords.size(); ++i) {
      coords4i[i] << coords[i], 1;
    }

    buckets.emplace_back(coords4i);
  }

  return buckets;
}

bool BBS3D::save_voxel_params(const std::string& folder_path) {
  boost::filesystem::path dir(folder_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open folder" << std::endl;
    return false;
  }

  float min_level_res = voxelmaps_ptr_->get_min_res();
  int max_level = voxelmaps_ptr_->get_max_level();
  float v_rate = voxelmaps_ptr_->get_voxel_expantion_rate();

  // create voxelmaps coords folder
  const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
  boost::filesystem::path voxelmaps_coords_dir(voxelmaps_folder_path);
  if (!boost::filesystem::exists(voxelmaps_coords_dir)) {
    boost::filesystem::create_directory(voxelmaps_coords_dir);
  }

  std::ofstream ofs(voxelmaps_folder_path + "/voxel_params.txt");
  ofs << "min_level_res " << min_level_res << std::endl;
  ofs << "max_level " << max_level << std::endl;
  ofs << "v_rate " << v_rate << std::endl;
  ofs << "min_x " << init_tx_range_.first << std::endl;
  ofs << "max_x " << init_tx_range_.second << std::endl;
  ofs << "max_y " << init_ty_range_.first << std::endl;
  ofs << "max_y " << init_ty_range_.second << std::endl;
  ofs << "max_z " << init_tz_range_.first << std::endl;
  ofs << "max_z " << init_tz_range_.second << std::endl;

  ofs.close();

  return true;
}

bool BBS3D::save_voxelmaps_pcd(const std::string& folder_path) {
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

  for (int i = 0; i < voxelmaps_ptr_->get_max_level() + 1; ++i) {
    const std::string file_path = voxelmaps_folder_path + "/" + std::to_string(i) + ".pcd";
    const auto coords = voxelmaps_ptr_->multi_buckets_[i];
    if (coords.empty()) {
      std::cout << "[ERROR] Can not get coords" << std::endl;
      return false;
    }

    std::vector<Eigen::Vector3i> coords3i;
    coords3i.reserve(coords.size());
    for (const auto& coord : coords) {
      coords3i.emplace_back(coord.head<3>());
    }

    if (!pciof::save_pcd<int>(file_path, coords3i)) {
      std::cout << "[ERROR] Can not save pcd file: " << file_path << std::endl;
      return false;
    }
  }
  return true;
}
}  // namespace gpu