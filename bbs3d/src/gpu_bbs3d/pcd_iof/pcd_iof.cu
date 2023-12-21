#include <gpu_bbs3d/bbs3d.cuh>
#include <gpu_bbs3d/voxelmaps.cuh>
#include <pcd_iof/io.hpp>

#include <boost/filesystem.hpp>

namespace gpu {

void BBS3D::load_voxel_params(const std::string& folder_path) {
  std::ifstream ifs(folder_path + "/voxel_params.txt");
  if (!ifs) {
    std::cout << "[ERROR] Can not open file: " << folder_path + "/voxel_params.txt" << std::endl;
    return;  // output error
  }

  std::string str;
  std::getline(ifs, str);
  const float min_level_res = std::stof(str.substr(str.find(" ") + 1));
  const int max_level = std::stoi(str.substr(str.find(" ") + 1));
  const float v_rate = std::stof(str.substr(str.find(" ") + 1));
  const int min_x = std::stoi(str.substr(str.find(" ") + 1));
  const int max_x = std::stoi(str.substr(str.find(" ") + 1));
  const int min_y = std::stoi(str.substr(str.find(" ") + 1));
  const int max_y = std::stoi(str.substr(str.find(" ") + 1));
  const int min_z = std::stoi(str.substr(str.find(" ") + 1));
  const int max_z = std::stoi(str.substr(str.find(" ") + 1));

  voxelmaps_ptr_.reset(new VoxelMaps);
  voxelmaps_ptr_->set_min_res(min_level_res);
  voxelmaps_ptr_->set_max_level(max_level);
  voxelmaps_ptr_->set_voxel_expantion_rate(v_rate);

  init_tx_range_ = std::make_pair(min_x, max_x);
  init_ty_range_ = std::make_pair(min_y, max_y);
  init_tz_range_ = std::make_pair(min_z, max_z);
}

std::vector<std::vector<Eigen::Vector4i>> BBS3D::load_buckets(std::string folder_path) {
  const auto pcd_files = load_pcd_files(folder_path);

  std::vector<std::vector<Eigen::Vector4i>> buckets;
  buckets.reserve(pcd_files.size());

  for (const auto& file : pcd_files) {
    const auto coords = pciof::read_pcd<int>(file.first);
    if (coords.empty()) {
      std::cout << "[ERROR] Can not load pcd file: " << file.first << std::endl;
      return {};  // output error
    }

    std::vector<Eigen::Vector4i> coords4i(coords.size());
    for (int i = 0; i < coords.size(); ++i) {
      coords4i[i] << coords[i], 1;
    }

    buckets.emplace_back(coords4i);
  }

  return buckets;
}

std::vector<std::pair<std::string, std::string>> BBS3D::load_pcd_files(const std::string& folder_path) {
  boost::filesystem::path dir(folder_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open folder" << std::endl;
    return {};  // output error
  }

  std::vector<std::pair<std::string, std::string>> pcd_files;
  for (const auto& file : boost::filesystem::directory_iterator(folder_path)) {
    const std::string extension = file.path().extension().string();
    if (extension != ".pcd" && extension != ".PCD") {
      continue;
    }
    pcd_files.emplace_back(file.path().string(), file.path().stem().string());
  }

  std::sort(pcd_files.begin(), pcd_files.end(), [](const std::pair<std::string, std::string>& a, const std::pair<std::string, std::string>& b) {
    return a.second < b.second;
  });

  return pcd_files;
}

void BBS3D::save_voxel_params(const std::string& folder_path) {
  float min_level_res = voxelmaps_ptr_->get_min_res();
  int max_level = voxelmaps_ptr_->get_max_level();
  float v_rate = voxelmaps_ptr_->get_voxel_expantion_rate();
  const auto trans_search_range = get_trans_search_range();

  std::ofstream ofs(folder_path + "/voxel_params.txt");
  ofs << "min_level_res " << min_level_res << std::endl;
  ofs << "max_level " << max_level << std::endl;
  ofs << "v_rate " << v_rate << std::endl;
  ofs << "min_x " << trans_search_range[0].first << std::endl;
  ofs << "max_x " << trans_search_range[0].second << std::endl;
  ofs << "max_y " << trans_search_range[1].first << std::endl;
  ofs << "max_y " << trans_search_range[1].second << std::endl;
  ofs << "max_z " << trans_search_range[2].first << std::endl;
  ofs << "max_z " << trans_search_range[2].second << std::endl;
}

void BBS3D::save_voxelmaps_pcd(const std::string& folder_path) {
  boost::filesystem::path dir(folder_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open folder" << std::endl;
    return;  // output error
  }

  for (int i = 0; i < voxelmaps_ptr_->get_max_level(); ++i) {
    const std::string file_path = folder_path + "/" + std::to_string(i) + ".pcd";
    const auto coords = voxelmaps_ptr_->multi_buckets_[i];
    if (coords.empty()) {
      std::cout << "[ERROR] Can not get coords" << std::endl;
      return;  // output error
    }

    std::vector<Eigen::Vector3i> coords3i;
    coords3i.reserve(coords.size());
    for (const auto& coord : coords) {
      coords3i.emplace_back(coord.head<3>());
    }

    if (!pciof::save_pcd<int>(file_path, coords3i)) {
      std::cout << "[ERROR] Can not save pcd file: " << file_path << std::endl;
      return;  // output error
    }
  }
}
}  // namespace gpu