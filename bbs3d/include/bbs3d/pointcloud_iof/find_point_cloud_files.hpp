#pragma once
#include <filesystem>
#include <iostream>
#include <vector>

namespace pciof {
inline bool can_convert_to_int(const std::vector<std::string>& name_vec) {
  for (const auto& name : name_vec) {
    try {
      std::stoi(std::filesystem::path(name).stem().string());
    } catch (const std::invalid_argument& e) {
      return false;
    } catch (const std::out_of_range& e) {
      return false;
    }
  }
  return true;
}

inline bool can_convert_to_double(const std::vector<std::string>& name_vec) {
  for (const auto& name : name_vec) {
    try {
      std::stod(std::filesystem::path(name).stem().string());
    } catch (const std::invalid_argument& e) {
      return false;
    } catch (const std::out_of_range& e) {
      return false;
    }
  }
  return true;
}

inline std::vector<std::string> find_point_cloud_files(const std::string& path) {
  std::filesystem::path dir(path);
  std::vector<std::string> files;

  if (!std::filesystem::exists(dir)) {
    std::cout << "[ERROR] Cannot open folder" << std::endl;
    return files;
  }

  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    const std::string extension = entry.path().extension().string();
    if (extension == ".pcd") {
      files.emplace_back(entry.path().string());
    }
  }

  return files;
}
}  // namespace pciof