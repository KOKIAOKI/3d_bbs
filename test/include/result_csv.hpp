#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <cmath>

struct ResultCsv {
private:
  std::ofstream csv_file;
  std::vector<double> times;
  std::vector<Eigen::Matrix4d> ground_truths;

public:
  ResultCsv(const std::string& folder_name, const std::string& gt_pose_file_name) {
    std::string file_path = folder_name + "/result.csv";
    csv_file.open(file_path);
    if (csv_file.is_open()) {
      csv_file << "file_name,bbs_gt_error_trans,bbs_gt_error_rot,bbs3d_x,bbs3d_y,bbs3d_z,bbs3d_q_x,bbs3d_q_y,bbs3d_q_z,bbs3d_q_w,"
                  "gicp_x,gicp_y,gicp_z,gicp_q_x,gicp_q_y,gicp_q_z,gicp_q_w,"
                  "gt_x,gt_y,gt_z,gt_q_x,gt_q_y,gt_q_z,gt_q_w\n";
    }

    if (gt_pose_file_name != "") {
      // set ground truth from csv data [sec, nsec, x, y, z, q_x, q_y, q_z, q_w].
      std::ifstream csv_file(gt_pose_file_name);
      if (!csv_file.is_open()) return;

      std::string line;
      while (std::getline(csv_file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::vector<double> values;

        while (std::getline(iss, token, ',')) {
          values.push_back(std::stod(token));
        }

        if (values.size() == 9) {                      // Ensure we have all 9 values
          double time = values[0] + values[1] * 1e-9;  // Convert sec and nsec to a single time value
          times.push_back(time);

          Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
          pose.block<3, 1>(0, 3) = Eigen::Vector3d(values[2], values[3], values[4]);
          Eigen::Quaterniond q(values[8], values[5], values[6], values[7]);  // w, x, y, z
          pose.block<3, 3>(0, 0) = q.toRotationMatrix();

          ground_truths.push_back(pose);
        }
      }
    }
  }

  ~ResultCsv() {
    if (csv_file.is_open()) {
      csv_file.close();
    }
  }

  void write(const std::string& file_name, const Eigen::Matrix4d& bbs3d_result, const Eigen::Matrix4d& gicp_result) {
    if (!csv_file.is_open()) return;

    // get nearest time ground truth. file name expressed as timestamp
    double timestamp = std::stod(file_name);
    auto it = std::lower_bound(times.begin(), times.end(), timestamp);
    size_t index = std::distance(times.begin(), it);
    if (index > 0 && index < times.size()) {
      if (timestamp - times[index - 1] < times[index] - timestamp) {
        index--;
      }
    }
    Eigen::Matrix4d ground_truth = ground_truths[index];

    // Calculate relative pose errors
    double bbs_gt_error_trans = (bbs3d_result.block<3, 1>(0, 3) - ground_truth.block<3, 1>(0, 3)).norm();
    double bbs_gt_error_rot = std::acos((bbs3d_result.block<3, 3>(0, 0).transpose() * ground_truth.block<3, 3>(0, 0)).trace() / 2 - 0.5);

    // Extract translations and quaternions
    auto extract_trans_quat = [](const Eigen::Matrix4d& mat) {
      Eigen::Quaterniond q(mat.block<3, 3>(0, 0));
      return std::make_tuple(mat(0, 3), mat(1, 3), mat(2, 3), q.x(), q.y(), q.z(), q.w());
    };

    auto [bbs3d_x, bbs3d_y, bbs3d_z, bbs3d_q_x, bbs3d_q_y, bbs3d_q_z, bbs3d_q_w] = extract_trans_quat(bbs3d_result);
    auto [gicp_x, gicp_y, gicp_z, gicp_q_x, gicp_q_y, gicp_q_z, gicp_q_w] = extract_trans_quat(gicp_result);
    auto [gt_x, gt_y, gt_z, gt_q_x, gt_q_y, gt_q_z, gt_q_w] = extract_trans_quat(ground_truth);

    csv_file << file_name << "," << bbs_gt_error_trans << "," << bbs_gt_error_rot << "," << bbs3d_x << "," << bbs3d_y << "," << bbs3d_z << ","
             << bbs3d_q_x << "," << bbs3d_q_y << "," << bbs3d_q_z << "," << bbs3d_q_w << "," << gicp_x << "," << gicp_y << "," << gicp_z << ","
             << gicp_q_x << "," << gicp_q_y << "," << gicp_q_z << "," << gicp_q_w << "," << gt_x << "," << gt_y << "," << gt_z << "," << gt_q_x << ","
             << gt_q_y << "," << gt_q_z << "," << gt_q_w << "\n";
  }
};