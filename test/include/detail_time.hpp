#pragma once

#include <vector>
#include <numeric>
#include <iostream>

struct MeanSTD {
  std::vector<double> time_vec;

  void print() {
    double mean = std::accumulate(time_vec.begin(), time_vec.end(), 0.0) / time_vec.size();

    double std = std::sqrt(std::inner_product(time_vec.begin(), time_vec.end(), time_vec.begin(), 0.0) / time_vec.size() - mean * mean);

    std::cout << "mean: " << mean << ", std: " << std << std::endl;
  }
};
