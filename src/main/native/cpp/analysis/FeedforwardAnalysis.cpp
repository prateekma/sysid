// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/FeedforwardAnalysis.h"
#include <cmath>
#include <numeric>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/OLS.h"

using namespace sysid;

std::tuple<std::vector<double>, double> sysid::CalculateFeedforwardGains(
    const Storage& data, const AnalysisType& type) {
  // Create a raw vector of doubles with our data in it.
  std::vector<double> olsData;
  olsData.reserve(4 *
                  (std::get<0>(data).size() + std::get<1>(data).size() - 2));

  // Iterate through the data and add it to our raw vector.
  auto PopulateVector = [&](const std::vector<PreparedData>& d) {
    for (size_t i = 0; i < d.size() - 1; ++i) {
      olsData.push_back(d[i + 1].velocity);
      olsData.push_back(std::copysign(1, d[i].velocity));
      olsData.push_back(d[i].voltage);
      olsData.push_back(d[i].velocity);
    }
  };
  PopulateVector(std::get<0>(data));
  PopulateVector(std::get<1>(data));

  auto ols = sysid::OLS(olsData, 3);

  double gamma = std::get<0>(ols)[0];
  double beta = std::get<0>(ols)[1];
  double alpha = std::get<0>(ols)[2];

  // Calculate the mean dt.
  const auto& d = std::get<0>(data);
  auto dt = (d.back().timestamp - d.front().timestamp) / d.size();
  wpi::outs() << dt << "\n";
  wpi::outs().flush();

  return std::make_tuple(
      std::vector<double>{-gamma / beta, (1 - alpha) / beta,
                          dt * (alpha - 1) / beta / std::log(alpha)},
      std::get<1>(ols));
}
