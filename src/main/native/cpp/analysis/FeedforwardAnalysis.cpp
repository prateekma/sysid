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
  // This implements the OLS algorithm defined in
  // https://file.tavsys.net/control/sysid-ols.pdf.

  // Create a raw vector of doubles with our data in it.
  std::vector<double> olsData;
  olsData.reserve(4 *
                  (std::get<0>(data).size() + std::get<1>(data).size() - 2));

  // Record dts for later averaging
  std::vector<double> dts;

  // Iterate through the data and add it to our raw vector.
  auto PopulateVector = [&](const std::vector<PreparedData>& d) {
    for (size_t i = 0; i < d.size() - 1; ++i) {
      // Filter out dts from large gaps in the data (e.g., gaps between tests)
      double dt = d[i + 1].timestamp - d[i].timestamp;
      if (dt < 1.0) {
        dts.emplace_back(dt);
      }

      // x_k+1 = alpha x_k + beta u_k + gamma
      olsData.push_back(d[i + 1].velocity);
      olsData.push_back(d[i].velocity);
      olsData.push_back(d[i].voltage);
      olsData.push_back(std::copysign(1, d[i].velocity));
    }
  };
  PopulateVector(std::get<0>(data));
  PopulateVector(std::get<1>(data));

  auto ols = sysid::OLS(olsData, 3);

  double dt = std::accumulate(dts.begin(), dts.end(), 0.0) / dts.size();
  double alpha = std::get<0>(ols)[0];
  double beta = std::get<0>(ols)[1];
  double gamma = std::get<0>(ols)[2];

  return std::make_tuple(
      std::vector<double>{-gamma / beta, (1 - alpha) / beta,
                          dt * (alpha - 1) / beta / std::log(alpha)},
      std::get<1>(ols));
}
