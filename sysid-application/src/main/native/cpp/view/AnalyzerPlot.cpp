// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/AnalyzerPlot.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <imgui.h>
#include <implot.h>
#include <units/math.h>
#include <units/time.h>
#include <wpi/StringExtras.h>
#include <wpi/array.h>

#include "sysid/Util.h"
#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/ArmSim.h"
#include "sysid/analysis/ElevatorSim.h"
#include "sysid/analysis/FilteringUtils.h"
#include "sysid/analysis/SimpleMotorSim.h"

using namespace sysid;

static ImPlotPoint Getter(void* data, int idx) {
  return static_cast<ImPlotPoint*>(data)[idx];
}

static double simSquaredErrorSum = 0.0;
static int timeSeriesPoints = 0;

template <typename Model>
static std::shared_ptr<std::vector<std::vector<ImPlotPoint>>>
PopulateTimeDomainSim(const std::vector<PreparedData>& data,
                      const std::array<units::second_t, 4>& startTimes,
                      size_t step, Model model) {
  // Create the vector of ImPlotPoints that will contain our simulated data.
  auto ptr = std::make_shared<std::vector<std::vector<ImPlotPoint>>>();
  auto& pts = *ptr;
  std::vector<ImPlotPoint> tmp;

  auto startTime = data[0].timestamp;

  tmp.emplace_back(startTime.to<double>(), data[0].velocity);

  model.Reset(data[0].position, data[0].velocity);
  units::second_t t = 0_s;

  for (size_t i = 1; i < data.size(); ++i) {
    const auto& now = data[i];
    const auto& pre = data[i - 1];

    t += now.timestamp - pre.timestamp;

    // If the current time stamp and previous time stamp are across a test's
    // start timestamp, it is the start of a new test and the model needs to be
    // reset.
    if (std::find(startTimes.begin(), startTimes.end(), now.timestamp) !=
        startTimes.end()) {
      pts.emplace_back(std::move(tmp));
      model.Reset(now.position, now.velocity);
      continue;
    }

    model.Update(units::volt_t{pre.voltage}, pre.dt);
    tmp.emplace_back((startTime + t).to<double>(), model.GetVelocity());
    simSquaredErrorSum += std::pow(now.velocity - model.GetVelocity(), 2);
    timeSeriesPoints++;
  }

  pts.emplace_back(std::move(tmp));
  return ptr;
}

AnalyzerPlot::AnalyzerPlot(wpi::Logger& logger) : m_logger{logger} {
  // Create titles for all charts.
  m_charts[0].title = "Quasistatic Velocity vs. Velocity-Portion Voltage";
  m_charts[1].title = "Dynamic Acceleration vs. Acceleration-Portion Voltage";
  m_charts[2].title = "Quasistatic Velocity vs. Time";
  m_charts[3].title = "Quasistatic Acceleration vs. Time";
  m_charts[4].title = "Dynamic Velocity vs. Time";
  m_charts[5].title = "Dynamic Acceleration vs. Time";
  m_charts[6].title = "Timesteps vs. Time";

  // Pre-allocate filtered data vector with max size. Raw data is not used for
  // every chart so they can be initialized on a case-by-case basis.
  for (auto&& chart : m_charts) {
    chart.filteredData.reserve(kMaxSize);
  }
}

void AnalyzerPlot::SetData(const Storage& rawData, const Storage& filteredData,
                           const std::string& unit,
                           const std::vector<double>& ffGains,
                           const std::array<units::second_t, 4>& startTimes,
                           AnalysisType type, std::atomic<bool>& abort) {
  // Lock mutex to ensure that we don't run multiple of these at once.
  std::scoped_lock lock{m_mutex};

  // Reference to charts for convenience.
  auto& ci = m_charts;

  // Perform some basic cleanup on the charts -- clear vectors, and reset
  // arbitrary storage.
  for (auto&& chart : m_charts) {
    chart.data.reset();
    chart.filteredData.clear();
    chart.rawData.clear();
  }

  // Get abbreviation for unit.
  m_abbreviation = GetAbbreviation(unit);

  // Obtain references to data and feedforward gains for convenience.
  auto& [slow, fast] = filteredData;
  auto& [rawSlow, rawFast] = rawData;
  const auto& Ks = ffGains[0];
  const auto& Kv = ffGains[1];
  const auto& Ka = ffGains[2];

  // Reset parameters for RMSE calculation.
  simSquaredErrorSum = 0;
  timeSeriesPoints = 0;

  // Calculate step sizes to ensure that we only use the memory that we
  // allocated.
  auto slowStep = std::ceil(slow.size() * 1.0 / kMaxSize * 4);
  auto fastStep = std::ceil(fast.size() * 1.0 / kMaxSize * 4);
  auto rawSlowStep = std::ceil(rawSlow.size() * 1.0 / kMaxSize * 4);
  auto rawFastStep = std::ceil(rawFast.size() * 1.0 / kMaxSize * 4);

  // Calculate min and max velocities and accelerations of the slow and fast
  // datasets respectively.
  auto slowMinElement =
      std::min_element(slow.cbegin(), slow.cend(), [](auto& a, auto& b) {
        return a.velocity < b.velocity;
      })->velocity;

  auto slowMaxElement =
      std::max_element(slow.cbegin(), slow.cend(), [](auto& a, auto& b) {
        return a.velocity < b.velocity;
      })->velocity;

  auto fastMinElement =
      std::min_element(fast.cbegin(), fast.cend(), [](auto& a, auto& b) {
        return a.acceleration < b.acceleration;
      })->acceleration;

  auto fastMaxElement =
      std::max_element(fast.cbegin(), fast.cend(), [](auto& a, auto& b) {
        return a.acceleration < b.acceleration;
      })->acceleration;

  // Calculate mean time delta.
  units::second_t dtMean = GetMeanTimeDelta(filteredData);

  // Calculate points to show the line of best fit.
  ci[0].data = std::make_shared<wpi::array<ImPlotPoint, 2>>(
      ImPlotPoint{Kv * slowMinElement, slowMinElement},
      ImPlotPoint{Kv * slowMaxElement, slowMaxElement});

  ci[1].data = std::make_shared<wpi::array<ImPlotPoint, 2>>(
      ImPlotPoint{Ka * fastMinElement, fastMinElement},
      ImPlotPoint{Ka * fastMaxElement, fastMaxElement});

  // Populate quasistatic time-domain graphs and quasistatic velocity vs.
  // velocity-portion voltage graph.
  for (size_t i = 0; i < slow.size(); i += slowStep) {
    // Abort setting data based on the atomic value.
    if (abort) {
      return;
    }

    // Calculate portion of voltage that corresponds to change in velocity.
    double Vportion = slow[i].voltage - std::copysign(Ks, slow[i].velocity) -
                      Ka * slow[i].acceleration;

    if (type == analysis::kElevator) {
      const auto& Kg = ffGains[3];
      Vportion -= Kg;
    } else if (type == analysis::kArm) {
      const auto& Kcos = ffGains[3];
      Vportion -= Kcos * slow[i].cos;
    }

    // Append data to relevant charts.
    ci[0].filteredData.emplace_back(Vportion, slow[i].velocity);
    ci[2].filteredData.emplace_back(slow[i].timestamp.to<double>(),
                                    slow[i].velocity);
    ci[3].filteredData.emplace_back(slow[i].timestamp.to<double>(),
                                    slow[i].acceleration);

    if (i > 0) {
      // If the current timestamp is not in the startTimes array, it is the
      // during a test and should be included. If it is in the startTimes array,
      // it is the beginning of a new test and the dt will be inflated.
      // Therefore we skip those to exclude that dt and effectively reset dt
      // calculations.
      if (slow[i].dt > 0_s &&
          std::find(startTimes.begin(), startTimes.end(), slow[i].timestamp) ==
              startTimes.end()) {
        ci[6].filteredData.emplace_back(
            slow[i].timestamp.to<double>(),
            units::millisecond_t{slow[i].dt}.to<double>());
      }
    }
  }

  // Populate dynamic time-domain graphs and dynamic acceleration vs.
  // acceleration-portion voltage graph.
  for (size_t i = 0; i < fast.size(); i += fastStep) {
    // Abort setting data based on the atomic value.
    if (abort) {
      return;
    }

    // Calculate portion of voltage that corresponds to change in acceleration.
    double Vportion = fast[i].voltage - std::copysign(Ks, fast[i].velocity) -
                      Kv * fast[i].velocity;

    if (type == analysis::kElevator) {
      const auto& Kg = ffGains[3];
      Vportion -= Kg;
    } else if (type == analysis::kArm) {
      const auto& Kcos = ffGains[3];
      Vportion -= Kcos * fast[i].cos;
    }

    // Append data to relevant charts.
    ci[1].filteredData.emplace_back(Vportion, fast[i].acceleration);
    ci[4].filteredData.emplace_back(fast[i].timestamp.to<double>(),
                                    fast[i].velocity);
    ci[5].filteredData.emplace_back(fast[i].timestamp.to<double>(),
                                    fast[i].acceleration);

    if (i > 0) {
      // If the current timestamp is not in the startTimes array, it is the
      // during a test and should be included. If it is in the startTimes array,
      // it is the beginning of a new test and the dt will be inflated.
      // Therefore we skip those to exclude that dt and effectively reset dt
      // calculations.
      if (fast[i].dt > 0_s &&
          std::find(startTimes.begin(), startTimes.end(), fast[i].timestamp) ==
              startTimes.end()) {
        ci[6].filteredData.emplace_back(
            fast[i].timestamp.to<double>(),
            units::millisecond_t{fast[i].dt}.to<double>());
      }
    }
  }

  auto minTime =
      units::math::min(slow.front().timestamp, fast.front().timestamp);
  auto maxTime = units::math::max(slow.back().timestamp, fast.back().timestamp);

  // Add first and last recorded timestamp to mean.
  ci[6].data = std::make_shared<wpi::array<ImPlotPoint, 2>>(
      ImPlotPoint{minTime.to<double>(),
                  units::millisecond_t{dtMean}.to<double>()},
      ImPlotPoint{maxTime.to<double>(),
                  units::millisecond_t{dtMean}.to<double>()});

  // Populate raw slow time-domain data.
  for (size_t i = 0; i < rawSlow.size(); i += rawSlowStep) {
    ci[2].rawData.emplace_back(rawSlow[i].timestamp.to<double>(),
                               rawSlow[i].velocity);
    ci[3].rawData.emplace_back(rawSlow[i].timestamp.to<double>(),
                               rawSlow[i].acceleration);
  }

  // Populate raw fast time-domain data.
  for (size_t i = 0; i < rawFast.size(); i += rawFastStep) {
    ci[4].rawData.emplace_back(rawFast[i].timestamp.to<double>(),
                               rawFast[i].velocity);
    ci[5].rawData.emplace_back(rawFast[i].timestamp.to<double>(),
                               rawFast[i].acceleration);
  }

  // Populate simulated time-domain data.
  auto& quasistatic = ci[2].data;
  auto& dynamic = ci[4].data;

  if (type == analysis::kElevator) {
    const auto& Kg = ffGains[3];
    quasistatic = PopulateTimeDomainSim(rawSlow, startTimes, fastStep,
                                        sysid::ElevatorSim{Ks, Kv, Ka, Kg});
    dynamic = PopulateTimeDomainSim(rawFast, startTimes, fastStep,
                                    sysid::ElevatorSim{Ks, Kv, Ka, Kg});
  } else if (type == analysis::kArm) {
    const auto& Kcos = ffGains[3];
    quasistatic = PopulateTimeDomainSim(rawSlow, startTimes, fastStep,
                                        sysid::ArmSim{Ks, Kv, Ka, Kcos});
    dynamic = PopulateTimeDomainSim(rawFast, startTimes, fastStep,
                                    sysid::ArmSim{Ks, Kv, Ka, Kcos});
  } else {
    quasistatic = PopulateTimeDomainSim(rawSlow, startTimes, fastStep,
                                        sysid::SimpleMotorSim{Ks, Kv, Ka});
    dynamic = PopulateTimeDomainSim(rawFast, startTimes, fastStep,
                                    sysid::SimpleMotorSim{Ks, Kv, Ka});
  }

  // RMSE = std::sqrt(sum((x_i - x^_i)^2) / N) where sum represents the sum of
  // all time series points, x_i represents the velocity at a timestep, x^_i
  // represents the prediction at the timestep, and N represents the number of
  // points
  m_RMSE = std::sqrt(simSquaredErrorSum / timeSeriesPoints);
}

static void PlotSimData(std::vector<std::vector<ImPlotPoint>>& data) {
  for (auto&& pts : data) {
    ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 1.5);
    ImPlot::PlotLineG("Simulation", Getter, pts.data(), pts.size());
  }
}

bool AnalyzerPlot::Plot(bool grid) {
  // Check whether plots are still loading; display message if appropriate.
  std::unique_lock lock{m_mutex, std::defer_lock};
  if (!lock.try_lock()) {
    ImGui::Text("Loading %c",
                "|/-\\"[static_cast<int>(ImGui::GetTime() / 0.05f) & 3]);
    return false;
  }

  // Parameters that need to be adjusted based on whether we want a grid.
  int rows, cols, flags;
  if (grid) {
    rows = 2;
    cols = 4;
    flags = ImPlotSubplotFlags_ShareItems;
  } else {
    rows = 7;
    cols = 1;
    flags = ImPlotSubplotFlags_None;
  }

  // Use subplots to display all plots in a grid.
  static constexpr std::array<size_t, 7> kOrder{2, 3, 4, 5, 0, 1, 6};
  if (ImPlot::BeginSubplots("##Combined Diagnostics", rows, cols,
                            ImVec2(-1, kCombinedPlotSize * rows), flags)) {
    for (auto i : kOrder) {
      // Get the chart.
      auto& chart = m_charts[i];

      // Create axis labels.
      auto axes = wpi::split(chart.title, " vs. ");
      auto x = fmt::format("{} ({})", axes.second, i > 1 ? "s" : "V");

      std::string y{axes.first};
      if (i < 6) {
        if (i % 2 == 0) {
          y = fmt::format("{} ({} / s)", y, m_abbreviation);
        } else {
          y = fmt::format("{} ({} / s^2)", y, m_abbreviation);
        }
      } else {
        y += " (ms)";
      }

      // Create plot.
      if (ImPlot::BeginPlot(chart.title.c_str(), x.c_str(), y.c_str(),
                            ImVec2(-1, 0), ImPlotFlags_None,
                            ImPlotAxisFlags_NoGridLines)) {
        // Get a reference to the data that we are plotting.
        auto& rawData = chart.rawData;
        auto& data = chart.filteredData;

        // Plot filtered data (common for all plots).
        ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);
        ImPlot::PlotScatterG("Filtered Data", Getter, data.data(), data.size());

        // Plot items specific to each plot.
        switch (i) {
          // Quasistatic Velocity vs. Velocity-Portion Voltage
          // Dynamic Acceleration vs. Acceleration-Portion Voltage
          // Timesteps vs. Time
          case 0:
          case 1:
          case 6:
            ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 1.5);
            ImPlot::PlotLineG(
                "Fit", Getter,
                chart.GetStorage<wpi::array<ImPlotPoint, 2>>()->data(), 2);
            break;

          // Quasistatic Velocity vs. Time
          // Quasistatic Acceleration vs. Time
          // Dynamic Velocity vs. Time
          // Dynamic Acceleration vs. Time
          case 2:
          case 3:
          case 4:
          case 5:
            // Plot raw data.
            ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);
            ImPlot::PlotScatterG("Raw Data", Getter, rawData.data(),
                                 rawData.size());
            // Plot simulation.
            if (i % 2 == 0) {
              auto st =
                  chart.GetStorage<std::vector<std::vector<ImPlotPoint>>>();
              PlotSimData(*st);
            }
            break;
        }

        // End plot.
        ImPlot::EndPlot();
      }
    }
    ImPlot::EndSubplots();
  }
  return true;
}
