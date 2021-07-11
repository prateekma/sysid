// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <imgui.h>
#include <implot.h>
#include <units/time.h>
#include <units/voltage.h>
#include <wpi/Logger.h>
#include <wpi/StringMap.h>
#include <wpi/spinlock.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/FeedforwardAnalysis.h"

namespace sysid {
/**
 * Class that helps with plotting data in the analyzer view.
 */
class AnalyzerPlot {
 public:
  // Size of plots in the combined view.
  static constexpr int kCombinedPlotSize = 300;

  /**
   * Constructs an instance of the analyzer plot helper and allocates memory for
   * all data vectors.
   */
  explicit AnalyzerPlot(wpi::Logger& logger);

  /**
   * Sets the raw data to be displayed on the plots.
   *
   * @param rawData      Raw data storage.
   * @param filteredData Filtered data storage.
   * @param ffGains      List of feedforward gains (Ks, Kv, Ka, and optionally
   *                     either Kg or Kcos).
   * @param startTimes   Array of dataset start times.
   * @param type         Type of analysis.
   * @param abort        Aborts analysis early if set to true from another
   *                     thread.
   */
  void SetData(const Storage& rawData, const Storage& filteredData,
               const std::string& unit, const std::vector<double>& ff,
               const std::array<units::second_t, 4>& startTimes,
               AnalysisType type, std::atomic<bool>& abort);

  /**
   * Plots all charts to the screen.
   *
   * @param grid Whether a grid view should be used. If this is set to false,
   *             then all plots will be in a single column.
   *
   * @return Whether the plots were shown (false when loading).
   */
  bool Plot(bool grid);

  /**
   * Returns a pointer to the root mean squared error of the fit.
   *
   * @return A pointer to the root mean squared error of the fit.
   */
  double* GetRMSE() { return &m_RMSE; }

 private:
  /** Stores information that is needed to plot one chart. */
  struct ChartInfo {
    // The chart title.
    std::string title;

    // Filtered and raw data.
    std::vector<ImPlotPoint> filteredData;
    std::vector<ImPlotPoint> rawData;

    // Arbitrary data storage for other data that needs to be plotted.
    std::shared_ptr<void> data;

    /** Retrieves the arbitrary data storage with the given type. */
    template <typename T>
    T* GetStorage() {
      return static_cast<T*>(data.get());
    }
  };

  // Array of chart information.
  std::array<ChartInfo, 7> m_charts;

  // Unit abbreviation.
  std::string m_abbreviation;

  // The maximum size of each vector (dataset to plot).
  static constexpr size_t kMaxSize = 2048;

  // Root mean squared error.
  double m_RMSE;

  // Thread safety
  wpi::spinlock m_mutex;

  // Logger
  wpi::Logger& m_logger;
};
}  // namespace sysid
