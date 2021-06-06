// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <wpi/Logger.h>
#include <wpi/uv/Loop.h>
#include <wpi/uv/Pipe.h>
#include <wpi/uv/Process.h>

namespace sysid {
/**
 * Represents a Romi session.
 *
 * An instance of this class must be kept alive in memory until GetStatus()
 * returns kFailure or kSuccess. Otherwise the session will fail!
 */
class RomiSession {
 public:
  /** Represents the status of the Romi session. */
  enum class Status { kInProgress, kFailure, kSuccess };

  /**
   * Constructs an instance of the Romi session.
   *
   * @param logger A reference to a logger where log messages should be sent.
   */
  explicit RomiSession(wpi::Logger& logger) : m_logger{logger} {}

  /**
   * Executes the session -- starts the Romi program.
   *
   * @param lp A reference to a libuv event loop to run the session on.
   */
  void Execute(wpi::uv::Loop& lp);

  /**
   * Kills the Romi process if it is running, no-op otherwise.
   */
  void Kill();

  /**
   * Returns the state of the Romi session.
   */
  Status GetStatus() const;

 private:
  // Logger reference where log messages will be sent.
  wpi::Logger& m_logger;

  // Romi process handle.
  std::shared_ptr<wpi::uv::Process> m_process;

  // Romi process stdout pipe.
  std::shared_ptr<wpi::uv::Pipe> m_stdoutPipe;

  // Exit code of the process running the Romi program. If the optional has no
  // value, then we don't have an exit code yet.
  std::optional<int64_t> m_exitCode;
};
}  // namespace sysid
