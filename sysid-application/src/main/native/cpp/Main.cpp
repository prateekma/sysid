// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <wpi/EventLoopRunner.h>
#include <wpi/Logger.h>

#include "sysid/deploy/RomiSession.h"

void Application();

#ifdef _WIN32
int __stdcall WinMain(void* hInstance, void* hPrevInstance, char* pCmdLine,
                      int nCmdShow) {
#else
int main() {
#endif
  wpi::Logger logger;
  logger.SetLogger(
      [](unsigned int level, const char* file, unsigned int line,
         const char* message) { std::cout << message << std::endl; });
  logger.set_min_level(0);

  wpi::EventLoopRunner runner;

  sysid::RomiSession session{logger};
  runner.ExecSync([&session](wpi::uv::Loop& lp) { session.Execute(lp); });

  auto status = session.GetStatus();

  while (status != sysid::RomiSession::Status::kFailure &&
         status != sysid::RomiSession::Status::kSuccess) {
    status = session.GetStatus();
  }

  // Application();
}
