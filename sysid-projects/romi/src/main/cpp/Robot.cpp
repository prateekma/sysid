// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Encoder.h>
#include <frc/RobotBase.h>
#include <frc/RomiGyro.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/HALBase.h>
#include <wpi/raw_ostream.h>
#include <wpi/numbers>

#include "logging/SysIdDrivetrainLogger.h"
#include "units/time.h"

/**
 * This is the main robot class for the Romi. We don't use the setup from
 * sysid-library because some vendors don't offer static libraries for desktop
 * simulation.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Set motor inversions.
    m_rMotor.SetInverted(true);

    // Set encoder inversions.
    m_lEncoder.SetDistancePerPulse(1 / m_cpr);
    m_rEncoder.SetDistancePerPulse(1 / m_cpr);
  }

  void RobotPeriodic() override {
    frc::SmartDashboard::PutNumber("Left Position", m_lEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Right Position", m_rEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Left Velocity", m_lEncoder.GetRate());
    frc::SmartDashboard::PutNumber("Right Velocity", m_rEncoder.GetRate());

    frc::SmartDashboard::PutNumber("Gyro Reading", m_gyro.GetAngle());
    frc::SmartDashboard::PutNumber("Gyro Rate", m_gyro.GetRate());
  }

  void AutonomousInit() override {
    // m_logger.InitLogging();
  }

  void AutonomousPeriodic() override {
    // Send data.
    // m_logger.Log(m_lEncoder.GetDistance(), m_rEncoder.GetDistance(),
    //              m_lEncoder.GetRate(), m_rEncoder.GetRate(),
    //              m_gyro.GetAngle() * wpi::numbers::pi / 180.0,
    //              m_gyro.GetRate() * wpi::numbers::pi / 180.0);

    // Set motor voltages.
    // m_lMotor.SetVoltage(m_logger.GetLeftMotorVoltage());
    // m_rMotor.SetVoltage(m_logger.GetRightMotorVoltage());
  }

  void DisabledInit() override {
    // Stop motors.
    m_lMotor.StopMotor();
    m_rMotor.StopMotor();

    // Send data.
    wpi::outs() << "Sending data...\n";
    // m_logger.SendData();
  }

 private:
  frc::Spark m_lMotor{0};
  frc::Spark m_rMotor{1};
  frc::Encoder m_lEncoder{4, 5};
  frc::Encoder m_rEncoder{6, 7};
  double m_cpr = 1440.0;
  double m_gearing = 1.0;
  frc::RomiGyro m_gyro;

  // SysIdDrivetrainLogger m_logger;
};

// These are defined in the respective sim module static libraries.
extern "C" {
void HALSIM_InitExtension_GUI(void);
void HALSIM_InitExtension_DS_SOCKET(void);
void HALSIM_InitExtension_WS_CLIENT(void);
}  // extern "C"

int main() {
  // Manually initialize HAL and extensions before starting the robot program.
  // It is safe to call HAL_Initialize as many times as we want.
  if (!HAL_Initialize(500, 0)) {
    wpi::errs() << "FATAL: HAL could not be initialized.\n";
  }

  // Initialize extensions.
  HALSIM_InitExtension_GUI();
  HALSIM_InitExtension_DS_SOCKET();
  HALSIM_InitExtension_WS_CLIENT();

  // Start robot program.
  return frc::StartRobot<Robot>();
}
