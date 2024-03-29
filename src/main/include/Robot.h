// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/PneumaticHub.h>
#include <frc/Timer.h>
#include <frc/livewindow/LiveWindow.h>
#include "ctre/Phoenix.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/motorcontrol/PWMMotorController.h"
#include <frc/controller/PIDController.h>

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  frc::XboxController driveController{0};
  frc::XboxController secondaryController{1};
  WPI_VictorSPX m_frontLeft{11};
  WPI_VictorSPX m_backLeft{10};
  WPI_VictorSPX m_frontRight{21};
  WPI_VictorSPX m_backRight{20};

  ctre::phoenix6::hardware::TalonFX m_intake{1};

  WPI_VictorSPX m_shooterTop{31};
  WPI_VictorSPX m_shooterLow{30};

  ctre::phoenix6::hardware::TalonFX m_climbLeft{40};
  ctre::phoenix6::hardware::TalonFX m_climbRight{41};

  frc::PIDController m_climbPIDController{0.05, 0, 0};

  frc::MotorControllerGroup m_left{m_frontLeft, m_backLeft};
  frc::MotorControllerGroup m_right{m_frontRight, m_backRight};
  frc::MotorControllerGroup m_shooter{m_shooterLow, m_shooterTop};

  frc::DifferentialDrive m_drive{m_left, m_right};

  units::time::second_t speakerTime;
  units::time::second_t ampTime;

  units::time::second_t timeStamp;

  bool firstSpeaker = false;
  bool firstAmp = false;
  double speedMultiplier;
  double turnMultiplier;

  frc::SendableChooser<std::string> autoChooser;

private:
};
