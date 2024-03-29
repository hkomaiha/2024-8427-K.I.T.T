// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <cameraserver/CameraServer.h>

using namespace frc;

void Robot::RobotInit()
{
    // m_backLeft.Follow(m_frontLeft);
    // m_backRight.Follow(m_frontRight);
    m_right.SetInverted(true);
    m_shooterLow.SetInverted(false);
    m_climbLeft.SetPosition(units::turn_t(0));
    m_climbRight.SetPosition(units::turn_t(0));
    m_climbLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_climbRight.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_frontLeft.SetNeutralMode(Brake);
    m_backLeft.SetNeutralMode(Brake);
    m_frontRight.SetNeutralMode(Brake);
    m_backRight.SetNeutralMode(Brake);

    autoChooser.AddOption("Drive Back", "Drive Back");
    autoChooser.AddOption("Shoot and Drive back", "Shoot and Drive back");
    autoChooser.AddOption("Just Shoot", "Just Shoot");
    autoChooser.AddOption("2 Note Playoff", "2 Note Playoff");
    


    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
    frc::CameraServer::StartAutomaticCapture();
}

void Robot::RobotPeriodic()
{
    // Regular checks to be performed every 20ms
}

void Robot::AutonomousInit()
{
    timeStamp = frc::Timer::GetFPGATimestamp();
}

void Robot::AutonomousPeriodic()
{
    std::string pickedAuto = autoChooser.GetSelected();
    if (pickedAuto == "Drive Back")
    {
        if (frc::Timer::GetFPGATimestamp() - timeStamp < 2_s)
        {
            m_drive.ArcadeDrive(-0.5, 0.0, 0.0);
        }
        else
        {
            m_drive.ArcadeDrive(0.0, 0.0, 0.0);
        }
    }
    else if (pickedAuto == "Shoot and Drive back")
    {
        // if (frc::Timer::GetFPGATimestamp() - timeStamp < 0.5_s)
        // {
        //     m_drive.ArcadeDrive(1.0, 0.0, 0.0);
        // }   
        // else
        // {
        //     m_drive.ArcadeDrive(0.0, 0.0, 0.0);
        // }

        m_shooterTop.Set(1);
        if (!firstSpeaker)
        {
            speakerTime = frc::Timer::GetFPGATimestamp();
            firstSpeaker = true;
        }
        if (((frc::Timer::GetFPGATimestamp() - speakerTime) > 2_s))
        {
            m_shooterLow.Set(1.0);
        }

        if (frc::Timer::GetFPGATimestamp() - timeStamp < 7_s && frc::Timer::GetFPGATimestamp() - timeStamp > 5_s)
        {
            m_drive.ArcadeDrive(-0.5, 0.0, 0.0);
        }
        else if (frc::Timer::GetFPGATimestamp() - timeStamp > 7_s)
        {
            m_drive.ArcadeDrive(0.0, 0.0, 0.0);
        }
    }
    else if (pickedAuto == "Just Shoot")
    {

        if (frc::Timer::GetFPGATimestamp() - timeStamp < 0.5_s)
        {
            m_drive.ArcadeDrive(1.0, 0.0, 0.0);
        }
        else
        {
            m_drive.ArcadeDrive(0.0, 0.0, 0.0);
        }

        m_shooterTop.Set(1);
        if (!firstSpeaker)
        {
            speakerTime = frc::Timer::GetFPGATimestamp();
            firstSpeaker = true;
        }
        if (((frc::Timer::GetFPGATimestamp() - speakerTime) > 2_s))
        {
            m_shooterLow.Set(1);
        }
    }
    else if (pickedAuto == "2 Note Playoff") //start of 2 note
    {
        m_shooterTop.Set(1);
        if (!firstSpeaker)
        {
            speakerTime = frc::Timer::GetFPGATimestamp();
            firstSpeaker = true;
        }
        if (((frc::Timer::GetFPGATimestamp() - speakerTime) > 2_s))
        {
            m_shooterLow.Set(1.0);
        }
        if (frc::Timer::GetFPGATimestamp()- timeStamp < 4_s){
            m_shooterLow.Set(0.0);
            m_drive.ArcadeDrive(0.0, .20, 0.0);

        }
        if (frc::Timer::GetFPGATimestamp() - timeStamp < 7_s && frc::Timer::GetFPGATimestamp() - timeStamp > 6_s)
        {
            m_drive.ArcadeDrive(-0.5, 0.0, 0.0);
        } //add code to intake and shoot
            if (((frc::Timer::GetFPGATimestamp() - speakerTime) > 9_s))
        {
            m_intake.Set(1.0);
        }
        if (frc::Timer::GetFPGATimestamp() - timeStamp < 12_s && frc::Timer::GetFPGATimestamp() - timeStamp > 11_s)
        {
            m_drive.ArcadeDrive(0.5, 0.0, 0.0);
            m_intake.Set(0.0);
        }
        if (((frc::Timer::GetFPGATimestamp() - speakerTime) > 14_s))
        {
            m_shooterLow.Set(1.0);
        }
        else if (frc::Timer::GetFPGATimestamp() - timeStamp > 17_s)//change time //if not work then change time to 10
        {
            m_drive.ArcadeDrive(0.0, 0.0, 0.0);
        }
    }

}

void Robot::TeleopInit()
{
    // Teleop initialization code
}

void Robot::TeleopPeriodic()
{
    if (driveController.GetRightTriggerAxis() > 0.5)
    {
        speedMultiplier = -1;
        turnMultiplier = -0.8;
    }
    else
    {
        speedMultiplier = -1;
        turnMultiplier = -.75;
    }

    m_drive.ArcadeDrive(((-driveController.GetLeftY()) * speedMultiplier), ((driveController.GetRightX()) * turnMultiplier));
    
    if (secondaryController.GetRightTriggerAxis() > 0.2)
    {
        if (!firstSpeaker)
        {
            speakerTime = frc::Timer::GetFPGATimestamp();
            firstSpeaker = true;
        }
        if (((frc::Timer::GetFPGATimestamp() - speakerTime) > 1_s))
        {
            m_shooterLow.Set(1);
            m_intake.Set(1);
        }
    }
    else if (secondaryController.GetRightTriggerAxis() < 0.2 && firstSpeaker)
    {
        firstSpeaker = false;
    }
    else if (secondaryController.GetRightBumper())
    {
        m_shooterTop.Set(0.20);
        if (!firstAmp)
        {
            ampTime = frc::Timer::GetFPGATimestamp();
            firstAmp = true;
        }
        if (((frc::Timer::GetFPGATimestamp() - ampTime) > 1_s))
        {
            m_shooterLow.Set(.6);
        }
    }
    else if (!(secondaryController.GetRightBumper()) && firstAmp)
    {
        firstAmp = false;
    }
    else if (secondaryController.GetLeftTriggerAxis() > 0.2)
    {
        m_shooter.Set(-.7);
    }
    else if (secondaryController.GetLeftTriggerAxis() > 0.2)
    {                        // Add a proper condition here
        m_shooter.Set(-0.6); // Positive value for forward direction
    }
    else
    {
        m_shooter.Set(0);
        m_intake.Set(0);
    }

    if (secondaryController.GetBButton())
    {
        m_intake.Set(0.8);
    }
    else if (secondaryController.GetXButton())
    {
        m_intake.Set(-1);
    }

    if (secondaryController.GetAButton())
    {
        m_climbLeft.Set(m_climbPIDController.Calculate(m_climbLeft.GetPosition().GetValueAsDouble(), 74.0));
        m_climbRight.Set(m_climbPIDController.Calculate(m_climbRight.GetPosition().GetValueAsDouble(), 74.0));
    }
    else if (secondaryController.GetYButton())
    {
        m_climbLeft.Set(m_climbPIDController.Calculate(m_climbLeft.GetPosition().GetValueAsDouble(), 0));
        m_climbRight.Set(m_climbPIDController.Calculate(m_climbRight.GetPosition().GetValueAsDouble(), 0));
    }

    else
    {
        double climbVal = (m_climbLeft.GetPosition().GetValueAsDouble() + m_climbRight.GetPosition().GetValueAsDouble()) / 2;
        m_climbLeft.Set(m_climbPIDController.Calculate(m_climbLeft.GetPosition().GetValueAsDouble(), climbVal));
        m_climbRight.Set(m_climbPIDController.Calculate(m_climbRight.GetPosition().GetValueAsDouble(), climbVal));
    }

    // else {
    //     m_shooter.Set(0);
    // }

    // frc::SmartDashboard::PutNumber("xSpeed", xSpeed);
    // frc::SmartDashboard::PutNumber("zRotation", zRotation);
    frc::SmartDashboard::PutNumber("(frc::Timer::GetFPGATimestamp()", frc::Timer::GetFPGATimestamp().value());
    frc::SmartDashboard::PutNumber("speakerTimer", speakerTime.value());
    frc::SmartDashboard::PutNumber("ampTimer", ampTime.value());
    frc::SmartDashboard::PutBoolean("firstSpeaker", firstSpeaker);
}

void Robot::DisabledInit()
{
    // Disabled initialization code
}

void Robot::DisabledPeriodic()
{
    // Code to run while robot is disabled
}

void Robot::TestInit()
{
    // Test mode initialization
}

void Robot::TestPeriodic()
{
    // Test mode periodic code
}

void Robot::SimulationInit()
{
    // Simulation initialization
}

void Robot::SimulationPeriodic()
{
    // Simulation periodic code
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}

// else if (currTime < 5){
//     m_drive.DifferentialDrive(0, 0.3);
// else if (currTime < 7) {
//     m_drive.DifferentialDrive(0.4 , 0);
// }

// if (currTime < 3){
//    m_shooterTop.Set(topShooterSpeed);
// if (!firstSpeaker){
//     currentTime = frc::Timer::GetFPGATimestamp();
//     firstSpeaker = true;
// }
// if (((frc::Timer::GetFPGATimestamp() - currentTime) > delay))
// {
//     m_shooterLow.Set(lowShooterSpeed);
// }
// }
// else if (currTime < 5){
//     m_drive.DifferentialDrive(0, -0.4);
// else if (currTime < 7) {
//     m_drive.DifferentialDrive(0.3 , 0);
// }

// if (currTime < 3){
//    m_shooterTop.Set(topShooterSpeed);
// if (!firstSpeaker){
//     currentTime = frc::Timer::GetFPGATimestamp();
//     firstSpeaker = true;
// }
// if (((frc::Timer::GetFPGATimestamp() - currentTime) > delay))
// {
//     m_shooterLow.Set(lowShooterSpeed);
// }
// }
// else if (currTime < 6) {
//     m_drive.DifferentialDrive(0 , 0.4);
// }
// else if (currTime < 7.5) {
//     m_drive.DifferentialDrive(0.3 , 0);
// }

#endif