// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_LimeLight;

public class CMD_DriveAlignTagPid extends CommandBase {
  SUB_Drivetrain m_drivetrain;
  SUB_LimeLight m_limeLight;

  boolean end;

  public CMD_DriveAlignTagPid(SUB_Drivetrain p_drivetrain, SUB_LimeLight p_limeLight) {
    m_drivetrain = p_drivetrain;
    m_limeLight = p_limeLight;
    end = false;
    addRequirements(m_drivetrain, m_limeLight);
  }

  @Override
  public void initialize() {
    if (!m_limeLight.hasTarget()) {
      System.out.println("no limelight targets found, bailing");
      end = true;
      return;
    }

    m_drivetrain.resetOdometryPose2d(m_limeLight.getRobotPoseInTargetSpace());
    end = true; /* TEMP */
  }

  @Override
  public void execute() {
    if (end) {
      return;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return end;
  }
}
