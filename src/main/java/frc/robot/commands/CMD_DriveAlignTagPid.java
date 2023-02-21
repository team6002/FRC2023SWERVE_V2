// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_LimeLight;

public class CMD_DriveAlignTagPid extends CommandBase {
  private SUB_Drivetrain m_drivetrain;
  private SUB_LimeLight m_limeLight;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController turnController;

  private boolean end;

  private double xSpeed, ySpeed, turnSpeed;

  public CMD_DriveAlignTagPid(SUB_Drivetrain p_drivetrain, SUB_LimeLight p_limeLight) {
    m_drivetrain = p_drivetrain;
    m_limeLight = p_limeLight;

    xController = new ProfiledPIDController(Constants.AutoAlignConstants.driveKp,
      Constants.AutoAlignConstants.driveKi,
      Constants.AutoAlignConstants.driveKd,
      Constants.AutoAlignConstants.driveConstraints);

    yController = new ProfiledPIDController(Constants.AutoAlignConstants.driveKp,
      Constants.AutoAlignConstants.driveKi,
      Constants.AutoAlignConstants.driveKd,
      Constants.AutoAlignConstants.driveConstraints);

    turnController = new ProfiledPIDController(Constants.AutoAlignConstants.turnKp,
      Constants.AutoAlignConstants.turnKi,
      Constants.AutoAlignConstants.turnKd,
      Constants.AutoAlignConstants.turnConstraints);

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

    // m_drivetrain.resetOdometryPose2d(m_limeLight.getRobotPoseInTargetSpace());

    xController.setGoal(Constants.AutoAlignConstants.goalPose.getX());
    yController.setGoal(Constants.AutoAlignConstants.goalPose.getY());
    turnController.setGoal(Constants.AutoAlignConstants.goalPose.getRotation().getDegrees());

    xController.setTolerance(.1);
    yController.setTolerance(.1);
    turnController.setTolerance(5);
  }

  @Override
  public void execute() {
    if (end) {
      return;
    }

    if (xController.atGoal() && yController.atGoal() && turnController.atGoal()) {
      end = true;
      return;
    }

    xSpeed = xController.calculate(m_limeLight.getTargetX());
    ySpeed = yController.calculate(m_limeLight.getTargetY());
    turnSpeed= turnController.calculate(m_limeLight.getTargetYaw());

    m_drivetrain.drive(xSpeed, ySpeed, turnSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return end;
  }
}
