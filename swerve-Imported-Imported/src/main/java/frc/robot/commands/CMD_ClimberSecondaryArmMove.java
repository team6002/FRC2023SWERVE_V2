// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Climber;

public class CMD_ClimberSecondaryArmMove extends CommandBase {
  /** Moves the secondary climbing arm */
  SUB_Climber m_climber;
  private double m_setpoint;
  private double m_tolerance;
  public CMD_ClimberSecondaryArmMove(SUB_Climber p_climber, double p_setpoint,
                                    double p_tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = p_climber;
    m_setpoint = p_setpoint;
    m_tolerance = p_tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setSecondaryPosition(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_climber.getSecondarySetpoint() - m_climber.getSecondaryPosition()) < m_tolerance;
  }
}
