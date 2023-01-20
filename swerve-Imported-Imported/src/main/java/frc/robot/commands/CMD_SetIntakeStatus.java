// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;
public class CMD_SetIntakeStatus extends CommandBase {
  /** Creates a new CMD_SetIntakeStatus. */
  FSM_IntakeStatus m_IntakeStatus;
  IntakeState m_wantedStatus;
 
  public CMD_SetIntakeStatus(FSM_IntakeStatus p_IntakeStatus, IntakeState p_wantedStatus) {
    m_IntakeStatus = p_IntakeStatus;
    m_wantedStatus = p_wantedStatus;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeStatus.setState(m_wantedStatus);
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
    return true;
  }
}
