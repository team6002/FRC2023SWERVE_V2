// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elevator;

public class CMD_ElevatorCheck extends CommandBase {
  /** Creates a new CMD_ArmCheck. */
  SUB_Elevator m_arm;
  double m_wantedPosition;
  public CMD_ElevatorCheck(SUB_Elevator p_arm, double p_wantedPosition) {
    m_arm = p_arm;
    m_wantedPosition = p_wantedPosition;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_arm.getPosition() >= m_wantedPosition - 3 && m_arm.getPosition() <= m_wantedPosition + 3 ){
      return true;
    }
    return false;
  }
}
