// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Turret;

public class CMD_TurretPrepareForClimb extends CommandBase {
  /** Homes and reset the turret then puts it in the position for climb */
  SUB_Turret m_turret;
  public CMD_TurretPrepareForClimb(SUB_Turret p_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = p_turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.turretReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setFrontPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_turret.getTurretMode() == 1);
  }
}
