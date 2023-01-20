// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Turret;

public class CMD_SetTurretJoystickMode extends CommandBase {
  /** Set the turret to 3 (joystick mode) */
  SUB_Turret m_turret;

  public CMD_SetTurretJoystickMode(SUB_Turret p_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = p_turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.setTurretMode(2);
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
