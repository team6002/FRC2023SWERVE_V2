// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Elevator;

public class CMD_ElevatorSetPower extends CommandBase {
  CommandXboxController m_power;
  SUB_Elevator m_Elevator;
  public CMD_ElevatorSetPower(SUB_Elevator p_elevator, CommandXboxController p_power) {
    m_power = p_power;
    m_Elevator = p_elevator;
    addRequirements(m_Elevator);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_Elevator.setPower(-m_power.getLeftY());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
