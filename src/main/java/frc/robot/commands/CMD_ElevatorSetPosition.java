// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elevator;

public class CMD_ElevatorSetPosition extends CommandBase {
  SUB_Elevator m_elevator;
  double m_position;
  public CMD_ElevatorSetPosition(SUB_Elevator p_elevator, double p_position) {
    m_elevator = p_elevator;
    m_position = p_position;
  }

  @Override
  public void initialize() {
    m_elevator.setPosition(m_position);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
