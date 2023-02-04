// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Wrist;

public class CMD_WristSetPosition extends CommandBase {
  SUB_Elbow m_elbow;
  SUB_Wrist m_wrist;
  double m_position;

  public CMD_WristSetPosition(SUB_Elbow p_elbow, SUB_Wrist p_wrist, double p_position) {
    m_elbow = p_elbow;
    m_wrist = p_wrist;
    m_position = p_position;
  }

  @Override
  public void initialize() {
    m_wrist.setReference(m_position);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
